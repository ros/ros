(in-package roslisp)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The operations called by client code
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun start-ros-node (name &key (xml-rpc-port 8001) (pub-server-port 7001) 
		       (master-uri (make-uri "127.0.0.1" 11311)) &allow-other-keys)
  "Start up the ROS Node with the given name and master URI.  Reset any stored state left over from previous invocations."

  (with-mutex (*ros-lock*)
    (handle-command-line-arguments name)
    (sb-thread:make-thread 
     #'(lambda ()

	 (when (eq *node-status* :running) 
	   (error "Can't start node as status already equals running.  Call shutdown-ros-node first."))


	 ;; Start publication and xml-rpc servers.  The loops are to scan for a port that isn't in use.
	 (loop
	    (handler-case 
		(progn 
		  (setf *xml-server* (start-xml-rpc-server :port xml-rpc-port))
		  (return))
	      (address-in-use-error (c)
		(declare (ignore c))
		(when *log* (warn "When starting xml-rpc-server, port ~a in use ... trying next one." xml-rpc-port))
		(incf xml-rpc-port))))

	 (loop
	    (handler-case
		(progn 
		  (setf *publication-server* (ros-node-tcp-server pub-server-port))
		  (return))
	      (address-in-use-error (c)
		(declare (ignore c))
		(when *log* (warn "When starting TCP server for publications, port ~a in use... trying next one." pub-server-port))
		(incf pub-server-port))))

  
	 (setf *publication-server-address-string* (hostname)
	       ;; The old way was to store a string of the form "127.0.0.1" but now we're using hostnames
	       ;; (format nil "~a.~a.~a.~a" (aref pub-server-address 0)(aref pub-server-address 1)
	       ;; (aref pub-server-address 2) (aref pub-server-address 3))


	       *services-to-start* (make-queue)
	       *publication-server-port* pub-server-port
	       *broken-socket-streams* (make-hash-table :test #'eq)
	       *master-uri* master-uri
	       *xml-rpc-caller-api* (format nil "http://~a:~a" (hostname) xml-rpc-port)
	       *publications* (make-hash-table :test #'equal)
	       *subscriptions* (make-hash-table :test #'equal)
	       *services* (make-hash-table :test #'equal)
	       *node-status* :running
	       )

	 ;; Finally, start the serve-event loop
	 (event-loop))
     :name "ROSLisp event loop")

    ;; There's no race condition - if this test and the following advertise call all happen before the event-loop starts,
    ;; things will just queue up
    (spin-until (eq *node-status* :running) 1))

  ;; Advertise on global rosout topic for debugging messages
  (advertise "/rosout" "roslib/Log"))


(defmacro with-ros-node (args &rest body)
  "with-ros-node ARGS &rest BODY.  
Call start-ros-node with argument list ARGS, then execute the body.  Takes care of shutting down the ROS node if the body terminates or is interrupted.  

In addition to the start-ros-node arguments, ARGS may also include the boolean argument :spin.  If this is true, after body is executed, the node will just spin forever.

Assuming spin is not true, this call will return the return value of the final statement of body."

  (dbind (name &rest a &key spin) args
    (declare (ignorable name a))
    `(unwind-protect
	  (restart-case 
	      (progn
		(start-ros-node ,@args)
		,@body
		,@(when spin `((spin-until nil 100))))
	    (shutdown-ros-node (&optional a) (ros-info t "About to shutdown~:[~; due to condition ~:*~a~]" a)))
       (shutdown-ros-node))))


(defun shutdown-ros-node ()
  "Shutdown-ros-node.  Set the status to shutdown, close all open sockets and XML-RPC servers, and unregister all publications, subscriptions, and services with master node.  Finally, if *running-from-command-line* is true, exit lisp."
  (ros-debug t "Initiating shutdown")
  (with-mutex (*ros-lock*)
    (setf *node-status* :shutdown)
    (handler-case
	(stop-server *xml-server*)
      (error (c)
	(cerror "Continue" "Error stopping xml-rpc server: ~a" c)))
    (close-socket *publication-server*)
    (do-hash (k v *services*)
      (close-socket (service-socket v)))

    ;; Unregister from publications and subscriptions and close the sockets and kill subscription threads
    (do-hash (topic pub *publications*)
      (ros-rpc-call *master-uri* "unregisterPublisher" topic *xml-rpc-caller-api*)
      (dolist (sub (subscriber-connections pub))
	(handler-case
	    (close-socket (subscriber-socket sub))
	  (sb-int:simple-stream-error (c)
	    (ros-info t "Received stream error ~a when attempting to close socket ~a.  Skipping." c (subscriber-socket sub))))))

    (do-hash (topic sub *subscriptions*)
      (ros-rpc-call *master-uri* "unregisterSubscriber" topic *xml-rpc-caller-api*)
      (terminate-thread (topic-thread sub))

      ;; No longer need this as the subscriber thread takes care of it
      #|(dolist (pub (publisher-connections sub))
      (handler-case 
      (close-socket (publisher-socket pub))
      ((or sb-int:simple-stream-error simple-error) (c)
      (warn "received error ~a when attempting to close socket ~a.  Skipping." c (publisher-socket pub)))))|#
      )

    ;; Unregister services and close sockets
    (do-hash (name s *services*)
      (let ((i (ros-rpc-call *master-uri* "unregisterService" name (service-api s))))
	(unless (eql i 1)
	  (ros-warn t "When trying to close service ~a, ~a services were closed instead of 1" name i)))
      (close-socket (service-socket s)))

    (format *log* "~&Shutdown complete")
    (when *running-from-command-line* (sb-ext:quit))))





(defun advertise (topic topic-type)
  "Set up things so that publish-on-topic may now be called with this topic"
  (with-fully-qualified-name topic
    (with-mutex (*ros-lock*)
      (when (hash-table-has-key *publications* topic)
	(error "Already publishing on ~a" topic))
  
      (setf (gethash topic *publications*) (make-publication :pub-topic-type topic-type :subscriber-connections nil))
      (ros-rpc-call *master-uri* "registerPublisher" topic topic-type *xml-rpc-caller-api*))))



(defun publish-on-topic (topic message)
  "Send message string to all nodes who have subscribed with me to this topic"
  (with-fully-qualified-name topic

    ;; It might be more efficient to have a lock per topic, but that's more work...
    (mvbind (publication known) (gethash topic *publications*)
      (unless known 
	(roslisp-error "Unknown topic ~a" topic))

      ;; Remove closed streams
      (setf (subscriber-connections publication)
	    (delete-if #'(lambda (sub) (not (open-stream-p (subscriber-stream sub))))
		       (subscriber-connections publication)))

      ;; Write message to each stream
      (dolist (sub (subscriber-connections publication))
	;; TODO: TCPROS has been hardcoded in
	(tcpros-write message (subscriber-stream sub))
	))))



(defun register-service (service-name request-type function)
  (with-fully-qualified-name service-name
    (with-mutex (*ros-lock*)
      (let ((s (gethash service-name *services*)))
	(when s (roslisp-error "Cannot create service ~a as it already exists with info ~a" service-name s))

	;; The service server needs to start in the event loop thread, so we push the info onto the list of services to start
	;; and spin until the list is cleared by the event loop
	(enqueue (list service-name request-type function) *services-to-start*)

	;; Exit only when we're sure that the event loop has handled it
	(spin-until (queue-empty *services-to-start*) .1)))))


(defmacro defservice (name type args &body body)
  (let ((req (gensym))
	(arguments (gensym)))
    `(defun ,name (,req)
       (symbol-macrolet ,(mapcar #'(lambda (a) `(,a (,a ,req))) args)
	 (flet ((make-response (&rest ,arguments)
		  (apply #'make-instance (service-response-type ',type) ,arguments)))
	   ,@body)))))


(defun call-service (service-name response-type request-type &rest request-args)
    
  ;; No locking needed for this 
  (with-fully-qualified-name service-name
    (mvbind (host port) (parse-rosrpc-uri (lookup-service service-name))
      ;; No error checking: lookup service should signal an error if there are problems
      (ros-debug t "Calling service at host ~a and port ~a" host port)
      (tcpros-call-service host port (apply #'make-instance request-type request-args) response-type))))
    


(defun subscribe (topic topic-type callback &key (max-queue-length 'infty))
  "subscribe TOPIC TOPIC-TYPE CALLBACK &key MAX-QUEUE-LENGTH 

Set up subscription to TOPIC with given type.  CALLBACK will be called on the received messages in a separate thread.  MAX-QUEUE-LENGTH is the number of messages that are allowed to queue up while waiting for CALLBACK to run, and defaults to infinity. 

Can also be called on a topic that we're already subscribed to - in this case, ignore MAX-QUEUE-LENGTH, and just add this new callback function.  It will run in the existing callback thread, so that at most one callback function can be running at a time."

  (with-fully-qualified-name topic
    (with-mutex (*ros-lock*)
      
      (if (hash-table-has-key *subscriptions* topic)
	  (let ((sub (gethash topic *subscriptions*)))
	    (assert (equal topic-type (sub-topic-type sub)) nil "Asserted topic type ~a for new subscription to ~a did not match existing type ~a" topic-type topic (sub-topic-type sub))
	    (push callback (callbacks sub)))
	  
	  ;; Else create a new thread
	  (let ((sub (make-subscription :buffer (make-queue :max-size max-queue-length) 
					:publisher-connections nil :callbacks (list callback) :sub-topic-type topic-type)))
	    (setf (gethash topic *subscriptions*) sub
		  (topic-thread sub) (sb-thread:make-thread
				      (subscriber-thread sub)
				      :name (format nil "Subscriber thread for topic ~a" topic)))
	    (update-publishers topic (ros-rpc-call *master-uri* "registerSubscriber" topic topic-type *xml-rpc-caller-api*))
	    (values))))))


(defun get-param (key)
  (with-fully-qualified-name key
    (ros-rpc-call *master-uri* "getParam" key)))

(defun set-param (key val)
  (with-fully-qualified-name key
    (ros-rpc-call *master-uri* "setParam" key val)))

(defun has-param (key)
  (with-fully-qualified-name key
    (ros-rpc-call *master-uri* "hasParam" key)))

(defun delete-param (key)
  (with-fully-qualified-name key
    (ros-rpc-call *master-uri* "deleteParam" key)))
  

(defun node-status ()
  *node-status*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun event-loop ()
  (loop
     
     ;; Really, we should just get the *ros-lock* mutex always.  But that was starving other threads (sbcl threading error?) so instead this hack
     (unless (eq *node-status* :running) (return))
     (handler-case
	 (sb-sys:serve-all-events 0)
       (simple-error (c)
	 (with-mutex (*ros-lock*)
	   (if (eq *node-status* :running)
	       (error c)
	       (progn
		 (ros-debug t "Event loop received error ~a.  Terminating, as node-status is now ~a" c *node-status*)
		 (return))))))
     
     (mvbind (item exists?) (dequeue *services-to-start*)
       (when exists?
	 (with-mutex (*ros-lock*)
	   (dbind (name request-type function) item
	     (mvbind (sock port) (service-server function request-type name)
	       (let ((uri (format nil "rosrpc://~a:~a" (hostname) port)))
		 (ros-rpc-call *master-uri* "registerService" name uri *xml-rpc-caller-api*)
		 (setf (gethash name *services*) (make-service :service-socket sock :name name :service-api uri)))))))))

  (ros-info t "Terminating ROS Node event loop"))
      

(defun subscriber-thread (sub)
  "This is the thread that takes items off the queue and performs the callback on them (as separate from the one that puts items onto the queue from the socket)"
  ;; We don't acquire *ros-lock* - the assumption is that the callback is safe to interleave with the node operations defined in the roslisp package
  (let ((q (buffer sub)))
    #'(lambda ()
	(loop
	   ;; We have to get this each time because there may be new callbacks
	   (let ((callbacks (callbacks sub)))
	     (mvbind (item exists) (dequeue-wait q)
	       (if exists
		   (dolist (callback callbacks)
		     (funcall callback item))
		   (return))))))))
	   

#|(defun topic-queue (topic)
  "Return the topic buffer, of type queue.  Should not be externally called."
  (with-fully-qualified-name topic
    (let ((sub (gethash topic *subscriptions*)))
      (if sub
	  (buffer sub)
	  (error "Unknown topic ~a" topic)))))|#


