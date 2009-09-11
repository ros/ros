;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Software License Agreement (BSD License)
;; 
;; Copyright (c) 2008, Willow Garage, Inc.
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with 
;; or without modification, are permitted provided that the 
;; following conditions are met:
;;
;;  * Redistributions of source code must retain the above 
;;    copyright notice, this list of conditions and the 
;;    following disclaimer.
;;  * Redistributions in binary form must reproduce the 
;;    above copyright notice, this list of conditions and 
;;    the following disclaimer in the documentation and/or 
;;    other materials provided with the distribution.
;;  * Neither the name of Willow Garage, Inc. nor the names 
;;    of its contributors may be used to endorse or promote 
;;    products derived from this software without specific 
;;    prior written permission.
;; 
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
;; CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
;; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
;; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
;; PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
;; COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
;; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
;; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
;; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
;; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
;; OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
;; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
;; DAMAGE.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package roslisp)

(set-debug-level 'roslisp :warn)
(set-debug-level '(roslisp top) :info)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The operations called by client code
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun start-ros-node (name &key (xml-rpc-port 8001) (pub-server-port 7001) 
		       (master-uri (make-uri "127.0.0.1" 11311) master-supplied) 
		       (anonymous nil)
		       &allow-other-keys)
  "Start up the ROS Node with the given name and master URI.  Reset any stored state left over from previous invocations.

MASTER-URI is either a string of the form http://foo:12345, or an object created using make-uri.  If MASTER-URI is not provided, use *default-master-uri* (non-nil unless explicitly set by client), and if that's nil, use the value of environment variable ROS_MASTER_URI.

ANONYMOUS, if non-nil, causes the current time to be appended to the node name (to make it unique)."

  (declare (string name) (integer xml-rpc-port pub-server-port) (type (or string uri) master-uri))
  (unless (eq *node-status* :shutdown)
    (warn "Before starting node, node-status equalled ~a instead of :shutdown.  Shutting the previous node invocation down now." *node-status*)
    (shutdown-ros-node)
    (sleep 3))
  
  (when anonymous
    (mvbind (success s ms) (sb-unix:unix-gettimeofday)
      (declare (ignore success))
      (setq name (format nil "~a-~a-~a" name ms s))))
	  
    
  (let ((params (handle-command-line-arguments name)))

    ;; Deal with the master uri.  
    (unless master-supplied
      (ros-debug (roslisp top) "Master uri was not supplied, so using default")
      (setq master-uri (or *default-master-uri* (sb-ext:posix-getenv "ROS_MASTER_URI")))
      (unless (and (stringp master-uri) (> (length master-uri) 0))
	(error "Master uri needs to be supplied either as an argument to start-ros-node, or through the environment variable ROS_MASTER_URI, or by setting the lisp variable *default-master-uri*")))

    (when (stringp master-uri)
      (mvbind (address port) (parse-uri master-uri)
	(setq master-uri (make-uri address port))))

    (symbol-macrolet ((address (uri-address master-uri)))
      (unless (parse-string-ip-address address)
       (setf address (ip-address-string (lookup-hostname-ip-address address)))))
      
    (ros-info (roslisp top) "master URI is ~a:~a" (uri-address master-uri) (uri-port master-uri))

    (with-mutex (*ros-lock*)
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
		  (ros-debug (roslisp top) "When starting xml-rpc-server, port ~a in use ... trying next one." xml-rpc-port)
		  (incf xml-rpc-port))))

	   (loop
	      (handler-case
		  (progn
		    (setq *tcp-server-hostname* (hostname)
			  *tcp-server* (ros-node-tcp-server pub-server-port))
		    (return))
		(address-in-use-error (c)
		  (declare (ignore c))
		  (ros-debug (roslisp top) "When starting TCP server for publications, port ~a in use... trying next one." pub-server-port)
		  (incf pub-server-port))))

  
	   (setq *tcp-server-port* pub-server-port
		 *broken-socket-streams* (make-hash-table :test #'eq)
		 *master-uri* master-uri
		 *service-uri* (format nil "rosrpc://~a:~a" *tcp-server-hostname* *tcp-server-port*)
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

    ;; Set params specified at command line
    (dolist (p params)
      (set-param (car p) (cdr p)))

    ;; Advertise on global rosout topic for debugging messages
    (advertise "/rosout" "roslib/Log")

    ;; Subscribe to time
    (when (member (get-param "use_sim_time" nil) '("true" 1 t) :test #'equal)
      (setq *use-sim-time* t)
      (subscribe "/time" "roslib/Time" (store-message-in *last-time*) ))))


(defmacro with-ros-node (args &rest body)
  "with-ros-node ARGS &rest BODY.  
Call start-ros-node with argument list ARGS, then execute the body.  Takes care of shutting down the ROS node if the body terminates or is interrupted.  

In addition to the start-ros-node arguments, ARGS may also include the boolean argument :spin.  If this is true, after body is executed, the node will just spin forever.

Assuming spin is not true, this call will return the return value of the final statement of body."

  (dbind (name &rest a &key spin &allow-other-keys) args
    (declare (ignorable name a))
    `(unwind-protect
	  (restart-case 
	      (progn
		(start-ros-node ,@args)
		,@body
		,@(when spin `((spin-until nil 100))))
	    (shutdown-ros-node (&optional a) (ros-info (roslisp top) "About to shutdown~:[~; due to condition ~:*~a~]" a)))
       (shutdown-ros-node))))


(defun shutdown-ros-node ()
  "Shutdown-ros-node.  Set the status to shutdown, close all open sockets and XML-RPC servers, and unregister all publications, subscriptions, and services with master node.  Finally, if *running-from-command-line* is true, exit lisp."
  (ros-debug (roslisp top) "Acquiring lock")
  (with-recursive-lock (*ros-lock*)
    (unless (eq *node-status* :shutdown)
      (ros-debug (roslisp top) "Initiating shutdown")
      (setf *node-status* :shutdown)
      (handler-case
	  (stop-server *xml-server*)
	(error (c)
	  (cerror "Continue" "Error stopping xml-rpc server: ~a" c)))
      (close-socket *tcp-server*)

      ;; Unregister from publications and subscriptions and close the sockets and kill subscription threads
      (do-hash (topic pub *publications*)
	(protected-call-to-master ("unregisterPublisher" topic *xml-rpc-caller-api*) c
	  (ros-warn (roslisp) "Could not contact master at ~a when unregistering as publisher of ~a during shutdown: ~a" *master-uri* topic c))
				  
				  
	(dolist (sub (subscriber-connections pub))
	  (handler-case
	      (close-socket (subscriber-socket sub))
	    (sb-int:simple-stream-error (c)
	      (ros-debug (roslisp top) "Received stream error ~a when attempting to close socket ~a.  Skipping." c (subscriber-socket sub))))))

      (do-hash (topic sub *subscriptions*)
	(protected-call-to-master ("unregisterSubscriber" topic *xml-rpc-caller-api*) c
	  (ros-warn (roslisp) "Could not contact master when unsubscribing from ~a during shutdown: ~a" topic c))
	(terminate-thread (topic-thread sub)))

      ;; Unregister services
      (do-hash (name s *services*)
	(let ((i (protected-call-to-master ("unregisterService" name *service-uri*) c
		   (ros-warn roslisp "During shutdown, unable to contact master to unregister service ~a: ~a" name c)
		   1)))
	  (unless (eql i 1)
	    (ros-warn (roslisp top) "When trying to close service ~a, ~a services were closed instead of 1" name i))))

      (ros-info (roslisp top) "Shutdown complete")
      (when *running-from-command-line* (sb-ext:quit)))))




(defun advertise (topic topic-type &key (latch nil))
  "TOPIC is a string naming a ros topic
TOPIC-TYPE is either a string that equals the ros datatype of the topic (e.g. robot_msgs/Pose) or the symbol naming the message type in lisp (e.g. 'robot_msgs:<Pose>)

LATCH (defaults to nil).  If non-nil, last message sent on this topic will be sent to new subscribers upon connection.

Set up things so that publish-on-topic may now be called with this topic"
  (declare (type (or string symbol) topic-type) (string topic))
  (ensure-node-is-running)
  (setq topic-type (lookup-topic-type topic-type))
  (with-fully-qualified-name topic
    (with-mutex (*ros-lock*)
      (when (hash-table-has-key *publications* topic)
	(roslisp-error "Already publishing on ~a" topic))

      (protected-call-to-master ("registerPublisher" topic topic-type *xml-rpc-caller-api*) c
	(remhash topic *publications*)
	(roslisp-error "Unable to contact master at ~a for advertising ~a: ~a" *master-uri* topic c))
      (setf (gethash topic *publications*) (make-publication :pub-topic-type topic-type :subscriber-connections nil :is-latching latch :last-message nil)))))




(defun publish (topic message)
  "TOPIC is a string naming a ros topic
MESSAGE is the message object of the appropriate type for this topic

Send message string to all nodes who have subscribed with me to this topic"
  (declare (ros-message message) (string topic))
  (ensure-node-is-running)
  (with-fully-qualified-name topic

    (mvbind (publication known) (gethash topic *publications*)
      (unless known 
	(if (equal topic "/rosout")
	    (error "The topic /rosout was itself unknown")
	    (roslisp-error "Unknown topic ~a" topic)))

      ;; Latch the message
      (when (is-latching publication)
	(setf (last-message publication) message))

      ;; Remove closed streams
      (setf (subscriber-connections publication)
	    (delete-if #'(lambda (sub) (not (open-stream-p (subscriber-stream sub))))
		       (subscriber-connections publication)))

      ;; Write message to each stream
      (dolist (sub (subscriber-connections publication))
	;; TODO: TCPROS has been hardcoded in
	(tcpros-write message (subscriber-stream sub))
	))))

(defun publish-on-topic (&rest args)
  "Alias for publish (backwards compatibility)"
  (apply #'publish args))



(defun register-service-fn (service-name function service-type)
  "Postcondition: the node has set up a callback for calls to this service, and registered it with the master"
  (declare (string service-name) (function function) (symbol service-type))
  (ensure-node-is-running)
  (with-fully-qualified-name service-name
    (with-mutex (*ros-lock*)
      (let ((info (gethash service-name *services*)))
	(when info (roslisp-error "Cannot create service ~a as it already exists with info ~a" service-name info)))

      (let ((uri *service-uri*)
	    (req-class (service-request-type service-type)))
	(setf (gethash service-name *services*)
	      (make-service :callback function :name service-name :ros-type (ros-datatype service-type)
			    :request-ros-type (ros-datatype (service-request-type service-type)) :response-ros-type (ros-datatype (service-response-type service-type))
			    :request-class req-class :md5 (string-downcase (format nil "~x" (md5sum req-class)))))
	(protected-call-to-master ("registerService" service-name uri *xml-rpc-caller-api*) c
	  (remhash service-name *services*)
	  (roslisp-error "Socket error ~a when attempting to contact master at ~a for advertising service ~a" c *master-uri* service-name))))))

(defmacro register-service (service-name service-type)
  "Register service with the given name SERVICE-NAME (a string) of type service-type (a symbol) with the master."
  `(register-service-fn ,service-name #',service-type ',service-type))
	
(defmacro def-service-callback (service-type-name (&rest args) &body body)
  "Define a service callback for service of type SERVICE-TYPE-NAME (a symbol, e.g 'roslisp_examples:AddTwoInts).  ARGS is a list of symbols naming particular fields of the service request object which will be available within the body.  Within the body, make-response will make an instance of the response object."
  (let ((req (gensym))
	(response-args (gensym))
	(response-type (gensym)))
    `(defun ,service-type-name (,req)
       (let ((,response-type (service-response-type ',service-type-name)))
	 (with-accessors ,(mapcar #'(lambda (arg) (list arg arg)) args) ,req
	   (flet ((make-response (&rest ,response-args)
		    (apply #'make-instance ,response-type ,response-args)))
	     ,@body))))))



(defun call-service (service-name service-type &rest request-args)
  "call-service SERVICE-NAME SERVICE-TYPE &rest ARGS

SERVICE-NAME - string naming the service
SERVICE-TYPE - symbol naming the service type
REQUEST-ARGS - initialization arguments that would be used when calling make-instance to create a request object.

Returns the response object from the service."

  (declare (string service-name) (symbol service-type))
  (ensure-node-is-running)
  (let ((request-type (service-request-type service-type))
	(response-type (service-response-type service-type)))
    (with-fully-qualified-name service-name
      (mvbind (host port) (parse-rosrpc-uri (lookup-service service-name))
	;; No error checking: lookup service should signal an error if there are problems
	(ros-debug (roslisp call-service) "Calling service at host ~a and port ~a" host port)
	(tcpros-call-service host port service-name (apply #'make-instance request-type request-args) response-type)))))
    



(defun subscribe (topic topic-type callback &key (max-queue-length 'infty))
  "subscribe TOPIC TOPIC-TYPE CALLBACK &key MAX-QUEUE-LENGTH 

TOPIC is a string that equals the ros name of the topic
TOPIC-TYPE is either a string equalling the ros datatype of the topic, or a symbol naming the lisp type of the messages (see advertise above).
CALLBACK is a function of a single argument.
MAX-QUEUE-LENGTH is a number.  If not provided, it defaults to infinity.

Set up subscription to TOPIC with given type.  CALLBACK will be called on the received messages in a separate thread.  MAX-QUEUE-LENGTH is the number of messages that are allowed to queue up while waiting for CALLBACK to run.

Can also be called on a topic that we're already subscribed to - in this case, ignore MAX-QUEUE-LENGTH, and just add this new callback function.  It will run in the existing callback thread for the topic, so that at most one callback function can be running at a time."
  
  (declare (string topic) (type (or symbol string) topic-type) (function callback))
  (ensure-node-is-running)
  (setq topic-type (lookup-topic-type topic-type))
  (with-fully-qualified-name topic
    (with-mutex (*ros-lock*)
      
      (if (hash-table-has-key *subscriptions* topic)

	  ;; If already subscribed to topic, just add a new callback
	  (let ((sub (gethash topic *subscriptions*)))
	    (unless (equal topic-type (sub-topic-type sub))
	      (roslisp-error "Asserted topic type ~a for new subscription to ~a did not match existing type ~a" topic-type topic (sub-topic-type sub)))
	    (push callback (callbacks sub)))
	  
	  ;; Else create a new thread
	  (let ((sub (make-subscription :buffer (make-queue :max-size max-queue-length) 
					:publisher-connections nil :callbacks (list callback) :sub-topic-type topic-type)))
	    (setf (gethash topic *subscriptions*) sub
		  (topic-thread sub) (sb-thread:make-thread
				      (subscriber-thread sub)
				      :name (format nil "Subscriber thread for topic ~a" topic)))
	    (update-publishers topic
			       (protected-call-to-master ( "registerSubscriber" topic topic-type *xml-rpc-caller-api*) c
				 (roslisp-error "Could not contact master at ~a when registering as subscriber to ~a: ~a" *master-uri* topic c)))
	    (values))))))


(defun get-param (key &optional (default nil default-supplied))
  "get-param KEY &optional DEFAULT.  

KEY is a string naming a ros parameter.

Looks up parameter on parameter server.  If not found, use default if provided, and error otherwise."
  (declare (string key))
  (ensure-node-is-running)
  (with-fully-qualified-name key
    (if (has-param key)
	(protected-call-to-master ("getParam" key) c
	    (roslisp-error "Could not contact master when getting param ~a: ~a" key c))
	(if default-supplied
	    default
	    (roslisp-error "Param ~a does not exist, and no default supplied" key)))))

(defun set-param (key val)
  "set-param KEY VAL

KEY is a string naming a ros parameter.
VAL is a string or integer.

Set the parameters value on the parameter server"

  (declare (string key))
  (ensure-node-is-running)
  (with-fully-qualified-name key
    (protected-call-to-master ("setParam" key val) c
      (roslisp-error "Could not contact master at ~a when setting param ~a" *master-uri* key))))

(defun has-param (key)
  "KEY is a string naming a ros parameter

Return true iff this parameter exists on the server"
  (declare (string key))
  (ensure-node-is-running)
  (with-fully-qualified-name key
    (protected-call-to-master ("hasParam" key) c
      (roslisp-error "Could not contact master at ~a for call to hasParam ~a: ~a" *master-uri* key c))))

(defun delete-param (key)
  "KEY is a string naming a ros parameter
Remove this key from parameter server"
  (declare (string key))
  (ensure-node-is-running)
  (with-fully-qualified-name key
    (protected-call-to-master ("deleteParam" key) c
      (roslisp-error "Could not contact master at ~a when deleting param ~a: ~a" *master-uri* key c))))

      


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ensure-node-is-running ()
  (unless (eq *node-status* :running)
    (cerror "Start a dummy node" "Node status is ~a" *node-status*)
    (start-ros-node "dummy")))

(defun event-loop ()
  (loop
     ;; If node has stopped, end loop
     (unless (eq *node-status* :running) (return))

     ;; Allow the tcp server to respond to any incoming connections
     (handler-case
	 (sb-sys:serve-all-events 1)
       (simple-error (c)
	 (with-mutex (*ros-lock*)
	   (if (eq *node-status* :running)
	       (error c)
	       (progn
		 (ros-info (roslisp event-loop) "Event loop received error ~a.  Node-status is now ~a" c *node-status*)
		 (return)))))))

  (ros-info (roslisp event-loop) "Terminating ROS Node event loop"))
      

(defun subscriber-thread (sub)
  "This is the thread that takes items off the queue and performs the callback on them (as separate from the one that puts items onto the queue from the socket)"
  ;; We don't acquire *ros-lock* - the assumption is that the callback is safe to interleave with the node operations defined in the roslisp package
  (declare (type subscription sub))
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
pp  (with-fully-qualified-name topic
    (let ((sub (gethash topic *subscriptions*)))
      (if sub
	  (buffer sub)
	  (error "Unknown topic ~a" topic)))))|#


