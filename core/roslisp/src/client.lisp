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

(in-package :roslisp)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The operations called by client code
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



(defun advertise (topic topic-type &key (latch nil))
  "TOPIC is a string naming a ros topic
TOPIC-TYPE is either a string that equals the ros datatype of the topic (e.g. robot_msgs/Pose) or the symbol naming the message type in lisp (e.g. 'robot_msgs:<Pose>)

LATCH (defaults to nil).  If non-nil, last message sent on this topic will be sent to new subscribers upon connection.

Set up things so that publish may now be called with this topic.  Also, returns a publication object that can be used instead of the topic name when publishing."
  (declare (type (or string symbol) topic-type) (string topic))
  (ensure-node-is-running)
  (setq topic-type (lookup-topic-type topic-type))
  (with-fully-qualified-name topic
    (with-mutex (*ros-lock*)
      (or (gethash topic *publications*)
          (let ((pub (make-publication :pub-topic-type topic-type :subscriber-connections nil :is-latching latch :last-message nil)))
            (setf (gethash topic *publications*) pub)      
            (protected-call-to-master 
                ("registerPublisher" topic topic-type *xml-rpc-caller-api*) c
              (remhash topic *publications*)
              (roslisp-error "Unable to contact master at ~a for advertising ~a: ~a" *master-uri* topic c))
            (ros-debug (roslisp pub) "Advertised ~a of type ~a" topic topic-type)
            pub)))))

(defun unadvertise (topic)
  (ensure-node-is-running)
  (with-fully-qualified-name topic
    (with-mutex (*ros-lock*)
      (unless (hash-table-has-key *publications* topic)
        (roslisp-warn "Not publishing on ~a" topic))
      (remhash topic *publications*)
      (protected-call-to-master ("unregisterPublisher" topic *xml-rpc-caller-api*) c
        (ros-warn (roslisp) "Could not contact master at ~a when unregistering as publisher of ~a during shutdown: ~a" *master-uri* topic c)))))


(defmacro publish-msg (pub &rest msg-args)
  "Convenience function that first does make-msg using the type of PUB and MSG-ARGS, then publishes the resulting message on PUB"
  (let ((p (gensym)))
    `(let ((,p ,pub))
       (publish ,p (make-msg (pub-topic-type ,p) ,@msg-args)))))

(defgeneric publish (pub message)
  (:documentation "PUB is either a publication object returned by advertise, or a string naming a ros topic.  MESSAGE is the message object of the appropriate type for this topic.")
  (:method :before (pub message)
	   (ensure-node-is-running))
  (:method ((topic string) message)
    (declare (type ros-message message))
    (with-fully-qualified-name topic
      (mvbind (publication known) (gethash topic *publications*)
	(unless known 
	  (if (equal topic "/rosout")
	      (error "The topic /rosout was itself unknown")
	      (roslisp-error "Unknown topic ~a" topic)))
	(publish publication message))))
  (:method ((publication publication) message)
    ;; Latch the message
    (when (is-latching publication)
      (setf (last-message publication) message))
    
    ;; Remove closed streams
    (setf (subscriber-connections publication)
	  (delete-if #'(lambda (sub) 
			 (let ((str (subscriber-stream sub)))
			   (or (not (open-stream-p str)) (gethash str *broken-socket-streams*))))
		     (subscriber-connections publication)))

    ;; Write message to each stream
    (let ((num-written 0))
      (dolist (sub (subscriber-connections publication) num-written)
	;; TODO: TCPROS has been hardcoded in
	(incf num-written (tcpros-write message (subscriber-stream sub))))
      )))

(defun publish-on-topic (&rest args)
  "Alias for publish (backwards compatibility)"
  ;; Remove by Jan 2010
  (ros-warn roslisp "Deprecated usage: use publish instead of publish-on-topic")
  (apply #'publish args))



(defun register-service-fn (service-name function service-type)
  "service-name is a string, and is the ros name of the service.  service type is the symbol lisp type of the service (the base name of the .srv file, e.g., 'roslisp_examples:AddTwoInts).

Postcondition: the node has set up a callback for calls to this service, and registered it with the master"
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
			    :request-class req-class :md5 (md5sum req-class)))
	(protected-call-to-master ("registerService" service-name uri *xml-rpc-caller-api*) c
	  (remhash service-name *services*)
	  (roslisp-error "Socket error ~a when attempting to contact master at ~a for advertising service ~a" c *master-uri* service-name))))))

(defmacro register-service (service-name service-type)
  "Register service with the given name SERVICE-NAME (a string) of type SERVICE-TYPE (a symbol) with the master.  The callback for the service is also a function named SERVICE-TYPE.  See also register-service-fn, for if you want to use a function object as the callback."
  (when (and (listp service-type) (eq 'quote (first service-type)))
    (setq service-type (second service-type)))
  `(register-service-fn ,service-name #',service-type ',service-type))
	
(defmacro def-service-callback (service (&rest args) &body body)
  "Define a service callback named SERVICE-FN-NAME for service of type SERVICE-TYPE-NAME (a symbol, e.g 'roslisp_examples:AddTwoInts).  ARGS is a list of symbols naming particular fields of the service request object which will be available within the body.  Within the body, you may also call the function make-response.  This will make an instance of the response message type.  E.g., to make a response object with field foo=3, (make-response :foo 3).

Instead of (SERVICE-FN-NAME SERVICE-TYPE-NAME), you can just specify a symbol SERVICE-NAME, which will then be used as both."
  (let ((req (gensym))
	(response-args (gensym))
	(response-type (gensym))
	service-type-name service-fn-name)
    (etypecase service
      (list (setq service-fn-name (first service) service-type-name (second service)))
      (symbol (setq service-fn-name service service-type-name service)))

    
    `(defun ,service-fn-name (,req)
       (declare (ignorable ,req)) ;; For the case when the request object is empty
       (let ((,response-type (service-response-type ',service-type-name)))
	 (with-fields ,args ,req
	   (flet ((make-response (&rest ,response-args)
		    (apply #'make-instance ,response-type ,response-args)))
	     ,@body))))))



(defun call-service (service-name service-type &rest request-args)
  "call-service SERVICE-NAME SERVICE-TYPE &rest ARGS
or
call-service SERVICE-NAME SERVICE-TYPE REQUEST-OBJECT (happens iff length(ARGS) is 1)

SERVICE-NAME - a string that is the ROS name of the service, e.g., my_namespace/my_srv
SERVICE-TYPE - symbol or string naming the Lisp type (the basename of the .srv file), e.g. 'AddTwoInts, or the fully qualified type of the service, e.g. \"test_ros/AddTwoInts\"
REQUEST-ARGS - initialization arguments that would be used when calling make-instance to create a request object.  
REQUEST-OBJECT - the request object itself

Returns the response object from the service."

  (declare (string service-name) ((or symbol string) service-type))
  (ensure-node-is-running)
  (let* ((service-type (etypecase service-type
                         (symbol service-type)
                         (string (make-service-symbol service-type))))
         (response-type (service-response-type service-type)))
    (with-fully-qualified-name service-name
      (mvbind (host port) (parse-rosrpc-uri (lookup-service service-name))
        ;; No error checking: lookup service should signal an error if there are problems

        (let ((obj (if (= 1 (length request-args))
                       (first request-args)
                       (apply #'make-service-request service-type request-args))))

          (ros-debug (roslisp call-service) "Calling service at host ~a and port ~a with ~a" host port obj)
          (tcpros-call-service host port service-name obj response-type))))))
    

(defun wait-for-service (service-name &optional timeout)
  "wait-for-service SERVICE-NAME &optional TIMEOUT

Blocks until a service with this name is known to the ROS Master (unlike roscpp, doesn't currently check if the connection is actually valid), then returns true.
TIMEOUT, if specified and non-nil, is the maximum (wallclock) time to wait for.  If we time out, returns false."
  (ensure-node-is-running)
  (let* ((first-time t)
         (timed-out
          (nth-value 
           1 (spin-until (handler-case (progn (lookup-service service-name) t)
                           (ros-rpc-error (c) (declare (ignore c)) nil))
                 (.1 timeout)
               (when first-time
                 (setq first-time nil)
                 (ros-debug (roslisp wait-for-service) "Waiting for service ~a" service-name))))))
    (ros-debug (roslisp wait-for-service) (not timed-out) "Found service ~a" service-name)
    (ros-debug (roslisp wait-for-service) timed-out "Timed out waiting for service ~a" service-name)
    (not timed-out)))
      




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





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declaim (inline ensure-node-is-running))
(defun ensure-node-is-running ()
  (unless (eq *node-status* :running)
    (cerror "Start a dummy node" "Node status is ~a" *node-status*)
    (start-ros-node "dummy")))

(defun event-loop ()
  (loop
     ;; If node has stopped, end loop
     (unless (eq *node-status* :running) (return))

     ;; Allow the tcp server to respond to any incoming connections
     (handler-bind
         ((simple-error #'(lambda (c)
			    (with-mutex (*ros-lock*)
			      (unless (eq *node-status* :running)
				(ros-info (roslisp event-loop) "Event loop received error ~a.  Node-status is now ~a" c *node-status*)
				(return))))))
       (sb-sys:serve-all-events 1)))

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
	   

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Experimental
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro def-ros-node (name params (&key spin) &body body)
  (let ((doc-string (if (stringp (first body)) (first body) ""))
	(body (if (stringp (first body)) (rest body) body)))
    `(defun ,name () ,doc-string 
	    (with-ros-node (,(string-downcase (symbol-name name)) ,@(when spin '(:spin t)))
	      (let ,(mapcar 
		      #'(lambda (param)
			  (let ((param-name (concatenate 'string "~" (string-downcase (symbol-name param)))))
			    `(,param (get-param ,param-name))))
		      params)
		,@body)))))

(defmacro def-service-call (a service-ros-name &key return-field)
  "Convenience macro for calls to a service.  

def-service-call (FN-NAME TYPE-NAME) ROS-NAME &key RETURN-FIELD

where FN-NAME and TYPE-NAME are unevaluated symbols, ROS-NAME is a string (evaluated, so doesn't have to be a literal)

This means define a function FN-NAME that calls a ros service with the given ros-name, whose lisp type (which is the base name of the .srv file, but may have to be package qualified if you haven't imported it) is TYPE-NAME.  The function does a calls the given service using an appropriately typed request object constructed using its arguments.

The first argument can also be a symbol NAME (unevaluated), which uses NAME for both FN-NAME and TYPE-NAME

If RETURN-FIELD is provided, that field of the response is returned.  Otherwise, the entire response is returned."

  (let ((args (gensym))
	(response (gensym))
	name service-type)
    (if (listp a)
	(setq name (first a) service-type `',(second a))
	(setq name a service-type `',a))
    `(defun ,name (&rest ,args)
       (let ((,response (apply #'call-service ,service-ros-name ,service-type ,args)))
	 ,(if return-field
	      `(with-fields (,return-field) ,response
		 ,return-field)
	      response)))))