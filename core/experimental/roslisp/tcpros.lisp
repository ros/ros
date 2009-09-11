(in-package roslisp)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Utility
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro bind-from-header (bindings header &body body)
  "Simplify binding a bunch of fields from a header and signaling a condition if there's a problem"
  (let ((h (gensym)))
    `(let ((,h ,header))
       (let ,(mapcar #'(lambda (binding) (list (first binding) `(lookup-alist ,h ,(second binding)))) bindings)
	 ,@body))))

(define-condition malformed-tcpros-header (error)
  ((msg :accessor msg :initarg :msg)))

(defun tcpros-header-assert (condition str &rest args)
  (unless condition
    (error 'malformed-tcpros-header :msg (apply #'format nil str args))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ROS Node connection server
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ros-node-tcp-server (port)
  "Return a passive socket that listens for connections on the given port.  The handler for incoming connections is (the function returned by) server-connection-handler."
  (let ((socket (make-instance 'inet-socket :type :stream :protocol :tcp))
	(ip-address (get-ip-address *tcp-server-hostname*)))
    (setf (sb-bsd-sockets:sockopt-reuse-address socket) t)
    (socket-bind socket ip-address port)
    (socket-listen socket 5)
    (sb-sys:add-fd-handler (socket-file-descriptor socket)
			   :input (server-connection-handler socket))
    socket))


(defun server-connection-handler (socket)
  "Return the handler for incoming connections to this socket.  The handler accepts the connection, and decides whether its a topic or service depending on whether the header has a topic field, and passes it to handle-topic-connection or handle-service connection as appropriate.  If the header cannot be parsed or lacks the necessary fields, send an error header across the socket, close it, and print a warning message on this side."
  #'(lambda (fd)
      (declare (ignore fd))
      (let* ((connection (socket-accept socket))
	     (stream (socket-make-stream connection :element-type '(unsigned-byte 8) :output t :input t :buffering :none)))
	(ros-debug "Accepted TCP connection ~a" connection)
	
	(let ((header (parse-tcpros-header stream)))
	  (handler-case
	      (if (assoc "topic" header :test #'equal)
		  (handle-topic-connection header connection stream)
		  (handle-service-connection header connection stream))
	    
	    (malformed-tcpros-header (c)
	      (send-tcpros-header stream "error" (msg c))
	      (warn "Connection server received error ~a when trying to parse header ~a.  Ignoring this connection attempt." (msg c) header)
	      (socket-close connection)))))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Topics
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun handle-topic-connection (header connection stream)
  "Handle topic connection by checking md5 sum, sending back a response header, then adding this socket to the publication list for this topic."
  (bind-from-header ((topic "topic") (md5 "md5sum")) header
    (let ((pub (gethash topic *publications*)))
      (tcpros-header-assert pub "unknown-topic")
      (let ((my-md5 (string-downcase (format nil "~x" (md5sum topic)))))
	(tcpros-header-assert (equal md5 my-md5) "md5sums do not match: ~a vs ~a" md5 my-md5)
	
	;; Now we must send back the response
	(send-tcpros-header stream "type" (pub-topic-type pub) "md5sum" my-md5))
      
      ;; Add this subscription to the list for the topic
      (let ((sub (make-subscriber-connection :subscriber-socket connection :subscriber-stream stream)))
	(ros-debug "~&Adding ~a to ~a" sub pub)
	(push sub (subscriber-connections pub))))))




(defun setup-tcpros-subscription (hostname port topic)
  "Connect to the publisher at the given address and do the header exchange, then start a thread that will deserialize messages onto the queue for this topic."
  (check-type hostname string)

  (mvbind (str connection) (tcp-connect hostname port)
    (handler-case

	(mvbind (sub known) (gethash topic *subscriptions*)
	  (assert known nil "Topic ~a unknown.  This error should have been caught earlier!" topic)
	  (let ((buffer (buffer sub))
		(topic-class-name (get-topic-class-name topic)))

	    ;; Send header and receive response
	    (send-tcpros-header str "topic" topic "md5sum" (string-downcase (format nil "~x" (md5sum topic))) "callerid" (fully-qualified-name *ros-node-name*))
	    (let ((response (parse-tcpros-header str)))

	      (when (assoc "error" response :test #'equal)
		(error "During TCPROS handshake, publisher sent error message ~a" (cdr (assoc "error" response))))

	      ;; TODO need to do something with the response, handle AnyMsg (see tcpros.py)

	      ;; Spawn a dedicated thread to deserialize messages off the socket onto the queue
	      (sb-thread:make-thread
	       #'(lambda ()
		   (block thread-block
		   (unwind-protect
			(handler-bind
			    ((error #'(lambda (c)
					(unless *break-on-socket-errors*
					  (ros-warn t "Received error ~a when reading connection to ~a:~a on topic ~a.  Connection terminated." c hostname port topic)
					  (return-from thread-block nil)))))
					  
					    
			  (loop
			     (unless (eq *node-status* :running)
			       (error "Node status is ~a" *node-status*))

			     ;; Read length (ignored)
			     (dotimes (i 4)
			       (read-byte str))
			     (let ((msg (deserialize topic-class-name str)))
			       (let ((num-dropped (enqueue msg buffer)))
				 (ros-info (> num-dropped 0) "Dropped ~a messages on topic ~a" num-dropped topic)))))
		     
		     ;; Always close the connection before leaving the thread
		     (socket-close connection))))
	       :name (format nil "Roslisp thread for subscription to topic ~a published from ~a:~a" 
			     topic hostname port)
	       ))))

      (malformed-tcpros-header (c)
	(send-tcpros-header str "error" (msg c))
	(socket-close connection)
	(error c)))))
  



(defun tcpros-write (msg str)
  (unless (gethash str *broken-socket-streams*)
    (handler-case
	(progn
	  (serialize-int (serialization-length msg) str)
	  (serialize msg str)

	  ;; Technically, force-output isn't supposed to be called on binary streams...
	  (force-output str))
      (error (c)
	(warn "Received error ~a when writing to ~a.  Skipping from now on." c str)
	(setf (gethash str *broken-socket-streams*) t)))))





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Services
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun handle-service-connection (header connection stream)
  "Handle service connection.  For now, we assume a single request, which is processed immediately in this thread."
  (bind-from-header ((md5 "md5sum") (service-name "service")) header
    (let* ((service (gethash service-name *services*))
	   (my-md5 (string-downcase (service-md5 service))))
      (tcpros-header-assert service "Unknown service")
      (tcpros-header-assert (equal md5 my-md5) "md5 sums don't match: ~a vs ~a" md5 my-md5)
      (send-tcpros-header stream "md5sum" my-md5 "callerid" *ros-node-name*
			  "type" (service-request-type-name service)
			  "request_type" (service-request-type-name service) 
			  "response_type" (service-response-type-name service))
      (handle-single-service-request stream connection (service-request-class service) 
				     (service-callback service)))))





(defun handle-single-service-request (stream connection request-class-name callback)
  ;; Read length
  (dotimes (i 4)
    (read-byte stream))
  (let* ((msg (deserialize request-class-name stream))
	 (response (funcall callback msg)))
    (unwind-protect
	 (progn
	   (write-byte 1 stream)
	   (serialize-int (serialization-length response) stream)
	   (serialize response stream)
	   (force-output stream))
      (socket-close connection))))
    




(defun tcpros-call-service (hostname port service-name req response-type)
  (check-type hostname string)
  (let ((str (tcp-connect hostname port)))
    (send-tcpros-header str "service" service-name "md5sum" (string-downcase (format nil "~x" (md5sum (class-name (class-of req))))) "callerid" *ros-node-name*)
    (parse-tcpros-header str)
    (tcpros-write req str)
    (let ((ok-byte (read-byte str)))
      (unless (eq ok-byte 1)
	(error "service-call to ~a:~a with request ~a failed" hostname port req))
      (let ((len (deserialize-int str)))
	(declare (ignore len))
	(deserialize response-type str)))))

	  


    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun send-tcpros-header (str &rest args)
  (assert (evenp (length args)))
  (let ((l args)
	(string ""))
    (while l
      (setf string (concatenate 'string string (format nil "~a=~a~%" (pop l) (pop l)))))
    (serialize-string string str))
  (force-output str))


(defun parse-tcpros-header (str)
  "STR is assumed to contain terms of the form FOO=BAR separated by newlines.  Return an association list between strings."
  (let ((header (deserialize-string str)))
    (mapcar
     #'(lambda (token)
	 (let ((tokenized (tokens token :separators '(#\=))))
	   (when (cddr tokenized)
	     (warn "Ignoring all but the first two tokens in ~a when parsing tcpros header" tokenized))
	   (cons (first tokenized) (or (second tokenized) ""))))
     (tokens header :separators '(#\Newline)))))
	 


(defun lookup-alist (l key)
  (let ((pair (assoc key l :test #'equal)))
    (unless pair
      (error 'malformed-tcpros-header :msg (format nil "Could not find key ~a in ~a" key l)))
    (cdr pair)))

