(in-package roslisp)



(define-condition malformed-tcpros-header (error)
  ((msg :accessor msg :initarg :msg)))

(defun tcpros-header-assert (condition str &rest args)
  (unless condition
    (error 'malformed-tcpros-header :msg (apply #'format nil str args))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ROS Node Publication TCP server
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ros-node-tcp-server (port)
  "Return a passive socket that listens for connections on the given port"
  (let ((socket (make-instance 'inet-socket :type :stream :protocol :tcp))
	(ip-address (get-ip-address (hostname))))
    (setf (sb-bsd-sockets:sockopt-reuse-address socket) t)
    (socket-bind socket ip-address port)
    (socket-listen socket 5)
    (sb-sys:add-fd-handler (socket-file-descriptor socket)
			   :input (server-connection-handler socket))
    socket))


(defun server-connection-handler (socket)
  "Return the handler for incoming connections to this socket.  The handler accepts the connection, then reads a the topic name and MD5 sum from it, and adds the socket to its publication list for that topic.  

If topic does not exist, print error message to log (TODO: find out what TCPROS specifies in this situation)"
  #'(lambda (fd)
      (declare (ignore fd))
      (let* ((connection (socket-accept socket))
	     (stream (socket-make-stream connection :element-type '(unsigned-byte 8) :output t :input t :buffering :none)))
	(force-format *log* "Accepted TCP connection ~a" connection)
	
	(let ((header (parse-tcpros-header stream)))
	  (handler-case
	      (let ((topic (lookup-alist header "topic"))
		    (md5 (lookup-alist header "md5sum"))
		    (caller-id (lookup-alist header "callerid")))
	    
		(declare (ignore caller-id)) ;; TODO Should we be doing anything with this?

		(let ((pub (gethash topic *publications*)))
		  (tcpros-header-assert pub "unknown-topic")
		  (let ((my-md5 (string-downcase (format nil "~x" (md5sum topic)))))
		    (tcpros-header-assert (equal md5 my-md5) "md5sums do not match: ~a vs ~a" md5 my-md5)

		    ;; Now we must send back the response
		    ;; TODO send back actual type (needs code generation changes)
		    (send-tcpros-header stream "topictype" (pub-topic-type pub) "md5sum" my-md5))

		  ;; Set up connection
		  (let ((sub (make-subscriber-connection :subscriber-socket connection :subscriber-stream stream)))
		    (force-format *log* "~&Adding ~a to ~a" sub pub)
		    (push sub (subscriber-connections pub)))))

	    (malformed-tcpros-header (c)
	      (send-tcpros-header stream "error" (msg c))
	      (warn "Pub server received error ~a when trying to parse header ~a.  Ignoring this connection attempt." (msg c) header)
	      (socket-close connection)))))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Services
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun service-server (function req-type service-name)
  "Return passive socket that handles connections to a ROSRPC service by calling function."
  (let ((socket (make-instance 'inet-socket :type :stream :protocol :tcp))
	(ip-address (get-ip-address (hostname))))

    (loop
       (handler-case
	   (progn
	     (socket-bind socket ip-address *next-service-port*)
	     (socket-listen socket 5)
	     (sb-sys:add-fd-handler (socket-file-descriptor socket) :input (service-handler socket function req-type))
	     (return))
	 (address-in-use-error (c)
	   (declare (ignore c))
	   (warn "Port ~a in use when starting service ~a... trying next one." *next-service-port* service-name)
	   (incf *next-service-port*))))
    (incf *next-service-port*)
    (values socket (1- *next-service-port*))))

(defun service-handler (socket f req-type)
  #'(lambda (fd)
      (declare (ignore fd))
      (let* ((connection (socket-accept socket))
	     (stream (socket-make-stream connection :output t :input t :element-type '(unsigned-byte 8))))
	(format *log* "~&Accepted connection to service... ")
	(unwind-protect 
	
	     (let ((length (deserialize-int stream)))
	       (unless (eq length 32)
		 (error "Expected md5 sum length to be 32 but it was ~a" length))
	       (repeat 32 (read-byte stream))
	       (setf length (deserialize-int stream))
	       (let ((req (deserialize req-type stream)))
		 (let ((response (funcall f req)))
		   (write-byte 1 stream)
		   (serialize-int (serialization-length response) stream)
		   (serialize response stream)
		   (force-output stream))))

	  (socket-close connection)))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subscription
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	   


(defun setup-tcpros-subscription (hostname port topic)
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

	      ;; TODO
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
				 (ros-info (> num-dropped 0) "Dropped ~a packets on topic ~a" num-dropped topic)))))

		     (socket-close connection))))))))

      (malformed-tcpros-header (c)
	(send-tcpros-header str "error" (msg c))
	(socket-close connection)
	(error c)))))
  







;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The actual communication
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


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






(defun tcpros-call-service (hostname port req response-type)
  (check-type hostname string)
  (let ((str (tcp-connect hostname port)))
    (serialize-string (string-downcase (format nil "~x" (md5sum (class-name (class-of req))))) str)
    (serialize-int (serialization-length req) str)
    (serialize req str)

    ;; Technically, force-output isn't supposed to be called on binary streams...
    (force-output str)
    

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
    (force-format *log* "Sending TCPROS header ~a" string)
    (serialize-string string str))
  (force-output str))


(defun parse-tcpros-header (str)
  "STR is assumed to contain terms of the form FOO=BAR separated by newlines.  Return an association list between strings."
  (force-format *log* "~&Awaiting TCPROS header")
  (let ((header (deserialize-string str)))
    (force-format *log* "~&Received TCPROS header ~a" header)
    (mapcar
     #'(lambda (token)
	 (dbind (key val) (tokens token :separators '(#\=))
	   (assert (and key val) nil "Malformed key value pair ~a in ~a" token str)
	   (cons key val)))
     (tokens header :separators '(#\Newline)))))
	 


(defun lookup-alist (l key)
  (let ((pair (assoc key l :test #'equal)))
    (unless pair
      (error 'malformed-tcpros-header :msg (format nil "Could not find key ~a in ~a" key l)))
    (cdr pair)))

