(in-package roslisp)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Slave API - implements XML-RPC calls to this node from
;; other nodes or from the master node.
;;
;; 1) XML-RPC calls have API-wide locking - only one can
;; be active at a time.
;; 2) The funny function names are because XML-RPC is 
;; case sensitive and Lisp symbols are not, by default.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



(defun |getPid| (caller-id)
  "getPid XML-RPC method.  Takes no arguments, returns 1 upon success."
  (declare (ignore caller-id))
  (let ((pid (sb-unix:unix-getpid)))
    (list 1 (format nil "PID is ~a" pid) pid)))



(defun |shutdown| (caller-id)
  "shutdown XML-RPC method.  Takes no arguments, shuts down the ROS node."
  ;; No need to acquire lock as shutdown acquires it
  (shutdown-ros-node)
  (list 1 (format nil "shutdown by ~a" caller-id) nil))



(defun |publisherUpdate| (caller-id topic publishers)
  "publisherUpdate XMl-RPC method.
TOPIC : string naming the topic
PUBLISHERS : list of publishers, each of which is a list (ADDRESS PORT)."
  (declare (ignore caller-id))

  (ros-info (roslisp topic) "Publisher update ~a ~a" topic publishers)
  (with-mutex (*ros-lock*)
    (update-publishers topic publishers)))
    


(defun |requestTopic| (caller-id topic protocols)
  "requestTopic XML-RPC method
TOPIC: string naming a topic
PROTOCOLS: list of protocols, each of which is a list (PROTOCOL-NAME-STRING . PROTOCOL-INBOUND-PARAMS)

If the topic is not one published by this node, return -1.
If none of the protocols supported by this node return 0.
Else return 1, msg, (PROTOCOL-NAME-STRING . PROTOCOL-OUTBOUND-PARAMS)

Notes
1. Currently only TCPROS is supported.  There are no other inbound params, and the outbound params are address, port (integers).
2. This call does not actually set up the transport over the agreed-upon protocol.  In the TCP case, the subscriber must then connect to the given address and port over TCP, and send a string containing the topic name and MD5 sum."
  (declare (ignore caller-id) (string topic) (sequence protocols))

  (with-mutex (*ros-lock*)
    (if (find "TCPROS" protocols :key #'first :test #'string=)
	(if (hash-table-has-key *publications* topic)
	    
	    ;; TCPROS-specific
	    (list 1 (format nil "ready on ~a:~a" *tcp-server-hostname* *tcp-server-port*)
		  (list "TCPROS" *tcp-server-hostname* *tcp-server-port*))

	    ;; If I don't know about this topic
	    (list -1 (format nil "Not a publisher of ~a" topic) nil))

	;; If TCPROS is not on the protocol list
	(list 0 (format nil "Protocol list ~a does not contain TCPROS" protocols) nil))))



;; Register the above operations as XML-RPC methods
(import '(|getPid| |shutdown| |publisherUpdate| |requestTopic|) 's-xml-rpc-exports)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun update-publishers (topic publishers)
  "Helper called by publisherUpdate and registerSubscription"

  ;; Sometimes this gets called with an empty string for publishers (a bug in s-xml-rpc?)
  ;; In that case, replace it with the empty list
  (when (stringp publishers)
    (if (= 0 (length (string-trim '(#\Space #\Tab #\Newline) publishers)))
	(setf publishers nil)
	(progn
	  (ros-error roslisp "In update publishers, got a string ~a rather than a list of publishers.  Skipping." publishers)
	  (return-from update-publishers))))

  ;; Get the current info for this topic
  (mvbind (subscription known) (gethash topic *subscriptions*)
    (cond
      (known 

       ;; Remove no-longer-existing publisher connections
       (setf (publisher-connections subscription)
	     (delete-if #'(lambda (conn) (not (member (uri conn) publishers :test #'equal))) 
			(publisher-connections subscription)))

       ;; Add and subscribe to new ones
       (dolist (pub publishers (list 1 "updated" 0))
	 (unless (member pub (publisher-connections subscription) :test #'equal :key #'uri)

	   ;; TCPROS-specific to assume connection consists of a socket and a stream
	   (handler-case
	       (mvbind (conn str) (subscribe-publisher pub topic)
		 (push (make-publisher-connection :publisher-socket conn :publisher-stream str :uri pub)
		       (publisher-connections subscription)))
	     (sb-bsd-sockets:connection-refused-error (c) (ros-debug (roslisp tcp) "Socket error ~a when attempting to subscribe to ~a; skipping" c pub)))
	     )))

      ((not known) (list 0 (format nil "I'm not interested in topic ~a" topic) 0)))))


(defun subscribe-publisher (uri topic)
  "Connect over XML-RPC to URI, negotiate a transport, and return the connection information.

Right now, the transport must be TCPROS and the return value is the socket."

  (ros-info (roslisp topic) "~&Subscribing to ~a at ~a" topic uri)
  (unless (hash-table-has-key *subscriptions* topic) (roslisp-error "I'm not subscribed to ~a" topic))

  (dbind (protocol address port) (ros-rpc-call uri "requestTopic" topic (list (list "TCPROS")))
    (if (string= protocol "TCPROS")
	(setup-tcpros-subscription address port topic)
	(ros-error (roslisp tcp) "Protocol ~a did not equal TCPROS... skipping connection" protocol))))




