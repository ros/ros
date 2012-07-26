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

(defparameter *tcp-timeout* 5.0 "How many seconds to wait until giving up")

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
        (ip-address #(0 0 0 0)))
    (setf (sb-bsd-sockets:sockopt-reuse-address socket) t)
    (socket-bind socket ip-address port)
    (ros-debug (roslisp tcp) "Bound tcp listener ~a" socket)
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
        (flet ((close-connection (&key (abort t))
               "Closes the connection when an error occurred."
               ;; We first close the stream with the abort parameter
               ;; since SOCKET-CLOSE does not allow to specify
               ;; abort. This function is ment to be used in error
               ;; handling since SOCKET-CLOSE currently has a nasty
               ;; bug that prevents it from closing broken
               ;; connections.
               (close stream :abort abort)
               (socket-close connection)))
          (ros-debug (roslisp tcp) "Accepted TCP connection ~a" connection)
        
          (mvbind (header parse-error) (ignore-errors (parse-tcpros-header stream))
            ;; Any errors guaranteed to be handled in the first cond clause

            (ros-debug (roslisp tcp) "Parsed header: ~a ~:[~;Parse error ~:*~a~]" header parse-error)
            (handler-case
                (cond
                  ((null header)
                   (ros-info (roslisp tcp) "Ignoring connection attempt due to error parsing header: '~a'" parse-error)
                   (socket-close connection))
                  ((assoc "service" header :test #'equal)
                   (handle-service-connection header connection stream))
                  ((equal (cdr (assoc "probe" header :test #'equal)) "1")
                   (ros-warn roslisp "Unexpectedly received a tcpros connection with probe set to 1.  Closing connection.")
                   (socket-close connection))
                  ((assoc "topic" header :test #'equal)
                   (handle-topic-connection header connection stream))
                  )
              (malformed-tcpros-header (c)
                (send-tcpros-header stream "error" (msg c))
                (warn "Connection server received error ~a when trying to parse header ~a.  Ignoring this connection attempt." (msg c) header)
                (close-connection))
              (stream-error (c)
                (declare (ignore c))                
                (ros-debug (roslisp tcp) "stream error on connection to service client (could be a probe)")
                (close-connection))))))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Topics
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun handle-topic-connection (header connection stream)
  "Handle topic connection by checking md5 sum, sending back a response header, then adding this socket to the publication list for this topic."
  (bind-from-header ((topic "topic") (md5 "md5sum") (uri "callerid")) header
    (let ((pub (gethash topic *publications*)))
      (tcpros-header-assert pub "unknown-topic")
      (let ((my-md5 (md5sum topic)))
        (tcpros-header-assert (or (equal md5 "*") (equal md5 my-md5)) "md5sums do not match for ~a: ~a vs ~a" topic md5 my-md5)
        
        ;; Now we must send back the response
        (send-tcpros-header stream 
                            "type" (ros-datatype topic)
                            "callerid" (caller-id)
                            "message_definition"  (message-definition topic)
                            "latching" (if (is-latching pub) "1" "0")
                            "md5sum" my-md5))
      
      ;; Add this subscription to the list for the topic
      (let ((sub (make-subscriber-connection :subscriber-socket connection :subscriber-stream stream 
                                             :subscriber-uri uri)))
        (ros-debug (roslisp tcp) "~&Adding ~a to ~a for topic ~a" sub pub topic)
        (push sub (subscriber-connections pub))

        (when (and (is-latching pub) (last-message pub))
          (ros-debug (roslisp tcp) "~&Resending latched message to new subscriber")
          (tcpros-write (last-message pub) stream))))))



(defparameter *setup-tcpros-subscription-max-retry* 3)

(defun setup-tcpros-subscription (hostname port topic)
  "Connect to the publisher at the given address and do the header exchange, then start a thread that will deserialize messages onto the queue for this topic."
  (check-type hostname string)
  (dotimes (retry-count *setup-tcpros-subscription-max-retry* (error 'simple-error :format-control "Timeout when
    trying to communicate with publisher ~a:~a for topic ~a, check publisher node
    status. Change *tcp-timeout* to increase wait-time."
                                     :format-arguments (list hostname
                                     port topic)))
    (when (> retry-count 0) (ros-warn (roslisp tcpros) "Failed to communicate
      with publisher ~a:~a for topic ~a, retrying: ~a" hostname port
      topic retry-count))
     (handler-case
        (return
  (mvbind (str connection) (tcp-connect hostname port)
    (ros-debug (roslisp tcp) "~&Successfully connected to ~a:~a for topic ~a" hostname port topic)
    (handler-case

        (mvbind (sub known) (gethash topic *subscriptions*)
          (assert known nil "Topic ~a unknown.  This error should have been caught earlier!" topic)
          (let ((buffer (buffer sub))
                (topic-class-name (get-topic-class-name topic)))

            ;; Send header and receive response
            (send-tcpros-header str 
                                "topic" topic 
                                "md5sum" (md5sum topic) 
                                "type" (ros-datatype topic)
                                "callerid" (caller-id))
            (let ((response (with-function-timeout *tcp-timeout* (lambda () (parse-tcpros-header str)))))

              (when (assoc "error" response :test #'equal)
                (roslisp-error "During TCPROS handshake, publisher sent error message ~a" (cdr (assoc "error" response :test #'equal))))

              ;; TODO need to do something with the response, handle AnyMsg (see tcpros.py)

              ;; Spawn a dedicated thread to deserialize messages off the socket onto the queue
              (let ((connection-thread
                     (sb-thread:make-thread
                      #'(lambda ()
                          (block thread-block
                            (unwind-protect
                                 (handler-bind
                                     ((error #'(lambda (c)
                                                 (unless *break-on-socket-errors*
                                                   (ros-debug (roslisp tcp) 
                                                             "Received error ~a when reading connection to ~a:~a on topic ~a.  Connection terminated." 
                                                             c hostname port topic)
                                                   (return-from thread-block nil)))))
                                   
                                   
                                   (loop
                                      (unless (eq *node-status* :running)
                                        (error "Node status is ~a" *node-status*))

                                      ;; Read length (ignored)
                                      (dotimes (i 4)
                                        (read-byte str))


                                      (let ((msg (deserialize topic-class-name str)))

                                        (let ((num-dropped (enqueue msg buffer)))
                                          (ros-debug (roslisp tcp) (> num-dropped 0) "Dropped ~a messages on topic ~a" num-dropped topic)))))
                              
                              ;; Always close the connection before leaving the thread
                              (socket-close connection))))
                      :name (format nil "Roslisp thread for subscription to topic ~a published from ~a:~a" 
                                    topic hostname port)
                      )))
                (assert (eq (mutex-owner *ros-lock*) *current-thread*)
                        nil "Code assumption violated; not holding lock in setup-tcpros-subscription")
                (ros-debug (roslisp deserialization-thread) "Adding deserialization thread for connection on topic ~a to ~a:~a" topic hostname port)
                (push connection-thread *deserialization-threads*)))))

      (malformed-tcpros-header (c)
        (send-tcpros-header str "error" (msg c))
        (socket-close connection)
        (error c)))))
  (function-timeout () ;;just retry
      nil))) )


(defvar *stream-error-in-progress* nil)

(defun tcpros-write (msg str)
  (or
   (unless (gethash str *broken-socket-streams*)
     (handler-case
         ;; We need to serialize the data first to a string stream and
         ;; then send the whole string at once over the socket. We
         ;; also need to prevent the send operation from
         ;; interrupts. Otherwise, when messages do not get sent
         ;; completely, we run out of sync and the connection to the
         ;; client will be lost.
         (let* ((msg-size (serialization-length msg))
                (data-strm (make-instance 'msg-serialization-stream :buffer-size (+ msg-size 4))))
           (serialize-int msg-size data-strm)
           (serialize msg data-strm)
           (sb-sys:without-interrupts
             (write-sequence (serialized-message data-strm) str :end (file-position data-strm))
             ;; Technically, force-output isn't supposed to be called on binary streams...
             (force-output str)
             1 ;; Returns number of messages written 
             ))
       ((or sb-bsd-sockets:socket-error stream-error) (c)
         (unless *stream-error-in-progress*
           (let ((*stream-error-in-progress* t))
             (ros-debug (roslisp tcp) "Received error ~a when writing to ~a.  Skipping from now on." c str)))
         (setf (gethash str *broken-socket-streams*) t)
         0)))
   0))





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Services
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun handle-service-connection (header connection stream)
  "Handle service connection.  For now, we assume a single request, which is processed immediately in this thread."
  (bind-from-header ((md5 "md5sum") (service-name "service")) header
    (let* ((service (gethash service-name *services*))
           (is-probe (equal (cdr (assoc "probe" header :test #'equal)) "1")))
      (tcpros-header-assert service "Unknown service")
      (let ((my-md5 (string-downcase (service-md5 service))))
        (tcpros-header-assert (or (equal md5 "*") (equal md5 my-md5)) "md5 sums don't match for ~a: ~a vs ~a" service-name md5 my-md5)
        (send-tcpros-header stream "md5sum" my-md5 "callerid" (caller-id)
                            "type" (service-ros-type service)
                            "request_type" (service-request-ros-type service) 
                            "response_type" (service-response-ros-type service)))
      (unwind-protect
           (unless is-probe
             (handle-single-service-request stream (service-request-class service) 
                                            (service-callback service)))
        (sb-thread:make-thread
         #'(lambda ()
             (sleep 10.0)
             (ros-debug (roslisp service) "In service connection cleanup")
             (when (socket-open-p connection)
               (ros-debug (roslisp service) "Connection for call to ~a still open after 10 seconds; closing" 
                          service-name)
               (socket-close connection))))))))



(define-condition service-error (simple-error) ())

(defun handle-single-service-request (stream request-class-name callback)
  ;; Read length
  (dotimes (i 4)
    (read-byte stream))
  (flet ((write-service-error (msg)
           (assert (stringp msg))
           (write-byte 0 stream)
           (serialize-string msg stream)
           (finish-output stream)))
  (let ((msg (deserialize request-class-name stream)))
    (ros-debug (roslisp service tcp) "Deserialized service request of type ~a" request-class-name)
      (handler-case
    (let ((response (funcall callback msg)))
      (ros-debug (roslisp service tcp) "Callback returned")
      (write-byte 1 stream)
      (serialize-int (serialization-length response) stream)
      (ros-debug (roslisp service tcp) "Wrote response length ~a" (serialization-length response))
      (serialize response stream)
      (ros-debug (roslisp service tcp) "Wrote response; flushing stream.")
      (finish-output stream)
            (ros-debug (roslisp service tcp) "Finished handling service request"))
        (service-error (e)
          (let ((msg (apply #'format nil
                            (simple-condition-format-control e)
                            (simple-condition-format-arguments e))))
            (ros-debug (roslisp service tcp) "Service-error during request ~a:~% ~a" e msg)
            (write-service-error
             (concatenate 'string "service cannot process request: " msg))))
        (error (e)
          (let ((msg (format nil "~a" e)))
            (ros-error (roslisp service tcp) "Error processing request ~a:~% ~a" e msg)
            (write-service-error
             (concatenate 'string "error processing request: " msg))))))))


(defun tcpros-establish-service-connection (hostname port service-name request-class &optional (persistent nil))
  (check-type hostname string)
  (multiple-value-bind (stream socket) (tcp-connect hostname port)
    (handler-bind
        ((error (lambda (e)
                  (declare (ignore e))
                  (socket-close socket))))
      (send-tcpros-header
       stream
       "service" service-name
       "md5sum" (md5sum request-class) 
       "callerid" (caller-id)
       "persistent" (if persistent "1" "0")))
    (values
     stream socket
     (with-function-timeout *tcp-timeout* (lambda () (parse-tcpros-header stream))))))

(define-condition service-call-error (error)
  ((message :initarg :message :reader service-call-error-message)))

(defun tcpros-do-service-request (stream request response-type)
  (tcpros-write request stream)
  (let ((ok-byte (read-byte stream)))
    (unless (eq ok-byte 1)
      (error 'service-call-error
             :message (handler-case
                          (deserialize-string stream)
                        ;; TODO(lorenz): don't catch all errors.
                        (error nil))))
    (let ((len (deserialize-int stream)))
      (declare (ignore len))
      (prog1
          (deserialize response-type stream)
        (assert (not (listen stream)) () "Still bytes in the stream. It seems like we went out of sync.")))))

(defun tcpros-call-service (hostname port service-name req response-type)
  (check-type hostname string)
  (dotimes (retry-count *setup-tcpros-subscription-max-retry* (error 'simple-error
                                                                     :format-control "Timeout when
    trying to communicate with ~a:~a for service ~a, check service node
    status. Change *tcp-timeout* to increase wait-time."
                                                                     :format-arguments (list hostname port
                                                                                             service-name)))
    (when (> retry-count 0) (ros-warn (roslisp tcpros) "Failed to communicate
      with ~a:~a for service-name ~a, retrying: ~a" hostname port
      service-name retry-count))
    (handler-case
        (return
          (multiple-value-bind (str socket)
              (tcpros-establish-service-connection hostname port service-name (class-name (class-of req)))
            (unwind-protect
                 (handler-case
                     (tcpros-do-service-request str req response-type)
                   (service-call-error (e)
                     (if (service-call-error-message e)
                         (roslisp-error "service-call to ~a:~a with request ~a failed with message: ~a"
                                        hostname port req (service-call-error-message e))
                         (roslisp-error "service-call to ~a:~a with request ~a failed."
                                        hostname port req))))
              (socket-close socket))))
      (function-timeout () ;;just retry
        nil))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun caller-id ()
  "The callerid field of any tcpros header used by a node"
  (fully-qualified-name *ros-node-name*))

(defun send-tcpros-header (str &rest args)
  (assert (evenp (length args)) nil "send-tcpros-header received odd number of arguments ~a" args)
  (let ((l args)
        (key-value-pairs nil)
        (total-length 0))

    (while l
      (let ((next-pair (format nil "~a=~a" (pop l) (pop l))))
        (incf total-length (+ 4 (length next-pair))) ;; 4 for the 4-byte length at the beginning
        (push next-pair key-value-pairs)))

    (ros-debug (roslisp tcp header) "Sending tcpros header ~a" key-value-pairs)
    (serialize-int total-length str)
    (dolist (pair key-value-pairs)
      (serialize-string pair str)))
  (force-output str))


(defun parse-tcpros-header (str)
  (let ((remaining-length (deserialize-int str))
        (key-value-pairs nil))
    (while (> remaining-length 0)
      (let ((field-string (deserialize-string str)))
        (decf remaining-length (+ 4 (length field-string))) ;; 4 for the length at the beginning
        (unless(>= remaining-length 0) 
          (roslisp-error "Error parsing tcpros header: header length and field lengths didn't match"))
        
        (push (parse-header-field field-string) key-value-pairs)
        ))
    (ros-debug (roslisp tcp header) "Received tcpros header ~a" key-value-pairs)
    key-value-pairs))

(defun parse-header-field (field-string)
  (let ((first-equal-sign-pos (position #\= field-string)))
    (if first-equal-sign-pos
        (cons (subseq field-string 0 first-equal-sign-pos)
              (subseq field-string (1+ first-equal-sign-pos)))
        (roslisp-error "Error parsing tcpros header field ~a: did not contain an '='"
                       field-string))))


(defun lookup-alist (l key)
  (let ((pair (assoc key l :test #'equal)))
    (unless pair
      (error 'malformed-tcpros-header :msg (format nil "Could not find key ~a in ~a" key l)))
    (cdr pair)))

