; TODO:
;  - read / write binary bytes (possibly a socket configuration?)
;  - convert bytes to/from message objects
;  - specify port of 0, and recover the port we ended up on
;  - discover network interfaces and pick a good one

(in-package roslisp)


(defun close-socket (socket)
  "Remove all handlers from this socket and close it"
  (ros-debug (roslisp tcp) "~&Closing ~a" socket)
  (invalidate-descriptor (socket-file-descriptor socket))
  (socket-close socket))



(defun tcp-connect (hostname port)
  "Helper that connects over TCP to this host and port, and returns 1) The stream 2) The socket"
  (let ((connection (make-instance 'inet-socket :type :stream :protocol :tcp))
	(ip-address (get-ip-address hostname)))
    (ros-debug (roslisp tcp) "~&Connecting to ~a ~a" ip-address port)
    (socket-connect connection ip-address port)
    (ros-debug (roslisp tcp) "~&Successfully connected to ~a ~a" ip-address port)
    (values (socket-make-stream connection :output t :input t :element-type '(unsigned-byte 8)) connection)))

