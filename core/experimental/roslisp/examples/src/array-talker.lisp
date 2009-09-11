(roslisp:ros-load-message-types "std_msgs/ChannelFloat32")

(defpackage :roslisp-array-talker
  (:use :cl :roslisp :std_msgs)
  (:export :main))

(in-package roslisp-array-talker)


(defun main ()
  "Illustrates array and compound messages, as well as the rosout functions"
  (with-ros-node ("talker")
    (advertise "array_chatter" "std_msgs/ChannelFloat32")
    (unwind-protect
	 (loop-at-most-every 1
	      (let* ((l (1+ (random 5)))
		     (a (make-array l)))
		(dotimes (j l)
		  (setf (aref a j) (if (zerop j) (random 100) (1+ (aref a (1- j))))))
		(ros-info t "~&About to publish an array with ~a elements" l)
		(ros-debug t "~&The elements are ~a" a)
		(publish-on-topic "array_chatter" (make-instance '<ChannelFloat32> :name "foo" :vals a))))
      
      (ros-fatal t "Ack... dying!"))))

						      
