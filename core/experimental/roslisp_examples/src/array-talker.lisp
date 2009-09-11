(roslisp:load-message-types "robot_msgs/ChannelFloat32")

(defpackage :roslisp-array-talker
  (:use :cl :roslisp :robot_msgs)
  (:export :main))

(in-package roslisp-array-talker)


(defun main ()
  "Illustrates array and compound messages, and rosout functions"
  (with-ros-node ("talker")
    (advertise "array_chatter" "robot_msgs/ChannelFloat32")
    (unwind-protect

	 ;; Protected form
	 (loop-at-most-every 1
	      (let* ((l (1+ (random 5)))
		     (a (make-array l)))
		(dotimes (j l)
		  (setf (aref a j) (if (zerop j) (random 100) (1+ (aref a (1- j))))))
		(ros-info array-talker "~&About to publish an array with ~a elements" l)
		(ros-debug array-talker "~&The elements are ~a" a)
		(publish-on-topic "array_chatter" (make-instance '<ChannelFloat32> :name "foo" :vals a))))
      
      ;; Cleanup form
      (ros-fatal array-talker "Ack... dying!"))))

						      
