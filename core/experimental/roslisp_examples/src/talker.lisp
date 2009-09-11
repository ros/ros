(roslisp:load-message-types "std_msgs/String")
(defpackage :roslisp-talker
  (:use :cl :roslisp :std_msgs)
  (:export :main))
(in-package roslisp-talker)

(defun main ()
  "Periodically print a string message on the /chatter topic"
  (with-ros-node ("talker")
    (advertise "chatter" "std_msgs/String")
    (let ((i 0))
      (loop-at-most-every .1
	 (publish-on-topic "chatter" (make-instance '<String> :data (format nil "foo ~a" (incf i))))))))




						      
