(roslisp:load-message-types "std_msgs")
(defpackage :roslisp-talker-test
  (:use :cl :roslisp :std_msgs-msg)
  (:export :main))
(in-package roslisp-talker-test)

(defun main ()
  "Periodically print a string message on the /chatter topic"
  (with-ros-node ("talker")
    (let ((pub (advertise "chatter" "std_msgs/String" :latch nil))
	  (i 0))
      (loop-at-most-every .1
	 (publish pub (make-instance '<String> :data (format nil "foo ~a" (incf i))))))))




						      
