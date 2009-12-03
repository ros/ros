(roslisp:load-message-types "std_msgs")
(defpackage :roslisp-listener-test
  (:use :cl :roslisp :std_msgs-msg)
  (:export :main))
(in-package roslisp-listener-test)

(defun main ()
  (with-ros-node ("listener" :spin t :anonymous t)
    (advertise "chatter_echo" "std_msgs/String" :latch t)
    (sleep 3)
    (subscribe "chatter" "std_msgs/String" #'(lambda (m) (publish "chatter_echo" (make-instance '<String> :data (reverse (data-val m))))))
    ))



  
  