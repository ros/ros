(roslisp:load-message-types "roslisp")
(defpackage :roslisp-complex-listener-test
  (:use :cl :roslisp :std_msgs-msg :roslisp-msg)
  (:export :main))
(in-package :roslisp-complex-listener-test)

(defun main ()
  (with-ros-node ("complex_listener" :spin t :anonymous t)
    (advertise "complex_chatter_echo" "roslisp/ComplexMessage")
    (sleep 3)
    (subscribe "complex_chatter" "roslisp/ComplexMessage" #'(lambda (m) (publish "complex_chatter_echo" m)))))



  
  