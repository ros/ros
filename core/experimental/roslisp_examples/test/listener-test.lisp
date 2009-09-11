(roslisp:load-message-types "std_msgs/String")
(defpackage :roslisp-listener-test
  (:use :cl :roslisp :std_msgs)
  (:export :main))
(in-package roslisp-listener-test)

(defun main ()
  (with-ros-node ("listener" :spin t)
    (advertise "chatter-echo" "std_msgs/String")
    (sleep 3)
    (subscribe "chatter" "std_msgs/String" #'(lambda (m) (publish-on-topic "chatter-echo" (make-instance '<String> :data (reverse (data-val m))))))
    ))



  
  