(roslisp:load-message-types "std_msgs/String")
(defpackage :roslisp-listener
  (:use :cl :roslisp :std_msgs-msg)
  (:export :main))
(in-package roslisp-listener)

(defun main ()
  (with-ros-node ("listener" :spin t)
    (subscribe "chatter" '<String> #'(lambda (m) (format t "~&~a" (data-val m))))
    ;; Second subscription on same topic
    (subscribe "chatter" "std_msgs/String" (let ((i 0)) #'(lambda (m) (when (evenp (incf i)) (format t "~&Even numbered message ~a" (data-val m))))))
    ))



  
  