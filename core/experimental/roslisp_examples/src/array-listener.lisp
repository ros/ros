(roslisp:load-message-types "roslisp_examples/Test")

(defpackage :roslisp-array-listener
  (:use :cl :roslisp :roslisp_examples-msg)
  (:export :main))

(in-package roslisp-array-listener)
  
(defun main ()
  "Like listener, except illustrates an array message."
  (with-ros-node ("listener" :spin t)
    (subscribe "array_chatter" '<Test> #'print)))


  
  