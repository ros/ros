(in-package :roslisp-test)

(defun complex-listener ()
  (with-ros-node ("complex_listener" :spin t :anonymous t)
    (advertise "complex_chatter_echo" "roslisp/ComplexMessage")
    (sleep 3)
    (subscribe "complex_chatter" "roslisp/ComplexMessage" #'(lambda (m) (publish "complex_chatter_echo" m)))))



  
  