(in-package :roslisp-test)

(defun listener ()
  (with-ros-node ("listener" :spin t :anonymous t)
    (advertise "chatter_echo" "std_msgs/String" :latch t)
    (sleep 3)
    (subscribe "chatter" "std_msgs/String" #'(lambda (m) (with-fields (data) m (publish "chatter_echo" (make-instance '<String> :data (reverse data))))))
    ))



  
  