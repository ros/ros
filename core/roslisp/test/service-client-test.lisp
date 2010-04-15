(in-package :roslisp-test)

(defun service-client ()
  (with-ros-node ("test_client" :spin t)
    (let ((pub (advertise "client_status" "std_msgs/Int16" :latch t)))
      (let ((sum (if (wait-for-service "adder" 20)
		     (sum-val (call-service "adder" 'Test :m (make-msg "roslisp/TwoInts" 
								       (data a) 24 (data b) 42)))
		     10)))
	(publish pub (make-msg "std_msgs/Int16" :data sum))))))
    
