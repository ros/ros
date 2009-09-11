(roslisp:load-message-types "roslisp/ComplexMessage")
(defpackage :roslisp-complex-talker-test
  (:use :cl :roslisp :std_msgs-msg :roslisp-msg)
  (:export :main))
(in-package roslisp-complex-talker-test)

(defun main ()
  "Periodically print a complex message on the /chatter topic"
  (with-ros-node ("complex_talker")
    (advertise "complex_chatter" "roslisp/ComplexMessage")
      (loop-at-most-every .1
	 (publish-on-topic 
	  "complex_chatter"
	  (make-complex-message)))))


(defun make-complex-message ()
  (make-instance '<ComplexMessage>
		 :x #(1.1 2.2 3.3 4.4)
		 :y (make-foo-array 3)
		 :z (make-foo-array 2)
		 :w (vector (make-instance '<String> :data "foo")
			    (make-instance '<String> :data "bar"))))


(defun make-foo-array (n)
  (let ((a (make-array n)))
    (dotimes (i n a)
      (setf (aref a i)
	    (make-instance '<Foo>
			   :y (make-bar 1)
			   :z (vector (make-bar 2) (make-bar 3)))))))

(defun make-bar (i)
  (make-instance '<Bar> :y i))

						      
