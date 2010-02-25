(in-package :roslisp-test)

(defun complex-talker ()
  (with-ros-node ("complex_talker")
    (let ((ping-pub (advertise "ping" "roslisp/ComplexMessage"))
	  (pang-pub (advertise "pang" "roslisp/ComplexMessage")))
      (subscribe "pong" "roslisp/ComplexMessage" 
		 #'(lambda (m) (publish pang-pub m)))
      (loop-at-most-every .1
	 (publish ping-pub (make-complex-message))))))

(defvar *c* 0)

(defun make-complex-message ()
  (incf *c*)
  (make-instance '<ComplexMessage>
		 :x #(1.1 2.2 3.3 4.4)
		 :y (make-foo-array 3)
		 :z (make-foo-array 2)
		 :ind (mod *c* (* 2 4))
		 :w (vector (make-instance '<String> :data "foo")
			    (make-instance '<String> :data "bar"))
		 :qux (evenp *c*)))


(defun make-foo-array (n)
  (let ((a (make-array n)))
    (dotimes (i n a)
      (setf (aref a i)
	    (make-instance '<Foo>
			   :y (make-bar 1)
			   :z (vector (make-bar 2) (make-bar 3)))))))

(defun make-bar (i)
  (make-instance '<Bar> :y i))

						      

