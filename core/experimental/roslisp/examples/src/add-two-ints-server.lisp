(roslisp:ros-load-service-types "roscpp_tutorials/TwoInts")

(defpackage :add-two-ints-server
  (:use :roslisp :cl :roscpp_tutorials)
  (:export :main))

(in-package add-two-ints-server)

(defun add (req)
  (make-instance '<TwoInts-response>
   :sum (+ (a-val req) (b-val req))))
		 

(defun main ()
  (with-ros-node ("two-ints-server" :spin t)
    (register-service "add_two_ints" '<TwoInts-request> #'add)))

