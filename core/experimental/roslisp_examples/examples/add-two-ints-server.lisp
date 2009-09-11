(roslisp:ros-load-service-types "roslisp_examples/AddTwoInts")
(defpackage :add-two-ints-server
  (:use :roslisp :cl :roslisp_examples)
  (:export :main))
(in-package add-two-ints-server)

(def-service-callback AddTwoInts (a-val b-val)
  (make-response :sum (+ a-val b-val)))

(defun main ()
  (with-ros-node ("two-ints-server" :spin t)
    (register-service "add_two_ints" AddTwoInts)))

