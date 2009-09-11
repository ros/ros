(roslisp:load-service-types "roslisp_examples/AddTwoInts")

(defpackage :add-two-ints-client
  (:use :roslisp :cl :roslisp_examples-srv)
  (:export :main))

(in-package add-two-ints-client)


(defun add (a b)
  (sum-val (call-service "add_two_ints" 'AddTwoInts :a a :b b)))

(defun main ()
  (with-ros-node ("two-ints-client")
    (let ((a (random 1000)) (b (random 1000)))
      (format t "~a + ~a = ~a~&" a b (add a b)))))


    
    
  
