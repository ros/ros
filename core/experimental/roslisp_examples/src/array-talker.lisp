(roslisp:load-message-types "roslisp_examples/Test")
(roslisp:load-message-types "roslisp_examples/Point")

(defpackage :roslisp-array-talker
  (:use :cl :roslisp :roslisp_examples-msg)
  (:export :main))

(in-package roslisp-array-talker)


(defun main ()
  "Illustrates array and compound messages, and rosout functions"
  (with-ros-node ("talker")
    (advertise "array_chatter" '<Test>)
    
    (loop-at-most-every 1
	 (let ((s (random 10))
	       (p (make-instance '<Point> :x 1 :y 2 :z 42)))
	   (publish-on-topic "array_chatter" 
			     (make-instance '<Test> :location (vector p p)
					    :orientation (vector (* 2 s) s s)))))))
						      
