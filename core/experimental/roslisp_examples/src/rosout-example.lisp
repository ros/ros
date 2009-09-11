(defpackage :rosout-example
  (:use :cl :roslisp)
  (:export :main))
(in-package :rosout-example)

(defun main ()
  "Print messages to rosout"
  (with-ros-node ("rosout-example")
    (let ((i 0))
      (loop-at-most-every .1
	 (ros-debug nil "ros-debug ~a" (incf i))
	 (ros-info (topic subtopic) "ros-info ~a" i)
	 (ros-warn (topic subtopic2) "ros-warn ~a" i)
	 (ros-error topic "ros-error ~a" i)
	 (ros-fatal topic "ros-fatal ~a" i)
	 (ros-warn (topic2 subtopic2) (evenp i) "Conditional ros-warn ~a" i)))))



;; To modify the behavior of this without recompiling, create a file
;; roslisp_examples/bin/rosout-example:main.init.lisp containing the
;; following lines (or variants)
;;
;; (in-package :rosout-example)
;; (set-debug-levels
;;  (topic subtopic2) :error
;;  topic :info)
