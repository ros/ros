(defpackage :rosout-example
  (:use :cl :roslisp)
  (:export :main))
(in-package :rosout-example)

(defun main ()
  "Print messages to rosout"
  (with-ros-node ("rosout-example")
    (let ((i 0))
      (loop-at-most-every .1
	 (ros-debug "ros-debug ~a" (incf i))
	 (ros-info "ros-info ~a" i)
	 (ros-warn "ros-warn ~a" i)
	 (ros-error "ros-error ~a" i)
	 (ros-fatal "ros-fatal ~a" i)
	 (ros-warn (evenp i) "Conditional ros-warn ~a" i)))))

