(defpackage :params-example
  (:use :cl :roslisp)
  (:export :main))
(in-package :params-example)

(defun main ()
  "Illustrates param operations."
  (with-ros-node ("param-example")
    (if (has-param "~foo")
	(ros-info "Param foo exists with value ~a" (get-param "~foo"))
	(ros-info "Param foo does not exist"))

    (set-param "~foo" (if (has-param "~foo") (1+ (get-param "~foo")) 42))
    (ros-info "Param ~~foo value is now ~a" (get-param "~foo"))))
