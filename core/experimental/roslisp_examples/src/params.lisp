(defpackage :params-example
  (:use :cl :roslisp)
  (:export :main))
(in-package :params-example)

(roslisp:set-debug-level 'params :info)

(defun main ()
  "Illustrates param operations."
  (with-ros-node ("param-example")
    (if (has-param "~foo")
	(ros-info params "Param foo exists with value ~a" (get-param "~foo"))
	(ros-info params "Param foo does not exist"))

    (set-param "~foo" (if (has-param "~foo") (1+ (get-param "~foo")) 42))
    (ros-info params "Param ~~foo value is now ~a" (get-param "~foo"))))
