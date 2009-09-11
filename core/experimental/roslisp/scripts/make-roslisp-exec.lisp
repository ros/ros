(require :asdf)
(let ((p (sb-ext:posix-getenv "ROS_ROOT")))
  (unless p (error "ROS_ROOT not set"))
  (push (pathname (concatenate 'string p "/core/experimental/roslisp/asdf/")) asdf:*central-registry*))



(asdf:operate 'asdf:load-op :ros-load-manifest :verbose nil)
(let ((l (ros-load-manifest:asdf-paths-to-add (second *posix-argv*))))
  (pprint '(require :asdf))
  (pprint '(push :roslisp-standalone-executable *features*))
  (pprint `(dolist (path ',l)
	     (pushnew path asdf:*central-registry* :test #'equal)))
  (format t "~&(handler-bind ((style-warning #'muffle-warning) (warning #'print)) (asdf:operate 'asdf:load-op ~a :verbose nil))" (third *posix-argv*))
  (format t "~&(handler-bind ((style-warning #'muffle-warning) (warning #'print)) (~a))" (fourth *posix-argv*))
  (format t "~&(sb-ext:quit)~&"))
(sb-ext:quit)