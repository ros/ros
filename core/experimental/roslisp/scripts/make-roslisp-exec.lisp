(require :asdf)
(let ((p (sb-ext:posix-getenv "ROS_ROOT")))
  (unless p (error "ROS_ROOT not set"))
  (push (pathname (concatenate 'string p "/core/experimental/roslisp/asdf/")) asdf:*central-registry*))



(asdf:operate 'asdf:load-op :ros-load-manifest :verbose nil)
(let ((l (ros-load-manifest:asdf-paths-to-add (second *posix-argv*))))
  (pprint '(require :asdf))
  (pprint '(push :roslisp-standalone-executable *features*))
  (pprint '(declaim (sb-ext:muffle-conditions sb-ext:compiler-note)))
  (pprint '(load (format nil "~a/.sbclrc-roslisp" (sb-ext:posix-getenv "HOME")) :if-does-not-exist nil))
  (pprint `(dolist (path ',l)
	     (pushnew path asdf:*central-registry* :test #'equal)))
  (pprint '(defun roslisp-debugger-hook (condition me)
	    (declare (ignore me))
	    (flet ((failure-quit (&key recklessly-p)
		     (quit :unix-status 1 :recklessly-p recklessly-p)))
	      (handler-case
		  (progn
		    (format *error-output*
			    "~&Roslisp exiting due to condition: ~a~&" condition)
		    (finish-output *error-output*)
		    (failure-quit))
		(condition ()
		  (ignore-errors)
		  (failure-quit :recklessly-p t))))))
  (pprint '(unless (sb-ext:posix-getenv "ROSLISP_BACKTRACE_ON_ERRORS")
	    (setq sb-ext:*invoke-debugger-hook* #'roslisp-debugger-hook)))

  (format t "~&(handler-bind ((style-warning #'muffle-warning) (warning #'print)) (asdf:operate 'asdf:load-op ~a :verbose nil))" (third *posix-argv*))
  (format t "~&(load (merge-pathnames ~aroslisp-init.lisp~a *load-pathname*) :if-does-not-exist nil)" #\" #\")
  (format t "~&(load (merge-pathnames ~a~a.init.lisp~a *load-pathname*) :if-does-not-exist nil)" #\" (fourth *posix-argv*) #\")
  (format t "~&(handler-bind ((style-warning #'muffle-warning) (warning #'print)) (~a))" (fourth *posix-argv*))
  (format t "~&(sb-ext:quit)~&"))
(sb-ext:quit)