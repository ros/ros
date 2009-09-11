(in-package roslisp)

(ros-load-message-types "roslib/Log")


(defun rosout-msg (level args)
  ;; Code-generating function called by the macros below; not to be called by client code.
  (let ((code (symbol-code 'roslib:<Log> level)))
    (when (typep (first args) 'string) (push t args))
    (dbind (check str &rest format-args) args
      (let ((output-string (gensym)))
	`(when (and (>= ,code *debug-level*) ,check)
	   (let ((,output-string (format nil ,str ,@format-args)))
	     (with-mutex (*debug-stream-lock*)
	       (format *debug-stream* "~&~a~&" ,output-string))
	     (force-output *debug-stream*)
	     (when (eq *node-status* :running)
	       (publish-on-topic 
		"/rosout"
		(make-instance 'roslib:<Log> :name *ros-node-name*
			       :level ,code :msg ,output-string)))))))))


(defmacro ros-debug (&rest args)
  "ros-debug [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, publish output at level :debug with the given format string and arguments.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg :debug args))

(defmacro ros-info (&rest args)
  "ros-info [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, publish output at level :info with the given format string and arguments.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg :info args))

(defmacro ros-warn (&rest args)
  "ros-warn ARGS
When CONDITION is true, publish output at level :warn with the given format string and arguments.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg :warn args))

(defmacro ros-error (&rest args)
  "ros-error [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, publish output at level :error with the given format string and arguments.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg :error args))

(defmacro ros-fatal (&rest args)
  "ros-fatal [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, publish output at level :fatal with the given format string and arguments.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg :fatal args))



