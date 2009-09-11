(in-package :roslisp)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; code generation for rosout
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(load-message-types "roslib/Log")

(defun designated-list (x)
  (if (listp x) x (list x)))

(defun rosout-msg (name level args)
  (let ((code (symbol-code 'roslib:<Log> level)))
    (when (typep (first args) 'string) (push t args))
    (dbind (check str &rest format-args) args
      (let ((output-string (gensym)))
	`(when (and (>= ,code (debug-level ',(reverse (if (listp name) name (list name))))) ,check)
	   (let ((,output-string (format nil ,str ,@format-args)))
	     (with-mutex (*debug-stream-lock*)
	       (format *debug-stream* "~&[~a ~a] ~a: ~a~&" ',(designated-list name) ,level (ros-time) ,output-string)
	       )
	     (force-output *debug-stream*)
	     (when (and (eq *node-status* :running) (gethash "/rosout" *publications*))
	       (publish-on-topic 
		"/rosout"
		(make-instance 'roslib:<Log> :name *ros-node-name*
			       :level ,code :msg ,output-string)))))))))
