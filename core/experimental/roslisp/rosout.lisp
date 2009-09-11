(in-package roslisp)

(ros-load-message-types "roslib/Log")


;; Here until the code generation for Log is fixed
(defun temp-get-symbol-code (a b)
  (declare (ignore a))
  (ecase b
    (:debug 1)
    (:info 2)
    (:warn 4)
    (:error 8)
    (:fatal 16)))


(defun rosout-msg (level args)
  (let ((code (temp-get-symbol-code 'roslib:<Log> level)))
    (dbind (check str &rest format-args) args
      (let ((output-string (gensym)))
	`(when (and (>= ,code *debug-level*) ,check)
	   (let ((,output-string (format nil ,str ,@format-args)))
	     (with-mutex (*debug-stream-lock*)
	       (format *debug-stream* "~&~a~&" ,output-string))
	     (force-output *debug-stream*)
	     #|(publish-on-topic 
	      "/rosout"
	      (make-instance 'roslib:<Log> :name *ros-node-name*
			     :level ,code
			     :msg ,output-string))|#))))))

(defmacro ros-debug (&rest args)
  (rosout-msg :debug args))

(defmacro ros-info (&rest args)
  (rosout-msg :info args))

(defmacro ros-warn (&rest args)
  (rosout-msg :warn args))

(defmacro ros-error (&rest args)
  (rosout-msg :error args))

(defmacro ros-fatal (&rest args)
  (rosout-msg :fatal args))




(defmacro roslisp-error (str &rest args)
  `(progn
     (ros-error t ,str ,@args)
     (error ,str ,@args)))