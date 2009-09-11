(in-package roslisp)

(defun fully-qualified-name (name)
  "Do the translation from a client-code-specified name to a fully qualified one.  Handles already-fully-qualified names, tilde for private namespace, unqualified names, and remapped names."
  (case (char name 0)
    (#\/ name)
    (#\~ (concatenate 'string *namespace* *ros-node-name* "/" (subseq name 1)))
    (otherwise
     (concatenate 'string *namespace* (gethash name *remapped-names* name)))))

(defmacro with-fully-qualified-name (n &body body)
  (assert (symbolp n))
  `(let ((,n (fully-qualified-name ,n)))
     ,@body))


(defun process-command-line-remappings (l)
  "Process command line remappings, including the three special cases for remapping the node name, namespace, and setting parameters.  Return alist of params to set."
  (setf *remapped-names* (make-hash-table :test #'equal))
  (let ((params nil))
    (dolist (x l params)
      (dbind (lhs rhs) x
	(cond
	  ((equal lhs "__ns") (setf *namespace* rhs))
	  ((equal lhs "__name") (setf *ros-node-name* rhs))
	  ((eql (char lhs 0) #\_) (push (cons (concatenate 'string "~" (subseq lhs 1)) 
					      (let ((rhs-val (read-from-string rhs)))
						(typecase rhs-val
						  (symbol rhs)
						  (otherwise rhs-val)))) params))
	  (t (setf (gethash lhs *remapped-names*) rhs)))))))

(defun postprocess-namespace (ns)
  "Ensure that namespace begins and ends with /"
  (unless (eql (char ns 0) #\/)
    (setf ns (concatenate 'string "/" ns)))
  (unless (eql (char ns (1- (length ns))) #\/)
    (setf ns (concatenate 'string ns "/")))
  ns)

(defun postprocess-node-name (name)
  "Trim any /'s from the node name"
  (string-trim '(#\/) name))

(defun parse-remapping (string)
  "If string is of the form FOO:=BAR, return foo and bar, otherwise return nil."
  (let ((i (search ":=" string)))
    (when i
      (values (subseq string 0 i) (subseq string (+ i 2))))))



(defun handle-command-line-arguments (name)
  "Postcondition: the variables *remapped-names*, *namespace*, and *ros-node-name* are set based on the command line arguments and the environment variable ROS_NAMESPACE as per the ros command line protocol"
  (let ((remappings
	 (mapcan #'(lambda (s) (mvbind (lhs rhs) (parse-remapping s) (when lhs (list (list lhs rhs))))) 
		 (rest sb-ext:*posix-argv*))))
    (setf *namespace* (or (sb-ext:posix-getenv "ROS_NAMESPACE") "/")
	  *ros-node-name* name)
    (let ((params (process-command-line-remappings remappings)))
      (setf *namespace* (postprocess-namespace *namespace*)
	    *ros-node-name* (postprocess-node-name *ros-node-name*))

      (ros-info "~&Node name is ~a~&Namespace is ~a~&Params are ~a~&Remappings are" *ros-node-name* *namespace* params)
      (maphash #'(lambda (k v) (format *debug-stream* "~&  ~a = ~a" k v)) *remapped-names*)
      params)))


