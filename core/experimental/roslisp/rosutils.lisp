(in-package roslisp)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ROSLisp-specific utility code
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lookup-service (name)
  (ros-rpc-call *master-uri* "lookupService" name))



(defun ros-rpc-call (uri name &rest args)
  "ros-rpc-call XML-RPC-SERVER-URI CALL-NAME &rest CALL-ARGS.  Preprends the ros node name to the arg list and does the call.  Throws a continuable error if a code <= 0 is returned.  Otherwise, return the values."
  (mvbind (address port) (parse-uri uri)
  
    (dbind (code msg vals)
	(xml-rpc-call 
	 (apply #'encode-xml-rpc-call name 
		(concatenate 'string "/" *ros-node-name*) ;; TODO: this assumes global namespace
		args) 
	 :host address :port port)
      (when (<= code 0) (cerror "Ignore and continue" "XML RPC call ~a to ~a failed with code ~a, message ~a, and values ~a" (cons name args) uri code msg vals))
      vals)))

(defun parse-uri (uri)
  "parse a uri struct or uri string of the form http://address:port.  Return two values: address and port."
  (etypecase uri
    (string (let ((tokens (tokens uri :separators (list #\: #\/))))
	      (unless (and (= (length tokens) 3)
			   (string= (first tokens) "http"))
		(error "Malformed URI ~a" uri))
	      (values (second tokens) (read-from-string (third tokens)))))
    (uri (values (uri-address uri) (uri-port uri)))))



(defun parse-rosrpc-uri (uri)
  (let ((tokens (tokens uri :separators (list #\/ #\:))))
    (unless (and (= (length tokens) 3)
		 (string= (first tokens) "rosrpc"))
      (error "URI ~a was not of the expected form" uri))

    (values (second tokens)
	    (read-from-string (third tokens)))))

	

(defmacro roslisp-error (str &rest args)
  `(progn
     (ros-error t ,str ,@args)
     (error ,str ,@args)))

(defun get-ip-address (hostname)
  "Map from hostname into a vector of the form #(127 0 0 1)"
  (let ((address-vector
	 (or (parse-string-ip-address hostname)
	     (cond
	       ((member hostname '("localhost" "127.0.0.1") :test #'equal) #(127 0 0 1))
	       ((and (typep hostname '(vector * 4)) (every #'numberp hostname)) hostname)
	       (t (let ((address (lookup-hostname-ip-address hostname)))
		    (etypecase address
		      (string (let ((tokens (tokens address :separators (list #\.))))
				(map 'vector #'(lambda (token) (read-from-string token)) tokens)))
		      (vector address)
		      (list (coerce address 'vector)))))))))
    (if (and (typep address-vector '(vector * 4)) (every #'(lambda (x) (typep x '(mod 256))) address-vector))
	address-vector
	(error "Converting ~a into ip address vector resulted in ~a, which doesn't look right" hostname address-vector))))


(defun parse-string-ip-address (hostname)
  "If string can be parsed as 4 numbers separated by .'s return vector of those numbers, else return nil"
  (let ((tokens (tokens hostname :separators (list #\.))))
    (when (= 4 (length tokens))
      (let ((read-tokens (map 'vector #'read-from-string tokens)))
	(when (every #'(lambda (x) (typep x '(mod 256))) read-tokens)
	  read-tokens)))))


(defun lookup-hostname-ip-address (hostname)
  (first (last (tokens (run-external "host" hostname)))))

(defun hostname ()
  (run-external "hostname"))


(defun get-topic-class-name (topic)
  "Given a topic foo with message type, say the string /std_msgs/bar, this returns the symbol named <bar> from the package std_msgs. The topic must be one that has already been advertised or subscribed to."
  (let ((sub (gethash topic *subscriptions*))
	(pub (gethash topic *publications*)))
    (assert (or sub pub) nil "Can't get class name of topic ~a that we neither publish or subscribe" topic)
    (let ((tokens (tokens (if sub (sub-topic-type sub) (pub-topic-type pub)) :separators '(#\/))))
      (assert (= 2 (length tokens)) nil "topic name ~a was not of the form /foo/bar" topic)
      (let ((class-name (concatenate 'string "<" (string-upcase (second tokens)) ">"))
	    (pkg-name (string-upcase (first tokens))))
	(find-symbol class-name pkg-name)))))

(defun ros-message-dir (ros-package-name)
  "Given, e.g., the string std_msgs, will return (in a typical setup), the string /u/your_name/ros/ros/std_msgs/msg/lisp/std_msgs/.  Requires that the PATH (in the environment of the Lisp process) includes rospack."
  (let ((base-dir (run-external "rospack" "find" ros-package-name)))
    (assert base-dir nil "Unable to find ~a." ros-package-name)
    (concatenate 'string base-dir "/msg/lisp/" ros-package-name "/")))

(defun ros-service-dir (ros-package-name)
  "Given, e.g., the string std_srvs, will return (in a typical setup), the string /u/your_name/ros/ros/std_srvs/srv/lisp/std_srvs/.  Requires that the PATH (in the environment of the Lisp process) includes rospack."
  (let ((base-dir (run-external "rospack" "find" ros-package-name)))
    (assert base-dir nil "Unable to find ~a." ros-package-name)
    (concatenate 'string base-dir "/srv/lisp/" ros-package-name "/")))


(defun run-external (cmd &rest args)
  (let ((str (make-string-output-stream)))
    (sb-ext:run-program cmd args :search t :output str)
    (let ((s (get-output-stream-string str)))
      (delete #\Newline s))))
      



(defvar *loaded-files* nil)

(defmacro load-if-necessary (f)
  "load-if-necessary FILENAME.  FILENAME is a string containing a path to a file.  Maintains an internal list (per Lisp session) of which ones have been already loaded, and does not reload a file twice (to avoid annoying warnings)."
  (let ((fname (gensym)))
    `(eval-when (:compile-toplevel :execute :load-toplevel)
       (let ((,fname ,f))
	 (unless (equal (subseq ,fname (- (length ,fname) 5)) ".lisp")
	   (setf ,fname (concatenate 'string ,fname ".lisp")))
	 (unless (member ,fname *loaded-files* :test #'equal)
	   (load ,fname)
	   (push ,fname *loaded-files*))))))

(defmacro ros-load-message-types (&rest message-types)
  "ros-load-message-types &rest MESSAGE-TYPES
Each message type is a string of form package/message.  Loads the corresponding Lisp files.  Makes sure to load things only once.  This means that if the .lisp files are changed during the current Lisp session (which wouldn't happen in the normal scheme of things), you will have to manually reload the file rather than using ros-load-message-types."
  (let ((s (gensym)))
    `(eval-when (:compile-toplevel :execute :load-toplevel)
       (dolist (,s ',message-types)
	 (let ((tokens (tokens ,s :separators (list #\/))))
	   (assert (= 2 (length tokens)) nil "~a was not of the form package/message" ,s)
	   (let ((dir (ros-message-dir (first tokens))))
	     (load-if-necessary (concatenate 'string dir "_package.lisp"))
	     (load-if-necessary (concatenate 'string dir (second tokens)))))))))

      
    
    
(defmacro ros-load-service-types (&rest service-types)
  "ros-load-service-types &rest SERVICE-TYPES
Each service type is a string of form package/service.  Loads the corresponding Lisp files.  Makes sure to load things only once.  This means that if the .lisp files are changed during the current Lisp session (which wouldn't happen in the normal scheme of things), you will have to manually reload the file rather than using ros-load-service-types."
  (let ((s (gensym)))
    `(eval-when (:compile-toplevel :execute :load-toplevel)
       (dolist (,s ',service-types)
	 (let ((tokens (tokens ,s :separators (list #\/))))
	   (assert (= 2 (length tokens)) nil "~a was not of the form package/service" ,s)
	   (let ((dir (ros-service-dir (first tokens))))
	     (load-if-necessary (concatenate 'string dir "_package.lisp"))
	     (load-if-necessary (concatenate 'string dir (second tokens)))))))))    


(defun standalone-exec-debug-hook (a b)
  (declare (ignore b))
  (when (eq *node-status* :running)
    (invoke-restart 'shutdown-ros-node a)
    ))



(defmacro store-message-in (place)
  "Macro store-message-in PLACE

Used if you want a callback function for a topic that just stores the message in some variable.  Expands to definition of a function that sets PLACE to equal its argument."
  (let ((message (gensym)))
    `#'(lambda (,message) (setf ,place ,message))))


