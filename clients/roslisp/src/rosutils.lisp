;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Software License Agreement (BSD License)
;; 
;; Copyright (c) 2008, Willow Garage, Inc.
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with 
;; or without modification, are permitted provided that the 
;; following conditions are met:
;;
;;  * Redistributions of source code must retain the above 
;;    copyright notice, this list of conditions and the 
;;    following disclaimer.
;;  * Redistributions in binary form must reproduce the 
;;    above copyright notice, this list of conditions and 
;;    the following disclaimer in the documentation and/or 
;;    other materials provided with the distribution.
;;  * Neither the name of Willow Garage, Inc. nor the names 
;;    of its contributors may be used to endorse or promote 
;;    products derived from this software without specific 
;;    prior written permission.
;; 
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
;; CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
;; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
;; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
;; PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
;; COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
;; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
;; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
;; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
;; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
;; OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
;; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
;; DAMAGE.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package roslisp)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ROSLisp-specific utility code
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-condition function-timeout (error) ())

(defun with-function-timeout (expires body-fun)
  "throws function-timeout error when call takes longer than expires arg"
  (flet ((timeout-fun () (error 'function-timeout)))
    (let ((timer (sb-ext:make-timer #'timeout-fun)))
      (sb-ext:schedule-timer timer expires)
      (unwind-protect
           (funcall body-fun)
        (sb-ext:unschedule-timer timer)))))

(defun get-ros-log-location (name)

  ;; Tries various possibilities in order of decreasing priority
  (let ((log-dir (sb-ext:posix-getenv "ROS_LOG_DIR"))
        (ros-home-dir (sb-ext:posix-getenv "ROS_HOME"))
        (home-dir (sb-ext:posix-getenv "HOME")))
    (or *ros-log-location* 
        (merge-pathnames
         (pathname (format nil "~a-~a.log"
                           (if (eql (aref name 0) #\/)
                               (subseq name 1)
                               name)
                           (unix-time)))
         (or (when log-dir (concatenate 'string log-dir "/"))
             (when ros-home-dir (concatenate 'string ros-home-dir "/log/"))
             (when home-dir (concatenate 'string home-dir "/.ros/log/"))
             (error "None of the possibilities for the log directory worked.  Even the HOME environment variable was not set."))))))

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
     (ros-error nil ,str ,@args)
     (error ,str ,@args)))

(defmacro roslisp-warn (str &rest args)
  `(progn
     (ros-warn nil ,str ,@args)
     (warn ,str ,@args)))

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

(defun ip-address-string (address)
  (etypecase address
    (string address)
    (list (format nil "~{~a~^.~}" address))
    (vector (ip-address-string (coerce address 'list)))))

(defun lookup-hostname-ip-address (hostname)
  (host-ent-address (get-host-by-name hostname)))

(defun hostname ()
  (or (sb-ext:posix-getenv "ROS_HOSTNAME")
      (sb-ext:posix-getenv "ROS_IP")
      (run-external "hostname")))

(defun get-topic-class-name (topic)
  "Given a topic foo with message type, say the string /std_msgs/bar, this returns the symbol named bar from the package std_msgs. The topic must be one that has already been advertised or subscribed to."
  (let ((sub (gethash topic *subscriptions*))
        (pub (gethash topic *publications*)))
    (assert (or sub pub) nil "Can't get class name of topic ~a that we neither publish nor subscribe" topic)
    (let* ((type (if sub (sub-topic-type sub) (pub-topic-type pub)))
           (tokens (tokens type :separators '(#\/))))
      (assert (= 2 (length tokens)) nil "topic type ~a of topic ~a was not of the form foo/bar" type topic)
      (let* ((class-name (concatenate 'string (string-upcase (second tokens))))
             (pkg-name (string-upcase (concatenate 'string (first tokens) "-msg")))
             (class-symbol (find-symbol class-name pkg-name)))
        (assert class-symbol nil "Could not find symbol ~a in ~a" class-name pkg-name)
        class-symbol))))

(defvar *message-dir-cache* (make-hash-table :test #'equal))
(defvar *service-dir-cache* (make-hash-table :test #'equal))



(defun run-external (cmd &rest args)
  (let ((str (make-string-output-stream)))
    (let ((proc (sb-ext:run-program cmd args :search t :output str :wait nil)))
      (sb-ext:process-wait proc)
      (if (zerop (sb-ext:process-exit-code proc))
	  (first (last (tokens (get-output-stream-string str) :separators '(#\Newline))))
	  (error "Received exit code ~a when running external process ~a with args ~a" (sb-ext:process-exit-code proc) cmd args)))))
      



(defvar *loaded-files* (make-hash-table :test #'equal))

(defmacro load-if-necessary (f)
  "load-if-necessary FILENAME.  FILENAME is a string containing a path to a file.  Maintains an internal list (per Lisp session) of which ones have been already loaded, and does not reload a file twice (to avoid annoying warnings). DEPRECATED!"
  (declare (ignorable f))
  (error "load-if-necessary deprecated!"))

(defmacro load-message-types (&rest message-types)
  `(progn ,@(mapcar (lambda (msg-type)
                      (let ((msg-system-name (concatenate
                                              'string
                                              (if (symbolp msg-type)
                                                  (string-downcase (symbol-name msg-type))
                                                  msg-type)
                                              "-msg")))
                        `(asdf:operate 'asdf:load-op ,msg-system-name)))
                    message-types)))
    
    
(defmacro load-service-types (&rest service-types)
  "load-service-types &rest SERVICE-TYPES
Each service type is a string of form package/service.  Loads the corresponding Lisp files.  Makes sure to load things only once.  This means that if the .lisp files are changed during the current Lisp session (which wouldn't happen in the normal scheme of things), you will have to manually reload the file rather than using this function."
  `(progn ,@(mapcar (lambda (msg-type)
                      (let ((msg-system-name (concatenate
                                              'string
                                              (if (symbolp msg-type)
                                                  (string-downcase (symbol-name msg-type))
                                                  msg-type)
                                              "-srv")))
                        `(asdf:operate 'asdf:load-op ,msg-system-name)))
                    service-types)))


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


(defmacro load-msg (msg-type)
  "Intended for interactive use.  Pass it a string which is the ros name of the given message, e.g. std_msgs/String.  Loads the corresponding class and calls use-package on that package."
  (let ((pkg-name (first (roslisp-utils:tokens msg-type :separators "/"))))
  `(progn
     (load-message-types ,msg-type)
     (use-package ,(intern (string-upcase (concatenate 'string pkg-name "-msg")) :keyword)))))

(defmacro load-srv (msg-type)
  "Intended for interactive use.  Pass it a string which is the ros name of the given message, e.g. std_msgs/String.  Loads the corresponding class and calls use-package on that package."
  (let ((pkg-name (first (roslisp-utils:tokens msg-type :separators "/"))))
  `(progn
     (load-service-types ,msg-type)
     (use-package ,(intern (string-upcase (concatenate 'string pkg-name "-srv")) :keyword)))))

(defun lookup-topic-type (type)
  "if it's, e.g., the string std_msgs/String just return it, if it's, e.g., 'std_msgs:<String>, return the string std_msgs/String"
  (etypecase type
    (string (ros-datatype (string-to-ros-msgtype-symbol type)))
    (symbol (ros-datatype type))))

(defun make-service-symbol (type-string)
  (let ((pkg (find-package
              (string-upcase (concatenate 'string
                                          (subseq type-string 0 (position #\/ type-string))
                                          "-srv"))))
        (msg-name (string-upcase (subseq type-string (1+ (position #\/ type-string))))))
    (assert pkg () "Service-package not found. Maybe it is not loaded?")
    (nth-value 0 (intern msg-name pkg))))
