;;;; -*- Mode: LISP -*-

;;; We always compile ros-load-manifest into ros-home since it is part
;;; of the roslisp bootstrapping process.

(defclass ros-cl-source-file (cl-source-file) ())

(defun ros-home ()
  (or (sb-ext:posix-getenv "ROS_HOME")
      (merge-pathnames (make-pathname :directory '(:relative ".ros"))
                       (user-homedir-pathname))))

(defmethod output-files ((operation asdf:compile-op) (c ros-cl-source-file))
  (labels ((find-system (c)
             "Returns the system of a component."
             (typecase c
               (asdf:system c)
               (asdf:component (find-system (asdf:component-parent c)))))
           (find-pkg-path (path &optional traversed)
             "Traverses the `path' upwards until it finds a manifest.
              Returns two values, the name of the ros package and the
              relative part of path inside the package. Throws an
              error if it cannot find the manifest."
             (let ((manifest (probe-file (merge-pathnames "manifest.xml" path))))
               (if manifest
                   (values (truename path) traversed)
                   (find-pkg-path (make-pathname :directory (butlast (pathname-directory path)))
                                  (cons (car (last (pathname-directory path)))
                                        traversed)))))
           (system-ros-name (system)
             "Returns the ros package name of a system."
             (multiple-value-bind (package-path rel-path)
                 (find-pkg-path (asdf:component-pathname system))
               (assert (eq (car (pathname-directory package-path)) :absolute))
               (values (car (last (pathname-directory package-path))) rel-path)))
           (pathname-rel-subdir (p1 p2)
             "returns the relative path of `p2' in `p1'."
             (loop with result = (pathname-directory  (truename p2))
                   for d1 in (pathname-directory (truename p1))
                   do (setf result (cdr result))
                   finally (return result))))
    (let ((system (find-system c))
          (component-path (asdf:component-pathname c)))
      (multiple-value-bind (package-name rel-path)
          (system-ros-name system)
        (list
         (asdf::compile-file-pathname
          (merge-pathnames (make-pathname :name (pathname-name component-path)
                                          :type (pathname-type component-path)
                                          :directory `(:relative ,@(pathname-rel-subdir
                                                                    (asdf:component-pathname system)
                                                                    component-path)))
                           (merge-pathnames
                            (make-pathname :directory `(:relative "roslisp" ,package-name ,@rel-path))
                            (ros-home)))))))))

(defsystem :ros-load-manifest
  :name "ros-load-manifest"
  :default-component-class ros-cl-source-file
  :components
  ((:file "load-manifest"))
  :depends-on (:sb-posix))

;;;; eof
