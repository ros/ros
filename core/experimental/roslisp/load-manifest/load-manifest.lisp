(defpackage :ros-load-manifest
  (:nicknames :ros-load)
  (:export :load-manifest :asdf-paths-to-add)
  (:use :cl :s-xml))

(in-package :ros-load-manifest)

(defun depended-packages (package-root)
  "Look in this directory for a manifest.xml.  If it doesn't exist, signal an error.  Else, look in the file for tags that look like <depend package=foo> (where foo is a string with double quotes), and return a list of such foo."
  (let ((tree (s-xml:parse-xml-file (merge-pathnames "manifest.xml" package-root))))
    (depended-packages-helper tree)))

(defun depended-packages-helper (tree)
  (when (listp tree)
    (if (eq :|depend| (first tree))
	(list (third tree))
	(mapcan #'depended-packages-helper tree))))

(defun asdf-paths-to-add (package)
  "Given a package name, looks in the manifest and follows dependencies.  Stops when it reaches a leaf or a package that contains no asdf/ directory.  Adds all the /asdf directories that it finds to a list and return it."
  (let* ((path (ros-package-path package))
	 (asdf-dir (get-asdf-directory path)))
    (when asdf-dir
      (cons asdf-dir (mapcan #'asdf-paths-to-add (depended-packages path))))))

(defun ros-package-path (p)
  (let ((str (make-string-output-stream)))
    (sb-ext:run-program "rospack" (list "find" p) :search t :output str)
    (let ((result (get-output-stream-string str)))
      (loop
	 (if (eql (aref result (1- (length result))) #\Newline)
	     (setq result (subseq result 0 (1- (length result))))
	     (return)))
      (let ((last-newline-pos (position #\Newline result :from-end t)))
	(when last-newline-pos
	  (setq result (subseq result (1+ last-newline-pos)))))
      (unless (eq #\/ (char result (1- (length result))))
	(setq result (concatenate 'string result "/")))
      (if (search "[rospack] couldn't find" result)
	  (error "Rospack error: ~a" result)
	  (pathname result)))))

(defun get-asdf-directory (path)
  (let ((asdf-path (merge-pathnames "asdf/" path)))
    (when (probe-file asdf-path) asdf-path)))
      


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; top level
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  
(defun load-manifest (package)
  "Walks down the tree of dependencies of this ros package.  Backtracks when it reaches a leaf or a package with no asdf/ subdirectory.  Adds all the asdf directories it finds to the asdf:*central-registry*."
  (dolist (p (asdf-paths-to-add package))
    (pushnew p asdf:*central-registry* :test #'equal)))

