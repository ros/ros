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
  (cerror "continue" "Called asdf-paths-to-add deprecated")
  (let* ((path (ros-package-path package))
	 (asdf-dir (get-asdf-directory path)))
    (append
     (when asdf-dir (list asdf-dir))
     (mapcan #'asdf-paths-to-add (depended-packages path)))))


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
	  (progn (warn "Rospack error: ~a" result) nil)
	  (pathname result)))))


(defun get-asdf-directory (path)
  (let ((asdf-path (merge-pathnames "asdf/" path)))
    (when (probe-file asdf-path) asdf-path)))
      
(defun asdf-ros-msg-srv-search (definition)
  (let ((package-name (subseq definition 0 (- (length definition) 4)))
        (package-suffix (subseq definition (- (length definition) 4))))
    (when (member package-suffix '("-msg" "-srv") :test #'equal)
      (let ((filename (merge-pathnames (make-pathname
                                        :directory `(:relative ,(subseq package-suffix 1)
                                                               "lisp" ,package-name)
                                        :name definition
                                        :type "asd")
                                       (parse-namestring (ros-package-path package-name)))))
        (when (probe-file filename)
          filename)))))

(defun asdf-ros-pkg-search (definition)
    (let ((pos (position #\/ definition :from-end t)))
      (when pos
	(let* ((pkg (subseq definition 0 pos))
	       (suffix (subseq definition (1+ pos)))
	       (pkg-path (parse-namestring (ros-package-path pkg)))
	       (filename
		(merge-pathnames
		 (make-pathname :directory '(:relative "asdf") :name suffix :type "asd")
		 pkg-path)))
	  (when (probe-file filename)
	    filename)))))


(setq asdf:*system-definition-search-functions* 
      (nconc asdf:*system-definition-search-functions*
	     '(asdf-ros-msg-srv-search asdf-ros-pkg-search)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; top level
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  
(defun load-manifest (package)
  "Walks down the tree of dependencies of this ros package.  Backtracks when it reaches a leaf or a package with no asdf/ subdirectory.  Adds all the asdf directories it finds to the asdf:*central-registry*."
  (cerror "continue" "Load manifest deprecated!")
  (dolist (p (asdf-paths-to-add package))
    (pushnew p asdf:*central-registry* :test #'equal)))

