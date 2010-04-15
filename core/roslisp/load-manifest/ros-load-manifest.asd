;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem :ros-load-manifest
  :name "ros-load-manifest"

  :components
  ((:file "load-manifest"))
	  
  :depends-on (:s-xml))

;;;; eof
