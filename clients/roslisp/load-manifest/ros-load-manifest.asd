;;;; -*- Mode: LISP -*-

(defsystem :ros-load-manifest
  :name "ros-load-manifest"
  :components ((:file "load-manifest"))
  :depends-on (:sb-posix))
