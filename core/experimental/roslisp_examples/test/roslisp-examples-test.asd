;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem :roslisp-examples-test
  :name "roslisp-examples-test"

  :components
  ((:file "talker-test")
   (:file "listener-test"))
	  
  :depends-on (:roslisp))

;;;; eof
