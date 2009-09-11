;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem :roslisp-test
  :name "roslisp-test"

  :components
  ((:file "talker-test")
   (:file "listener-test")
   (:file "complex-talker-test")
   (:file "complex-listener-test"))

	  
  :depends-on (:roslisp))

;;;; eof
