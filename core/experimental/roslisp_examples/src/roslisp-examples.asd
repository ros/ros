;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem :roslisp-examples
  :name "roslisp-examples"

  :components
  ((:file "talker")
   (:file "listener")
   (:file "add-two-ints-client")
   (:file "add-two-ints-server")
   (:file "array-talker")
   (:file "array-listener")
   (:file "params")
   (:file "rosout-example"))
	  
  :depends-on (:roslisp))

;;;; eof
