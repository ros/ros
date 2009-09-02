;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem "roslisp_examples/roslisp_examples"
  :components
  ((:file "talker")
   (:file "listener")
   (:file "add-two-ints-client")
   (:file "add-two-ints-server")
   (:file "array-talker")
   (:file "array-listener")
   (:file "params")
   (:file "rosout-example"))
	  
  :depends-on ("roslisp" "roslisp_examples-msg" "roslisp_examples-srv" "std_msgs-msg"))

;;;; eof
