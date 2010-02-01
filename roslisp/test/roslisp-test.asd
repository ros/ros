;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem "roslisp-test"
  :depends-on ("std_msgs-msg" "roslisp" "roslisp-msg")

  :components
  ((:file "talker-test")
   (:file "listener-test")
   (:file "complex-talker-test")
   (:file "complex-listener-test"))

	  
  :depends-on (:roslisp))

;;;; eof
