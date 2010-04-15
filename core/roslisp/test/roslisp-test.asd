;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem "roslisp-test"
  :depends-on ("std_msgs-msg" "roslisp" "roslisp-msg" "roslisp-srv")

  :components
  ((:file "package")
   (:file "talker-test" :depends-on ("package"))
   (:file "listener-test" :depends-on ("package"))
   (:file "complex-talker-test" :depends-on ("package"))
   (:file "complex-listener-test" :depends-on ("package"))
   (:file "service-server-test" :depends-on ("package"))
   (:file "service-client-test" :depends-on ("package")))

	  
  :depends-on (:roslisp))

;;;; eof
