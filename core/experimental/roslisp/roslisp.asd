;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem :roslisp
  :name "roslisp"

  :components
  ((:file "roslisp" :depends-on ("utils"))
   (:module "utils"
	    :components
	    ((:file "utils")
	     (:file "float-bytes")
	     (:file "extended-reals")
	     (:file "queue" :depends-on ("utils" "extended-reals"))
	     (:file "hash-utils" :depends-on ("utils"))))
   (:file "rosutils" :depends-on ("utils" "roslisp"))
   (:file "msg" :depends-on ("roslisp" "rosutils"))
   (:file "msg-header" :depends-on ("msg" "rosout"))
   (:file "rosout-codegen" :depends-on ("msg" "rosutils"))
   (:file "rosout" :depends-on ("rosout-codegen"))
   (:file "tcpros" :depends-on ("utils" "roslisp" "msg" "rosout"))
   (:file "sockets" :depends-on ("roslisp" "utils" "rosout"))
   (:file "slave" :depends-on ("sockets" "utils" "tcpros" "rosout"))
   (:file "namespace" :depends-on ("roslisp" "rosout"))
   (:file "client" :depends-on ("sockets" "utils" "namespace" "msg" "rosout"))
   )
	  

  :depends-on (:s-xml :s-xml-rpc :sb-bsd-sockets))

;;;; eof
