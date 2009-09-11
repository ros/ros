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
   (:file "rosout" :depends-on ("msg" "rosutils"))
   (:file "tcpros" :depends-on ("utils" "roslisp" "msg"))
   (:file "sockets" :depends-on ("roslisp" "utils"))
   (:file "slave" :depends-on ("sockets" "utils" "tcpros"))
   (:file "namespace" :depends-on ("roslisp"))
   (:file "client" :depends-on ("sockets" "utils" "namespace" "msg" "rosout"))
   )
	  

  :depends-on (:s-xml :s-xml-rpc :sb-bsd-sockets #+sbcl :sb-bsd-sockets))

;;;; eof
