;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem :roslisp
  :name "roslisp"

  :components
  ((:file "roslisp")
   (:file "rosutils" :depends-on ("roslisp"))
   (:file "msg" :depends-on ("roslisp" "rosutils"))
   (:file "msg-header" :depends-on ("msg" "rosout"))
   (:file "rosout-codegen" :depends-on ("msg" "rosutils"))
   (:file "rosout" :depends-on ("rosout-codegen"))
   (:file "tcpros" :depends-on ("roslisp" "msg" "rosout"))
   (:file "sockets" :depends-on ("roslisp" "rosout"))
   (:file "slave" :depends-on ("sockets" "tcpros" "rosout"))
   (:file "namespace" :depends-on ("roslisp" "rosout"))
   (:file "client" :depends-on ("sockets" "namespace" "msg" "rosout"))
   (:file "pprint" :depends-on ("rosout" "msg"))
   )
	  

  :depends-on (:s-xml :s-xml-rpc :sb-bsd-sockets
               :roslib-msg :roslisp-msg-protocol
               :roslisp-utils))

;;;; eof
