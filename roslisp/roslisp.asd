;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem :roslisp
  :name "roslisp"

  :components
  ((:file "roslisp")
   (:file "rosutils" :depends-on ("roslisp"))
   (:file "master" :depends-on ("rosutils"))
   (:file "namespace" :depends-on ("roslisp"))
   (:file "msg" :depends-on ("roslisp" "rosutils"))
   (:file "rosout-codegen" :depends-on ("msg" "rosutils"))
   (:file "rosout" :depends-on ("rosout-codegen"))
   (:file "msg-header" :depends-on ("msg" "rosout"))
   (:file "params" :depends-on ("namespace" "rosutils" "roslisp" "rosout" "master"))
   (:file "tcpros" :depends-on ("roslisp" "msg" "rosout"))
   (:file "sockets" :depends-on ("roslisp" "rosout"))
   (:file "slave" :depends-on ("sockets" "tcpros" "rosout"))
   (:file "command-line-args" :depends-on ("roslisp" "rosout"))
   (:file "client" :depends-on ("sockets" "command-line-args" "msg" "rosout"))
   (:file "debug-levels" :depends-on ("params" "client" "rosout"))
   (:file "node" :depends-on ("client"))
   (:file "pprint" :depends-on ("rosout" "msg"))
   )
	  

  :depends-on (:s-xml :s-xml-rpc :sb-bsd-sockets
               :roslib-msg :roslisp-msg-protocol
               :roslisp-utils :std_srvs-srv))

;;;; eof
