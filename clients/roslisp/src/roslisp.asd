;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem :roslisp
  :name "roslisp"

  :components
  ((:file "roslisp")
   (:file "rosutils" :depends-on ("roslisp"))
   (:file "master" :depends-on ("rosutils"))
   (:file "rosout-codegen" :depends-on ("msg" "rosutils"))
   (:file "rosout" :depends-on ("rosout-codegen"))
   (:file "time" :depends-on ("rosutils" "rosout"))
   (:file "namespace" :depends-on ("roslisp"))
   (:file "msg" :depends-on ("roslisp" "rosutils"))
   (:file "msg-header" :depends-on ("msg" "rosout" "time"))
   (:file "params" :depends-on ("namespace" "rosutils" "roslisp" "rosout" "master"))
   (:file "tcpros" :depends-on ("roslisp" "msg" "rosout" "msg-serialization-stream"))
   (:file "sockets" :depends-on ("roslisp" "rosout"))
   (:file "slave" :depends-on ("sockets" "tcpros" "rosout"))
   (:file "command-line-args" :depends-on ("roslisp" "rosout" "namespace"))
   (:file "client" :depends-on ("sockets" "namespace" "command-line-args" "msg" "rosout" "master"))
   (:file "persistent-service" :depends-on ("sockets" "namespace" "roslisp" "tcpros"))
   (:file "debug-levels" :depends-on ("params" "client" "rosout"))
   (:file "node" :depends-on ("client"))
   (:file "msg-serialization-stream" :depends-on ("roslisp"))
   (:file "pprint" :depends-on ("rosout" "msg"))
   )
	  

  :depends-on (:s-xml :s-xml-rpc :sb-bsd-sockets
               :rosgraph_msgs-msg :roslisp-msg-protocol
               :ros-load-manifest
               :roslisp-utils :std_srvs-srv))

;;;; eof
