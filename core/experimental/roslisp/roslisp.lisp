(defpackage :roslisp
  (:use 
   :cl
   :sb-bsd-sockets
   :sb-sys
   :sb-thread
   :s-xml-rpc
   :extended-reals
   :roslisp-queue
   :roslisp-utils
   )
  (:export
   :ros-message
   :serialize
   :deserialize
   :md5sum
   :serialization-length
   :symbol-codes
   :symbol-code
   :ros-message-to-list
   :service-request-type
   :service-response-type
   :list-to-ros-message
   :node-status

   :load-if-necessary

   :start-ros-node
   :shutdown-ros-node
   :with-ros-node
   :print-status
   :make-response
   :defservice
   :advertise
   :subscribe
   :register-service
   :call-service
   :publish-on-topic
   :loop-at-most-every

   :ros-load-message-types
   :ros-load-service-types

   :get-param
   :set-param
   :has-param
   :delete-param

   :pprint-ros-message
   :read-ros-message

   :ros-debug
   :ros-warn
   :ros-info
   :ros-error
   :ros-fatal
   
   :fully-qualified-name
   :make-uri

   :standalone-exec-debug-hook
   :*running-from-command-line*))
   

(in-package :roslisp)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ROS Node state
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *ros-node-name* nil "String holding node name")
(defvar *node-status* :shutdown)
(defvar *master-uri* nil "URI of ROS master")
(defvar *xml-server* nil "String holding name of XML-RPC server (needed by s-xml-rpc")
(defvar *xml-rpc-caller-api* nil "Holds the caller-api argument to XML RPC calls to the master.")
(defvar *publication-server* nil "Passive socket that topic-subscribers will connect to")
(defvar *publication-server-address-string* nil "As in *publication-server-address*, but a string")
(defvar *publication-server-port* nil "Port of publication server")
(defvar *publications* nil "Hashtable from topic name to list of subscriber-connections")
(defvar *subscriptions* nil "Hashtable from topic name to object of type subscription")
(defvar *services* nil "Hashtable from service name to corresponding socket")
(defvar *ros-lock* (make-mutex :name "API-wide lock for all operations that affect/are affected by state of node"))
(defvar *debug-stream-lock* (make-mutex :name "API-wide lock for the debugging output stream."))
(defvar *running-from-command-line* nil "True iff running ROS node script from command line (noninteractively)")
(defvar *broken-socket-streams* nil "Used by TCPROS to keep track of sockets that have died and shouldn't be written to any more.")
(defvar *services-to-start* nil "Holds pending services that need to be started by the event loop")
(defvar *next-service-port* 9001 "Holds the next port at which a service server will try to listen")
(defvar *namespace* nil "The name of the node's parent namespace")
(defvar *remapped-names* nil "Hash from strings to strings containing names that have been remapped on the command line")
(defvar *debug-stream* t "Stream to which to print debug messages.  Defaults to t (stdout).")
(defvar *break-on-socket-errors* nil "If true, then any error on a socket will cause a (continuable) break")
(defvar *debug-level* 2 "Controls the behavior of ros-debug and others.  The default value of 2 means print info and above.  1 would be everything.  4 would be warnings and above, etc.")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Type defs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO is this all that goes in a URI - Should the protocol name be included?
;: Note other code assumes URI's can be tested for equality using #'equalp
(defstruct (uri (:constructor create-uri)) address port)

(defun make-uri (address port)
  (create-uri :address address :port port))

(defstruct (subscription (:conc-name nil))
  sub-topic-type
  buffer 
  topic-thread
  (callbacks nil)
  publisher-connections)

(defstruct (publisher-connection (:conc-name nil))
  publisher-socket
  publisher-stream
  uri)

(defstruct (publication (:conc-name nil))
  pub-topic-type
  subscriber-connections)

(defstruct (subscriber-connection (:conc-name nil))
  subscriber-socket
  subscriber-stream)

(defstruct (service (:conc-name nil))
  service-socket
  name
  service-api)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun print-status ()
  (format t "~&Node ~a~2I~&status: ~a~&Master URI: ~a~&Publications ~/roslisp-utils:pprint-hash/~&Subscriptions ~/roslisp-utils:pprint-hash/"
	  *ros-node-name* *node-status* *master-uri* *publications* *subscriptions*))



