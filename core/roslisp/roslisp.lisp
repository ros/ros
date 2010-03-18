;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Software License Agreement (BSD License)
;; 
;; Copyright (c) 2008, Willow Garage, Inc.
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with 
;; or without modification, are permitted provided that the 
;; following conditions are met:
;;
;;  * Redistributions of source code must retain the above 
;;    copyright notice, this list of conditions and the 
;;    following disclaimer.
;;  * Redistributions in binary form must reproduce the 
;;    above copyright notice, this list of conditions and 
;;    the following disclaimer in the documentation and/or 
;;    other materials provided with the distribution.
;;  * Neither the name of Willow Garage, Inc. nor the names 
;;    of its contributors may be used to endorse or promote 
;;    products derived from this software without specific 
;;    prior written permission.
;; 
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
;; CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
;; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
;; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
;; PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
;; COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
;; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
;; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
;; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
;; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
;; OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
;; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
;; DAMAGE.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defpackage :roslisp
  (:use 
   :cl
   :sb-bsd-sockets
   :sb-sys
   :sb-thread
   :s-xml-rpc
   :roslisp-extended-reals
   :roslisp-queue
   :roslisp-utils
   :roslisp-msg-protocol
   :ros-load-manifest
   :std_srvs-srv
   )
  (:export
   :with-fields
   :make-message
   :make-msg
   :modify-message-copy
   :setf-msg
   :ros-message

   :node-status
   :make-response
   :make-request
   :symbol-code
   :symbol-codes

   :load-if-necessary
   
   :*current-ros-package*

   :start-ros-node
   :shutdown-ros-node
   :with-ros-node
   :def-ros-node 

   :print-status
   :make-response
   :defservice
   :advertise
   :subscribe
   :register-service
   :register-service-fn

   :def-service-callback
   :call-service
   :wait-for-service
   :def-service-call

   :publish-on-topic
   :publish
   :publish-msg

   :loop-at-most-every
   :spin-until 
   :wait-duration
   :with-parallel-thread
   :store-message-in

   :load-message-types
   :load-service-types

   :get-param
   :set-param
   :has-param
   :delete-param
   :list-params

   :ros-time
   :ros-time-not-yet-received
   :spin-until-ros-time-valid

   :pprint-ros-message
   :read-ros-message

   :set-debug-level
   :set-debug-levels
   :print-debug-levels
   :debug-level
   :ros-debug
   :ros-warn
   :ros-info
   :ros-error
   :ros-fatal

   
   ;; debug topics
   :roslisp
   :top
   :tcp

   :load-msg ;; todo remove?
   :load-srv ;; todo remove?

   :*ros-node-name*
   :num-subscribers
   
   :fully-qualified-name
   :make-uri

   :*default-master-uri*
   :*master-uri*

   :standalone-exec-debug-hook
   :*running-from-command-line*))
   

(in-package :roslisp)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ROS Node state
;; Stored in special variables since node is a singleton
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *ros-node-name* nil "String holding node name")
(defvar *node-status* :shutdown)
(defvar *master-uri* nil "URI of ROS master")
(defvar *default-master-uri* nil "Default master URI.  Is nil (intended for convenience during interactive use).")
(defvar *xml-server* nil "String holding name of XML-RPC server (needed by s-xml-rpc")
(defvar *xml-rpc-caller-api* nil "Holds the caller-api argument to XML RPC calls to the master.")
(defvar *tcp-server* nil "Passive socket that topic-subscribers will connect to")
(defvar *tcp-server-hostname* nil "Address of tcp server")
(defvar *tcp-server-port* nil "Port of tcp server")
(defvar *service-uri* nil "uri for service calls to this node")
(defvar *publications* nil "Hashtable from topic name to list of subscriber-connections")
(defvar *subscriptions* nil "Hashtable from topic name to object of type subscription")
(defvar *services* nil "Hashtable from service name to object of type service")
(defvar *ros-lock* (make-mutex :name "API-wide lock for all operations that affect/are affected by state of node"))
(defvar *debug-stream-lock* (make-mutex :name "API-wide lock for the debugging output stream."))
(defvar *running-from-command-line* nil "True iff running ROS node script from command line (noninteractively)")
(defvar *broken-socket-streams* nil "Used by TCPROS to keep track of sockets that have died and shouldn't be written to any more.")
(defvar *namespace* nil "The name of the node's parent namespace")
(defvar *ros-log-location* nil "Name of file to which ros lisp debugging info is written")
(defvar *ros-log-stream* nil "Output stream bound to log file during node execution")
(defvar *remapped-names* nil "Hash from strings to strings containing names that have been remapped on the command line")
(defvar *debug-stream* t "Stream to which to print debug messages.  Defaults to standard out.")
(defvar *break-on-socket-errors* nil "If true, then any error on a socket will cause a (continuable) break")
(defvar *debug-level* 2 "Controls the behavior of ros-debug and others.  The default value of 2 means print info and above.  1 would be everything.  4 would be warnings and above, etc.")
(defvar *last-clock* nil)
(defvar *use-sim-time* nil)



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
  subscriber-connections
  is-latching
  last-message)

(defstruct (subscriber-connection (:conc-name nil))
  subscriber-socket
  subscriber-stream)

(defstruct service
  md5
  name
  ros-type
  request-ros-type
  response-ros-type
  request-class
  callback)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Querying the node
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun node-status ()
  *node-status*)

(defun num-subscribers (pub)
  (length (subscriber-connections pub)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun print-status ()
  (format t "~&Node ~a~2I~&status: ~a~&Master URI: ~a~&Publications ~/roslisp-utils:pprint-hash/~&Subscriptions ~/roslisp-utils:pprint-hash/"
	  *ros-node-name* *node-status* *master-uri* *publications* *subscriptions*))


(define-condition compile-warning (condition)
  ((msg :initarg :msg)))
  
  
