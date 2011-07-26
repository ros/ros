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

(in-package :roslisp)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Code for talking to the master over xml rpc
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-condition ros-rpc-error ()
  ;; TODO
  ((code :initarg :code :reader code)
   (message :initarg :message :reader message)
   (vals :initarg :vals :reader vals)
   (call :initarg :call :reader call)
   (uri :initarg :uri :reader rpc-uri))
  (:report (lambda (c str) 
	     (format str "ros-rpc call ~a to ~a failed with code ~a, message ~a, values ~a"
		     (call c) (rpc-uri c) (code c) (message c) (vals c)))))

(defun lookup-service (name)
  (ros-rpc-call *master-uri* "lookupService" name))

(defun ros-rpc-call (uri name &rest args)
  "ros-rpc-call XML-RPC-SERVER-URI CALL-NAME &rest CALL-ARGS.  Preprends the ros node name to the arg list and does the call.  Throws a continuable error if a code <= 0 is returned.  Otherwise, return the values.  Requires that uri is not null (though node need not be running)."
  (mvbind (address port) (parse-uri uri)
  
    (dbind (code msg vals)
	(xml-rpc-call 
	 (apply #'encode-xml-rpc-call name 
		(or *ros-node-name* "uninitialized-roslisp-node")
		args) 
	 :host address :port port)
      (when (<= code 0) (cerror "Ignore and continue" 'ros-rpc-error
				:call (cons name args) :uri uri :code code :message msg :vals vals))
      vals)))

(defmacro protected-call-to-master ((&rest args) &optional c &body cleanup-forms)
  "Wraps an xml-rpc call to the master.  Calls cleanup forms if xml-rpc connection fails.  Requires that the node is running, or that the *master-uri* is set."
  (setf c (or c (gensym)))
  `(handler-case (let ((args (list ,@args)))
                   (assert *master-uri* nil "Master uri not set")
                   (ros-debug (roslisp master) "Calling master with arguments `~a'" args)
                   (apply #'ros-rpc-call *master-uri* args))
     (sb-bsd-sockets:connection-refused-error (,c)
       (declare (ignorable ,c))
       ,@cleanup-forms)))





