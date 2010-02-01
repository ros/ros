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

(defun get-param (key &optional (default nil default-supplied))
  "get-param KEY &optional DEFAULT.  

KEY is a string naming a ros parameter.

Looks up parameter on parameter server.  If not found, use default if provided, and error otherwise."
  (declare (string key))
  (with-fully-qualified-name key
    (if (has-param key)
	(protected-call-to-master ("getParam" key) c
	    (roslisp-error "Could not contact master when getting param ~a: ~a" key c))
	(if default-supplied
	    default
	    (roslisp-error "Param ~a does not exist, and no default supplied" key)))))

(defun set-param (key val)
  "set-param KEY VAL

KEY is a string naming a ros parameter.
VAL is a string or integer.

Post: parameter key set to value on the parameter server."
  (declare (string key))
  (with-fully-qualified-name key
    (protected-call-to-master ("setParam" key val) c
      (roslisp-error "Could not contact master at ~a when setting param ~a" *master-uri* key))))

(defun has-param (key)
  "KEY is a string naming a ros parameter

Return true iff this parameter exists on the server."
  (declare (string key))
  (with-fully-qualified-name key
    (protected-call-to-master ("hasParam" key) c
      (roslisp-error "Could not contact master at ~a for call to hasParam ~a: ~a" *master-uri* key c))))

(defun delete-param (key)
  "KEY is a string naming a ros parameter
Remove this key from parameter server."
  (declare (string key))
  (with-fully-qualified-name key
    (protected-call-to-master ("deleteParam" key) c
      (roslisp-error "Could not contact master at ~a when deleting param ~a: ~a" *master-uri* key c))))


(defun get-param-names ()
  "Return list of params on server"
  (protected-call-to-master ("getParamNames") c
    (roslisp-error "Could not contact master at ~a when getting param list: ~a" *master-uri* c)))

(defun list-params (&optional namespace)
  "NAMESPACE is either a list of symbols (e.g. '(foo bar) refers to /foo/bar) or a string that is treated as a namespace name (possibly relative to the current one).  Returns all parameters in that namespace."
  (declare (type (or string list) namespace))
  (setq namespace
	(if (listp namespace)
	    (format nil "~{/~a~}/" (mapcar #'(lambda (n) (string-downcase (symbol-name n))) namespace))
	    (let ((fqn (fully-qualified-name namespace)))
	      (if (eql #\/ (char fqn (1- (length fqn))))
		  fqn
		  (concatenate 'string fqn "/")))))
  

  (filter #'(lambda (name) (search namespace name :end2 (min (length namespace) (length name)))) (get-param-names)))