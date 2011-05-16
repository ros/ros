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


(in-package roslisp-msg-protocol)



(defclass ros-message () ())

(defgeneric serialize (msg str)
  (:documentation "Serialize message object MSG onto stream STR."))

(defgeneric deserialize (msg str)
  (:documentation "Deserialize from stream STR into message object MSG and also returns MSG.  MSG may also be a symbol naming a message type, in which case a new object of that type is created and returned."))
	   

(defgeneric serialization-length (msg)
  (:documentation "Length of this message"))

(defgeneric md5sum (msg-type)
  (:documentation "Return the md5 sum of this message type."))

(defgeneric ros-datatype (msg-type)
  (:documentation "Return the datatype given a message type, service type, service request or response type, or topic name"))

(defgeneric message-definition (msg-type)
  (:documentation "Return the definition of this message type"))


(defgeneric service-request-type (srv))
(defgeneric service-response-type (srv))

(defgeneric symbol-codes (msg-type)
  (:documentation "Return an association list from symbols to numbers (the const declarations in the .msg file)."))


(defgeneric symbol-code (msg-type symbol)
  (:documentation "symbol-code MSG-TYPE SYMBOL.  Gets the value of a message-specific constant declared in a msg file.  The first argument is either a symbol naming the message class, or an instance of the class, and the second argument is the keyword symbol corresponding to the constant name. 

For example, to get the value of the DEBUG constant from Log.msg, use (symbol-code '<Log> :debug)."))

(defgeneric ros-message-to-list (msg)
  (:documentation "Return a structured list representation of the message.  For example, say message type foo has a float field x equal to 42, and a field y which is itself a message of type bar, which has a single field z=24.  This function would then return the structured list '(foo (:x . 42.0) (:y . (bar (:z . 24)))).  

The return value can be passed to list-to-ros-message to retrieve an equivalent message to the original.

As a base case, non-ros messages just return themselves."))


(defgeneric list-to-ros-message (l))

(defvar *print-deprecation-warnings* t)

(defvar *warnings-table* (make-hash-table :test 'eq))

(defmacro msg-deprecation-warning (str &rest args)
  (let ((x (gensym)))
    `(unless (gethash ',x *warnings-table*)
       (when *print-deprecation-warnings* (warn ,str ,@args))
       (setf (gethash ',x *warnings-table*) t)
       )
    ))

;; datatype functions for string arguments

(defun string-to-ros-msgtype-symbol (msg-type)
  "returns the symbol of a know ros messagetype if the string matches the the name case sensitively, else throws error"
  (assert (= 1 (count  #\/ msg-type)) () "Too few or too many slashes in \"~a\"" msg-type)
  (let* ((slashpos (position  #\/ msg-type))
         (type-symbol (find-symbol  (string-upcase (subseq msg-type (1+ slashpos) (length msg-type)))
                                    (string-upcase (concatenate 'string (subseq msg-type 0 slashpos) "-MSG"))))
         (truename (unless (null type-symbol) (ros-datatype type-symbol))))
    (when (null type-symbol) (error "No datatype ~a known" msg-type))
    (unless (string= truename msg-type) (error "Case mismatch for message type ~a, did you mean ~a?" msg-type truename))
    type-symbol))