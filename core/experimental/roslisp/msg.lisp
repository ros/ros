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


(in-package roslisp)



(defmethod deserialize ((msg symbol) str)
  (let ((m (make-instance msg)))
    (deserialize m str)
    m))

(defmethod serialization-length ((msg symbol))
  (warn "Hmm... unexpectedly asked for serialization length of ~a.  Likely an error - please inform roslisp developers." msg)
  42)

(defmethod md5sum ((msg-type array))
  (if (stringp msg-type)
      (md5sum (get-topic-class-name msg-type))
      (progn
        (warn "Hmmm... unexpected topic type specifier ~a in md5sum.  Passing it on anyway..." msg-type)
        (call-next-method))))

(defmethod ros-datatype ((msg-type array))
  (if (stringp msg-type)
      (ros-datatype (get-topic-class-name msg-type))
      (progn
        (warn "Hmm... unexpected topic type specifier ~a in ros-datatype.  Passing it on anyway..." msg-type)
        (call-next-method))))

(defmethod message-definition ((msg-type array))
  (if (stringp msg-type)
      (message-definition (get-topic-class-name msg-type))
      (progn
        (warn "Hmm... unexpected topic type specifier ~a in message-definition.  Passing it on anyway..." msg-type)
        (call-next-method))))

(defun make-response (service-type &rest args)
  (apply #'make-instance (service-response-type service-type) args))

(defmethod symbol-codes ((msg-type symbol))
  nil)

(defmethod symbol-code ((m ros-message) s)
  (symbol-code (class-of m) s))

(defmethod symbol-code ((m symbol) s)
  (let ((pair (assoc s (symbol-codes m))))
    (unless pair
      (error "Could not get symbol code for ~a for ROS message type ~a" s m))
    (cdr pair)))

(defmethod ros-message-to-list (msg)
  (check-type msg (not ros-message) "something that is not a ros-message")
  msg)


(defmethod list-to-ros-message ((l list))
  (apply #'make-instance (first l) (mapcan #'(lambda (pair) (list (car pair) (list-to-ros-message (cdr pair)))) (rest l))))
(defmethod list-to-ros-message (msg)
  msg)

(defun convert-to-keyword (s)
  (declare (symbol s))
  (if (keywordp s)
      s
      (intern (symbol-name s) 'keyword)))

(defun extract-nested-field (l f)
  "extract a field from a message that has been converted into a list.  F can also be a list.  E.g, if F is '(:foo :bar) that means extract field foo of field bar of the message."
  (cond 
    ((symbolp f) (get-field l f))
    ((null (rest f)) (get-field l (first f)))
    (t (get-field (extract-nested-field l (rest f)) (first f)))))

(defun get-field (l f)
  (let ((pair (assoc f (rest l))))
    (unless pair
      (error "Could not find field ~a in ~a" f l))
    (cdr pair)))

(defmacro with-fields (bindings m &body body)
  "with-fields BINDINGS MSG &rest BODY

A macro for convenient access to message fields.

BINDINGS is an unevaluated list of bindings.  Each binding is like a let binding (FOO BAR), where FOO is a symbol naming a variable that will be bound to the field value.  BAR describes the field.  In the simplest case it's just a symbol naming the field.  It can also be a list, e.g. (QUX GAR).  This means the field QUX of the field GAR of the message.  Finally, the entire binding can be a symbol FOO, which is a shorthand for (FOO FOO).  
MSG evaluates to a message.
BODY is the body, surrounded by an implicit progn.

As an example, instead of 
(let ((foo (pkg:foo-val (pkg:bar-val m)))
      (baz (pkg:baz-val m)))
  (stuff))

you can use
(with-fields ((foo (foo bar))
	      baz)
    (stuff))"

  (let ((msg-list (gensym)))
    `(let ((,msg-list (ros-message-to-list ,m)))
       (let 
	   ,(mapcar #'(lambda (binding)
			(when (symbolp binding) (setq binding (list binding binding)))
			(symbol-macrolet ((field (second binding)))
			  (setf field (mapcar #'convert-to-keyword (if (symbolp field) (list field) field))))
			`(,(first binding) (extract-nested-field ,msg-list ',(second binding))))
		    bindings)
	 ,@body))))


(defun read-ros-message (stream)
  (list-to-ros-message (read stream)))


(defun field-pair (f l)
  (let ((p (assoc (intern (symbol-name (car f)) :keyword) (cdr l))))
    (assert p nil "Couldn't find field ~a in ~a (overall field spec was ~a)" (car f) (cdr l) f)
    (if (cdr f)
	(field-pair (cdr f) (cdr p))
	p)))



(defun make-message-fn (msg-type &rest args)
  (destructuring-bind (pkg type) (tokens (string-upcase msg-type) :separators '(#\/))
    (let ((pkg (find-package (intern (concatenate 'string pkg "-MSG") 'keyword))))
      (assert pkg nil "Can't find package ~a" pkg)
      (let ((class-name (find-symbol (concatenate 'string "<" type ">") pkg)))
	(assert class-name nil "Can't find class for ~a" msg-type)
	(let ((l (ros-message-to-list (make-instance class-name))))
	  (while args
	    (let* ((field (pop args))
		   (val (pop args)))
	      (setf (cdr (field-pair (reverse (designated-list field)) l)) val)))
	  (list-to-ros-message l))))))

(defmacro make-message (msg-type &rest args)
  "make-message MSG-TYPE &rest ARGS

Convenience macro for creating messages easily.

MSG-TYPE is a string naming a message ros datatype.

ARGS is a list of form FIELD-SPEC1 VAL1 ... FIELD-SPECk VALk
Each FIELD-SPEC (unevaluated) is a list (or a symbol, which designates a list of one element) that refers to a possibly nested field.
VAL is the corresponding value.

For example, if MSG-TYPE is the string robot_msgs/Pose, and ARGS are (x position) 42 (w orientation) 1
this will create a Pose with the x field of position equal to 42 and the w field of orientation equal to 1 (other fields equal their default values)."

  `(make-message-fn ,msg-type
		    ,@(loop
			 for i from 0
			 for arg in args
			 collect (if (evenp i) `',arg arg))))
			   



