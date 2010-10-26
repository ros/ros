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


(defmethod list-to-ros-message ((l null))
  ;; Tricky case: nil should be treated as false (i.e. a primitive boolean) rather than the empty list 
  ;; (since a ros message always has at least one element: the message type)
  nil)

(defmethod list-to-ros-message ((l list))
  (apply #'make-instance (first l) (mapcan #'(lambda (pair) (list (car pair) (list-to-ros-message (cdr pair)))) (rest l))))

;; Either a primitive type or vector or already a ros message (should do a bit more type checking)
(defmethod list-to-ros-message (msg)
  msg)

(defun convert-to-keyword (s)
  (declare (symbol s))
  (let ((name (string-upcase (symbol-name s))))
    (when (> (length name) 4)
      (let ((pos (- (length name) 4)))
	(when (search "-VAL" name :start2 pos)
	  (let ((new-name (subseq name 0 pos)))
	    (signal 'compile-warning
		    :msg (format nil "I'm assuming you're using ~a to refer to ~a (the old form), in a call to with-fields or def-service-callback.  For now, converting automatically.  This usage is deprecated though; switch to just using ~a (cf roslisp_examples/add-two-ints-server.lisp)." name new-name new-name))
	    (setq s (intern new-name 'keyword)))))))
  (if (keywordp s)
      s
      (intern (symbol-name s) 'keyword)))

(defun extract-nested-field (m f)
  "extract a named field from a message.  F can also be a list.  E.g, if F is '(:bar :foo) that means extract field foo of field bar of the message.  Calls list-to-ros-message before returning."
  (let ((l (ros-message-to-list m)))
    (list-to-ros-message
     (cond 
       ((symbolp f) (get-field l f))
       ((null (rest f)) (get-field l (first f)))
       (t (extract-nested-field (get-field l (first f)) (rest f)))))))

(defun get-field (l f)
  (declare (list l) (symbol f))
  (let ((pair (assoc f (rest l))))
    (unless pair
      (error "Could not find field ~a in ~a" f l))
    (cdr pair)))

(defun set-field (l f v)
  (declare (list l) (symbol f))
  (let ((pair (assoc f (rest l))))
    (unless pair
      (error "Could not find field ~a in ~a" f l))
    (setf (cdr pair) v)))

(defmacro with-fields (bindings m &body body)
  "with-fields BINDINGS MSG &rest BODY

A macro for convenient access to message fields.

BINDINGS is an unevaluated list of bindings.  Each binding is like a let binding (FOO BAR), where FOO is a symbol naming a variable that will be bound to the field value.  BAR describes the field.  In the simplest case it's just a symbol naming the field.  It can also be a list, e.g. (QUX GAR).  This means the field QUX of the field GAR of the message.  Finally, the entire binding can be a symbol FOO, which is a shorthand for (FOO FOO).  
MSG evaluates to a message.
BODY is the body, surrounded by an implicit progn.

As an example, instead of (let ((foo (pkg:foo-val (pkg:bar-val m)))
      (baz (pkg:baz-val m)))
  (stuff)) 

you can use (with-fields ((foo (foo bar)) baz)
		(stuff))"

  (let ((msg-list (gensym)))
    ;; Once message-to-list is cached, no need to call message-to-list here (just in extract-nested-field)
    `(let ((,msg-list (ros-message-to-list ,m)))
       (declare (ignorable ,msg-list))
       (let 
	   ,(mapcar #'(lambda (binding)
			(when (symbolp binding) (setq binding (list binding binding)))
			(symbol-macrolet ((field (second binding)))
			  (setf field (mapcar #'convert-to-keyword (if (symbolp field) (list field) field))))
			`(,(first binding) (extract-nested-field ,msg-list ',(reverse (second binding)))))
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



(defun listify-message (m nested-field)
  (if nested-field
      (dbind (f . r) nested-field
	(let ((m2 (ros-message-to-list m)))
	  (set-field m2 f (listify-message (get-field m2 f) r))
	  m2))
      m))


(defun ros-message-to-list-nested (m fields)
  "Return a copy of M which is sufficiently listified that all the specified fields can be accessed through lists"
  (dolist (f fields m)
    (setq m (listify-message m f))))
    

;; Basic helper function that takes in a message and returns a new message with some fields updated (see below)
(defun set-fields-fn (m &rest args)
  (let (fields vals)
    (while args
      (push (reverse (designated-list (pop args))) fields)
      (push (pop args) vals))
    (let ((l (ros-message-to-list-nested m fields)))
      (loop
	for field in fields
	for val in vals
	do (setf (cdr (field-pair field l)) val))
      (list-to-ros-message l))))


(defun make-message-fn (msg-type &rest args)
  "Creates a message of ros type MSG-TYPE (a string PKG/MSG), where the odd ARGS are lists of keywords that designated a nested field and the even arguments are the values.  E.g., where an odd argument '(:foo :bar) means the foo field of the bar field of the corresponding even argument."
  (destructuring-bind (pkg-name type) (tokens (string-upcase msg-type) :separators '(#\/))
    (let ((pkg (find-package (intern (concatenate 'string pkg-name "-MSG") 'keyword))))
      (assert pkg nil "Can't find package ~a-MSG" pkg-name)
      (let ((class-name (find-symbol (concatenate 'string "<" type ">") pkg)))
	(assert class-name nil "Can't find class for ~a" msg-type)
	(apply #'set-fields-fn (make-instance class-name) args)))))

(defun make-service-request-fn (srv-type &rest args)
  (destructuring-bind (pkg type) (tokens (string-upcase srv-type) :separators '(#\/))
    (let ((pkg (find-package (intern (concatenate 'string pkg "-SRV") 'keyword))))
      (assert pkg nil "Can't find package ~a" pkg)
      (let ((class-name (find-symbol (concatenate 'string "<" type "-REQUEST>") pkg)))
	(assert class-name nil "Can't find class ~a in package ~a" class-name pkg)
	(apply #'set-fields-fn (make-instance class-name) args)))))

(defmacro make-request (srv-type &rest args)
  "make-request SRV-TYPE &rest ARGS

Like make-message, but creates a service request object.  SRV-TYPE can be either a string of the form package_name/message_name, or a symbol naming the service (the name is the base name of the .srv file).  ARGS are as in make-message."
  (etypecase srv-type
    (string
       `(make-service-request-fn ,srv-type
				 ,@(loop
				     for i from 0
				     for arg in args
				     collect (if (evenp i) `',arg arg))))
    (symbol `(make-service-request ',srv-type ,@args))
    (cons (assert (eql (car srv-type) 'quote)) `(make-service-request ,srv-type ,@args))))

(defun make-service-request (service-type &rest args)
  (apply #'make-instance (service-request-type service-type) args))

(defmacro modify-message-copy (m &rest args)
  "modify-message-copy MSG &rest ARGS

Return a new message that is a copy of MSG with some fields modified.  ARGS is a list of the form FIELD-SPEC1 VAL1 ... FIELD-SPEC_k VAL_k as in make-message."
  `(set-fields-fn ,m ,@(loop for i from 0 for arg in args collect (if (evenp i) `',(mapcar #'convert-to-keyword (designated-list arg)) arg))))

(defmacro setf-msg (place &rest args)
  "Sets PLACE to be the result of calling modify-message-copy on PLACE and ARGS"
  (let ((m (gensym)))
    `(let ((,m ,place))
       (setf ,place (modify-message-copy ,m ,@args)))))

(defmacro make-message (msg-type &rest args)
  "make-message MSG-TYPE &rest ARGS

Convenience macro for creating messages easily.

MSG-TYPE is a string naming a message ros datatype, i.e.., of form package_name/message_name

ARGS is a list of form FIELD-SPEC1 VAL1 ... FIELD-SPECk VALk
Each FIELD-SPEC (unevaluated) is a list (or a symbol, which designates a list of one element) that refers to a possibly nested field.
VAL is the corresponding value.

For example, if MSG-TYPE is the string robot_msgs/Pose, and ARGS are (x position) 42 (w orientation) 1
this will create a Pose with the x field of position equal to 42 and the w field of orientation equal to 1 (other fields equal their default values)."

  `(make-message-fn ,msg-type
		    ,@(loop
			 for i from 0
			 for arg in args
			 collect (if (evenp i) `',(mapcar #'convert-to-keyword (designated-list arg)) arg))))

			   

(defmacro make-msg (&rest args)
  "Alias for make-message"
  `(make-message ,@args))


