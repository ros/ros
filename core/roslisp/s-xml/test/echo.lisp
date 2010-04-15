;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: echo.lisp,v 1.1 2005/08/17 13:44:30 scaekenberghe Exp $
;;;;
;;;; A simple example as well as a useful tool: parse, echo and pretty print XML
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

(defun indent (stream count)
  (loop :repeat (* count 2) :do (write-char #\space stream)))

(defclass echo-xml-seed ()
  ((stream :initarg :stream)
   (level :initarg :level :initform 0)))

#+NIL
(defmethod print-object ((seed echo-xml-seed) stream)
  (with-slots (stream level) seed
    (print-unreadable-object (seed stream :type t)
      (format stream "level=~d" level))))

(defun echo-xml-new-element-hook (name attributes seed)
  (with-slots (stream level) seed
    (indent stream level)
    (format stream "<~a" name)
    (dolist (attribute (reverse attributes)) 
      (format stream " ~a=\'" (car attribute))
      (print-string-xml (cdr attribute) stream)
      (write-char #\' stream))
    (format stream ">~%")
    (incf level)
    seed))

(defun echo-xml-finish-element-hook (name attributes parent-seed seed)
  (declare (ignore attributes parent-seed))
  (with-slots (stream level) seed 
    (decf level)
    (indent stream level)
    (format stream "</~a>~%" name)
    seed))

(defun echo-xml-text-hook (string seed)
  (with-slots (stream level) seed
    (indent stream level)
    (print-string-xml string stream)
    (terpri stream)
    seed))
  
(defun echo-xml (in out)
  "Parse a toplevel XML element from stream in, echoing and pretty printing the result to stream out"
  (start-parse-xml in
		   (make-instance 'xml-parser-state
				  :seed (make-instance 'echo-xml-seed :stream out)
				  :new-element-hook #'echo-xml-new-element-hook
				  :finish-element-hook #'echo-xml-finish-element-hook
				  :text-hook #'echo-xml-text-hook)))

;;;; eof
