;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: dom.lisp,v 1.2 2005/08/29 15:01:47 scaekenberghe Exp $
;;;;
;;;; This is the generic simple DOM parser and printer interface.
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

;;; top level DOM parser interface

(defgeneric parse-xml-dom (stream output-type)
  (:documentation "Parse a character stream as XML and generate a DOM of output-type"))

(defun parse-xml (stream &key (output-type :lxml))
  "Parse a character stream as XML and generate a DOM of output-type, defaulting to :lxml"
  (parse-xml-dom stream output-type))
  
(defun parse-xml-string (string &key (output-type :lxml))
  "Parse a string as XML and generate a DOM of output-type, defaulting to :lxml"
  (with-input-from-string (stream string)
    (parse-xml-dom stream output-type)))

(defun parse-xml-file (filename &key (output-type :lxml))
  "Parse a character file as XML and generate a DOM of output-type, defaulting to :lxml"
  (with-open-file (in filename :direction :input)
    (parse-xml-dom in output-type)))

;;; top level DOM printer interface

(defgeneric print-xml-dom (dom input-type stream pretty level)
  (:documentation "Generate XML output on a character stream from a DOM of input-type, optionally pretty printing using level"))

(defun print-xml (dom &key (stream t) (pretty nil) (input-type :lxml) (header))
  "Generate XML output on a character stream (t by default) from a DOM of input-type (:lxml by default), optionally pretty printing (off by default), or adding a header (none by default)"
  (when header (format stream header))
  (when pretty (terpri stream))
  (print-xml-dom dom input-type stream pretty 1))

(defun print-xml-string (dom &key (pretty nil) (input-type :lxml))
  "Generate XML output to a string from a DOM of input-type (:lxml by default), optionally pretty printing (off by default)"
  (with-output-to-string (stream)
    (print-xml dom :stream stream :pretty pretty :input-type input-type)))

;;; shared/common support functions

(defun print-spaces (n stream &optional (preceding-newline t))
  (when preceding-newline 
    (terpri stream))
  (loop :repeat n 
        :do (write-char #\Space stream)))

(defun print-solitary-tag (tag stream)
  (write-char #\< stream) 
  (print-identifier tag stream) 
  (write-string "/>" stream))

(defun print-closing-tag (tag stream)
  (write-string "</" stream) 
  (print-identifier tag stream) 
  (write-char #\> stream))
  
(defun print-attribute (name value stream)
  (write-char #\space stream)
  (print-identifier name stream t)
  (write-string "=\"" stream)
  (print-string-xml value stream)
  (write-char #\" stream))

;;;; eof
