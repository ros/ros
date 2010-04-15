;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: counter.lisp,v 1.2 2004/06/11 11:14:43 scaekenberghe Exp $
;;;;
;;;; A simple SSAX counter example that can be used as a performance test
;;;;
;;;; Copyright (C) 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

(defclass count-xml-seed ()
  ((elements :initform 0)
   (attributes :initform 0)
   (characters :initform 0)))

(defun count-xml-new-element-hook (name attributes seed)
  (declare (ignore name))
  (incf (slot-value seed 'elements))
  (incf (slot-value seed 'attributes) (length attributes))
  seed)

(defun count-xml-text-hook (string seed)
  (incf (slot-value seed 'characters) (length string))
  seed)
  
(defun count-xml (in)
  "Parse a toplevel XML element from stream in, counting elements, attributes and characters"
  (start-parse-xml in
		   (make-instance 'xml-parser-state
				  :seed (make-instance 'count-xml-seed)
				  :new-element-hook #'count-xml-new-element-hook
				  :text-hook #'count-xml-text-hook)))

(defun count-xml-file (pathname)
  "Parse XMl from the file at pathname, counting elements, attributes and characters"
  (with-open-file (in pathname)
    (let ((result (count-xml in)))
      (with-slots (elements attributes characters) result
        (format t 
                "~a contains ~d XML elements, ~d attributes and ~d characters.~%" 
                pathname elements attributes characters)))))

;;;; eof
