;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: sxml-dom.lisp,v 1.5 2005/11/20 14:34:15 scaekenberghe Exp $
;;;;
;;;; LXML implementation of the generic DOM parser and printer.
;;;;
;;;; Copyright (C) 2003, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

;;; the sxml hooks to generate sxml

(defun sxml-new-element-hook (name attributes seed)
  (declare (ignore name attributes seed))
  '())

(defun sxml-finish-element-hook (name attributes parent-seed seed)
  (let ((xml-element (append (list name)
			     (when attributes
			       (list (let (list)
				       (dolist (attribute attributes (cons :@ list))
					 (push (list (car attribute) (cdr attribute)) list)))))
			     (nreverse seed))))
    (cons xml-element parent-seed)))

(defun sxml-text-hook (string seed)
  (cons string seed))

;;; the standard DOM interfaces

(defmethod parse-xml-dom (stream (output-type (eql :sxml)))
  (car (start-parse-xml stream
			(make-instance 'xml-parser-state
				       :new-element-hook #'sxml-new-element-hook
				       :finish-element-hook #'sxml-finish-element-hook
				       :text-hook #'sxml-text-hook))))

(defmethod print-xml-dom (dom (input-type (eql :sxml)) stream pretty level)
  (declare (special *namespaces*))
  (cond ((stringp dom) (print-string-xml dom stream))
	((consp dom)
	 (let ((tag (first dom))
	       attributes
	       children)
	   (if (and (consp (second dom)) (eq (first (second dom)) :@))
	       (setf attributes (rest (second dom))
		     children (rest (rest dom)))
	     (setf children (rest dom)))
           (let ((*namespaces* (extend-namespaces (loop :for (name value) :in attributes 
                                                        :collect (cons name value))
                                                  *namespaces*)))
             (write-char #\< stream)
             (print-identifier tag stream)
             (loop :for (name value) :in attributes
                   :do (print-attribute name value stream))
             (if children
                 (progn
                   (write-char #\> stream)
                   (if (and (= (length children) 1) (stringp (first children)))
                       (print-string-xml (first children) stream)
                     (progn
                       (dolist (child children)
                         (when pretty (print-spaces (* 2 level) stream))
                         (if (stringp child)
                             (print-string-xml child stream)
                           (print-xml-dom child input-type stream pretty (1+ level))))
                       (when pretty (print-spaces (* 2 (1- level)) stream))))
                   (print-closing-tag tag stream))
               (write-string "/>" stream)))))
	(t (error "Input not recognized as SXML ~s" dom))))

;;;; eof
