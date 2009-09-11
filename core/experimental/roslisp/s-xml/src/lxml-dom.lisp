;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: lxml-dom.lisp,v 1.6 2005/11/20 14:34:15 scaekenberghe Exp $
;;;;
;;;; LXML implementation of the generic DOM parser and printer.
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

;;; the lxml hooks to generate lxml

(defun lxml-new-element-hook (name attributes seed)
  (declare (ignore name attributes seed))
  '())

(defun lxml-finish-element-hook (name attributes parent-seed seed)
  (let ((xml-element
	 (cond ((and (null seed) (null attributes))
		name)
	       (attributes
		`((,name ,@(let (list)
			     (dolist (attribute attributes list)
			       (push (cdr attribute) list)
			       (push (car attribute) list))))
		  ,@(nreverse seed)))
	       (t
		`(,name ,@(nreverse seed))))))
    (cons xml-element parent-seed)))

(defun lxml-text-hook (string seed)
  (cons string seed))

;;; standard DOM interfaces

(defmethod parse-xml-dom (stream (output-type (eql :lxml)))
  (car (start-parse-xml stream
			(make-instance 'xml-parser-state
				       :new-element-hook #'lxml-new-element-hook
				       :finish-element-hook #'lxml-finish-element-hook
				       :text-hook #'lxml-text-hook))))

(defun plist->alist (plist)
  (when plist 
    (cons (cons (first plist) (second plist))
          (plist->alist (rest (rest plist))))))

(defmethod print-xml-dom (dom (input-type (eql :lxml)) stream pretty level)
  (declare (special *namespaces*))
  (cond ((symbolp dom) (print-solitary-tag dom stream))
	((stringp dom) (print-string-xml dom stream))
	((consp dom)
	 (let (tag attributes)
	   (cond ((symbolp (first dom)) (setf tag (first dom)))
		 ((consp (first dom)) (setf tag (first (first dom)) 
                                            attributes (plist->alist (rest (first dom)))))
		 (t (error "Input not recognized as LXML ~s" dom)))
           (let ((*namespaces* (extend-namespaces attributes *namespaces*)))
             (write-char #\< stream) 
             (print-identifier tag stream)
             (loop :for (name . value) :in attributes 
                   :do (print-attribute name value stream))
             (if (rest dom)
                 (let ((children (rest dom)))
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
	(t (error "Input not recognized as LXML ~s" dom))))
  
;;;; eof