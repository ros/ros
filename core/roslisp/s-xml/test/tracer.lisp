;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: tracer.lisp,v 1.2 2004/06/11 11:14:43 scaekenberghe Exp $
;;;;
;;;; A simple SSAX tracer example that can be used to understand how the hooks are called
;;;;
;;;; Copyright (C) 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

(defun trace-xml-log (level msg &rest args)
  (indent *standard-output* level)
  (apply #'format *standard-output* msg args)
  (terpri *standard-output*))

(defun trace-xml-new-element-hook (name attributes seed)
  (let ((new-seed (cons (1+ (car seed)) (1+ (cdr seed)))))
    (trace-xml-log (car seed) 
                   "(new-element :name ~s :attributes ~:[()~;~:*~s~] :seed ~s) => ~s" 
                   name attributes seed new-seed)
    new-seed))

(defun trace-xml-finish-element-hook (name attributes parent-seed seed)
  (let ((new-seed (cons (1- (car seed)) (1+ (cdr seed)))))
    (trace-xml-log (car parent-seed)
                   "(finish-element :name ~s :attributes ~:[()~;~:*~s~] :parent-seed ~s :seed ~s) => ~s" 
                   name attributes parent-seed seed new-seed)
    new-seed))

(defun trace-xml-text-hook (string seed)
  (let ((new-seed (cons (car seed) (1+ (cdr seed)))))
    (trace-xml-log (car seed) 
                   "(text :string ~s :seed ~s) => ~s" 
                   string seed new-seed)
    new-seed))

(defun trace-xml (in)
  "Parse and trace a toplevel XML element from stream in"
  (start-parse-xml in
		   (make-instance 'xml-parser-state
				  :seed (cons 0 0) 
                                  ;; seed car is xml element nesting level
                                  ;; seed cdr is ever increasing from element to element
				  :new-element-hook #'trace-xml-new-element-hook
                                  :finish-element-hook #'trace-xml-finish-element-hook
				  :text-hook #'trace-xml-text-hook)))

(defun trace-xml-file (pathname)
  "Parse and trace XMl from the file at pathname"
  (with-open-file (in pathname)
    (trace-xml in)))

;;;; eof
