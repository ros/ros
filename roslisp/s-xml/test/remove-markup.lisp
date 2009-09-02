;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: remove-markup.lisp,v 1.1 2004/06/11 11:14:43 scaekenberghe Exp $
;;;;
;;;; Remove markup from an XML document using the SSAX interface
;;;;
;;;; Copyright (C) 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

(defun remove-xml-markup (in)
  (let* ((state (make-instance 'xml-parser-state
                              :text-hook #'(lambda (string seed) (cons string seed))))
         (result (start-parse-xml in state)))
    (apply #'concatenate 'string (nreverse result))))

;;;; eof