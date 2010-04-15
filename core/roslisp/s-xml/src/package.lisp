;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: package.lisp,v 1.6 2005/11/20 14:24:34 scaekenberghe Exp $
;;;;
;;;; This is a Common Lisp implementation of a very basic XML parser.
;;;; The parser is non-validating.
;;;; The API into the parser is pure functional parser hook model that comes from SSAX,
;;;; see also http://pobox.com/~oleg/ftp/Scheme/xml.html or http://ssax.sourceforge.net
;;;; Different DOM models are provided, an XSML, an LXML and a xml-element struct based one.
;;;;
;;;; Copyright (C) 2002, 2003, 2004, 2005 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(defpackage s-xml
  (:use common-lisp)
  (:export
   ;; main parser interface
   #:start-parse-xml
   #:print-string-xml
   #:xml-parser-error #:xml-parser-error-message #:xml-parser-error-args #:xml-parser-error-stream
   #:xml-parser-state #:get-entities #:get-seed
   #:get-new-element-hook #:get-finish-element-hook #:get-text-hook
   ;; dom parser and printer
   #:parse-xml-dom #:parse-xml #:parse-xml-string #:parse-xml-file
   #:print-xml-dom #:print-xml #:print-xml-string
   ;; xml-element structure
   #:make-xml-element #:xml-element-children #:xml-element-name 
   #:xml-element-attribute #:xml-element-attributes
   #:xml-element-p #:new-xml-element #:first-xml-element-child
   ;; namespaces
   #:*ignore-namespaces* #:*local-namespace* #:*namespaces*
   #:*require-existing-symbols* #:*auto-export-symbols* #:*auto-create-namespace-packages*
   #:find-namespace #:register-namespace #:get-prefix #:get-uri #:get-package
   #:resolve-identifier #:extend-namespaces #:print-identifier #:split-identifier)
  (:documentation 
   "A simple XML parser with an efficient, purely functional, event-based interface as well as a DOM interface"))

;;;; eof
