;;;; -*- Mode: LISP -*-
;;;;
;;;; $Id: s-xml.asd,v 1.2 2005/12/14 21:49:04 scaekenberghe Exp $
;;;;
;;;; The S-XML ASDF system definition
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :asdf)

(defsystem :s-xml
  :name "S-XML"
  :author "Sven Van Caekenberghe <svc@mac.com>"
  :version "3"
  :maintainer "Sven Van Caekenberghe <svc@mac.com>, Brian Mastenbrook <>, Rudi Schlatte <>"
  :licence "Lisp Lesser General Public License (LLGPL)"
  :description "Simple Common Lisp XML Parser"
  :long-description "S-XML is a Common Lisp implementation of a simple XML parser, with a SAX-like and DOM interface"

  :components
  ((:module
    :src
    :components ((:file "package")
                 (:file "xml" :depends-on ("package"))
                 (:file "dom" :depends-on ("package" "xml"))
                 (:file "lxml-dom" :depends-on ("dom"))
                 (:file "sxml-dom" :depends-on ("dom"))
                 (:file "xml-struct-dom" :depends-on ("dom"))))))

;;;; eof
