;;;; -*- Mode: LISP -*-
;;;;
;;;; $Id: s-xml-rpc.asd,v 1.2 2004/06/17 19:43:11 rschlatte Exp $
;;;;
;;;; The S-XML-RPC ASDF system definition
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :asdf)

(defsystem :s-xml-rpc
  :name "S-XML-RPC"
  :author "Sven Van Caekenberghe <svc@mac.com>"
  :version "7"
  :maintainer "Sven Van Caekenberghe <svc@mac.com>, Brian Mastenbrook <>, Rudi Schlatte <>"
  :licence "Lesser Lisp General Public License (LLGPL)"
  :description "Common Lisp XML-RPC Package"
  :long-description "s-xml-rpc is a Common Lisp implementation of the XML-RPC procotol for both client and server"

  :components
  ((:module
    :src 
    :components ((:file "base64")
                 (:file "package" :depends-on ("base64"))
                 (:file "sysdeps" :depends-on ("package"))
                 (:file "xml-rpc" :depends-on ("package" "sysdeps" "base64"))
                 (:file "extensions" :depends-on ("package" "xml-rpc")))))
  :depends-on (:s-xml #+sbcl :sb-bsd-sockets))

;;;; eof
