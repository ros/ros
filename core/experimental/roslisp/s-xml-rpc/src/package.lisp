;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: package.lisp,v 1.4 2004/06/17 19:43:11 rschlatte Exp $
;;;;
;;;; S-XML-RPC package definition
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser GNU Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(defpackage s-xml-rpc
  (:use 
   common-lisp 
   #+ccl ccl 
   #+lispworks mp
   #+lispworks comm
   s-xml 
   s-base64)
  (:export
   #:xml-rpc-call
   #:encode-xml-rpc-call
   #:call-xml-rpc-server
   #:xml-rpc-condition
   #:xml-rpc-fault #:xml-rpc-fault-code #:xml-rpc-fault-string
   #:xml-rpc-error #:xml-rpc-error-place #:xml-rpc-error-data
   #:start-xml-rpc-server
   #:xml-rpc-time #:xml-rpc-time-p
   #:xml-rpc-time-universal-time
   #:xml-rpc-struct #:xml-rpc-struct-p
   #:xml-rpc-struct-alist #:get-xml-rpc-struct-member #:xml-rpc-struct-equal
   #:*xml-rpc-host* #:*xml-rpc-port* #:*xml-rpc-url* #:*xml-rpc-agent*
   #:*xml-rpc-proxy-host* #:*xml-rpc-proxy-port* #:*xml-rpc-authorization*
   #:*xml-rpc-debug* #:*xml-rpc-debug-stream*
   #:*xml-rpc-package* #:*xml-rpc-call-hook*
   #:execute-xml-rpc-call #:stop-server
   #:|system.listMethods| #:|system.methodSignature| #:|system.methodHelp|
   #:|system.multicall| #:|system.getCapabilities|)
  (:documentation "An implementation of the standard XML-RPC protocol for both client and server"))

(defpackage s-xml-rpc-exports
  (:use)
  (:import-from :s-xml-rpc #:|system.listMethods| #:|system.methodSignature|
                #:|system.methodHelp| #:|system.multicall|
                #:|system.getCapabilities|)
  (:documentation "This package contains the functions callable via xml-rpc."))

;;;; eof
