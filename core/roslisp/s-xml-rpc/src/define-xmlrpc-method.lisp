;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: define-xmlrpc-method.lisp,v 1.1 2004/07/08 19:45:25 scaekenberghe Exp $
;;;;
;;;; The code in this file adds a very handly define-xmlrpc-method macro.
;;;;
;;;; (define-xmlrpc-method get-state-name (state)
;;;;   :url #u"http://betty.userland.com/RPC2"
;;;;   :method "examples.getStateName")
;;;;
;;;; (define-xmlrpc-method get-time ()
;;;;   :url #u"http://time.xmlrpc.com/RPC2"
;;;;   :method "currentTime.getCurrentTime")
;;;;
;;;; It require the PURI package.
;;;;
;;;; Copyright (C) 2004 Frederic Brunel.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser GNU Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(defmacro define-xmlrpc-method (name args &key url method)
  `(defun ,name ,args
     (xml-rpc-call (encode-xml-rpc-call ,method ,@args)
       :url ,(puri:uri-path url)
       :host ,(puri:uri-host url)
       :port ,(cond ((puri:uri-port url)) (t 80)))))

;;;; eof
