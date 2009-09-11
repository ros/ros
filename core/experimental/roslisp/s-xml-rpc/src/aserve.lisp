;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: aserve.lisp,v 1.1.1.1 2004/06/09 09:02:39 scaekenberghe Exp $
;;;;
;;;; This file implements XML-RPC client and server networking based
;;;; on (Portable) AllegroServe (see http://opensource.franz.com/aserve/
;;;; or http://sourceforge.net/projects/portableaserve/), which you have
;;;; to install first.
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser GNU Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(defpackage xml-rpc-aserve
  (:use common-lisp net.aserve.client net.aserve xml-rpc)
  (:export
   "XML-RPC-CALL"
   "START-XML-RPC-ASERVE"
   "PUBLISH-ASERVE-XML-RPC-HANDLER"))

(in-package :xml-rpc-aserve)

(defun xml-rpc-call-aserve (encoded &key
				    (url *xml-rpc-url*)
				    (agent *xml-rpc-agent*)
				    (host *xml-rpc-host*)
				    (port *xml-rpc-port*)
                                    (basic-autorization *xml-rpc-authorization*)
				    (proxy))
  (let ((xml (print-xml-string encoded)))
    (multiple-value-bind (response response-code headers uri)
	(do-http-request
	 (format nil "http://~a:~d~a" host port url)
	 :method :post
	 :protocol :http/1.0
	 :user-agent agent
	 :content-type "text/xml"
         :basic-authorization basic-autorization
	 :content xml
	 :proxy proxy)
      (declare (ignore headers uri))
      (if (= response-code 200)
          (let ((result (decode-xml-rpc (make-string-input-stream response))))
            (if (typep result 'xml-rpc-fault)
                (error result)
              (car result)))
	(error "http-error:~d" response-code)))))

(defun start-xml-rpc-aserve (&key (port *xml-rpc-port*))
  (process-run-function "aserve-xml-rpc" 
			#'(lambda ()
			    (start :port port 
				   :listeners 4 
				   :chunking nil 
				   :keep-alive nil))))

(defun publish-aserve-xml-rpc-handler (&key (url *xml-rpc-url*) (agent *xml-rpc-agent*))
  (declare (ignore agent))
  (publish :path url
	   :content-type "text/xml"
	   :function #'aserve-xml-rpc-handler))

(defun aserve-xml-rpc-handler (request entity)
  (with-http-response (request
		       entity
		       :response (if (eq :post (request-method request))
				     *response-ok*
				   *response-bad-request*))
    (with-http-body (request entity)
      (let ((body (get-request-body request))
	    (id (process-name *current-process*)))
	(with-input-from-string (in body)
	  (let ((xml (handle-xml-rpc-call in id)))
	    (format-debug t "~d sending ~a~%" id xml)
	    (princ xml *html-stream*)))))))

;;;; eof
