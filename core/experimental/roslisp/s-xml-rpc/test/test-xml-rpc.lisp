;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: test-xml-rpc.lisp,v 1.1.1.1 2004/06/09 09:02:41 scaekenberghe Exp $
;;;;
;;;; Unit and functional tests for xml-rpc.lisp
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml-rpc)

(assert
 (let ((now (get-universal-time)))
   (equal (iso8601->universal-time (universal-time->iso8601 now))
	  now)))

(assert
 (equal (with-input-from-string (in (encode-xml-rpc-call "add" 1 2))
	  (decode-xml-rpc in))
	'("add" 1 2)))

(assert
 (equal (with-input-from-string (in (encode-xml-rpc-result '(1 2)))
	  (car (decode-xml-rpc in)))
	'(1 2)))

(let ((condition (with-input-from-string (in (encode-xml-rpc-fault "Fatal Error" 100))
		   (decode-xml-rpc in))))
  (assert (typep condition 'xml-rpc-fault))
  (assert (equal (xml-rpc-fault-string condition) "Fatal Error"))
  (assert (equal (xml-rpc-fault-code condition) 100)))

(assert
 (xml-rpc-time-p (xml-rpc-call (encode-xml-rpc-call "currentTime.getCurrentTime")
			       :host "time.xmlrpc.com")))

(assert
 (equal (xml-rpc-call (encode-xml-rpc-call "examples.getStateName" 41)
		      :host "betty.userland.com")
	"South Dakota"))

(assert
 (equal (call-xml-rpc-server '(:host "betty.userland.com") "examples.getStateName" 41)
	"South Dakota"))

(assert
 (let ((server-process-name (start-xml-rpc-server :port 8080)))
   (sleep 1) ; give the server some time to come up ;-)
   (unwind-protect
       (equal (xml-rpc-call (encode-xml-rpc-call "XML-RPC-IMPLEMENTATION-VERSION") :port 8080)
	      (xml-rpc-implementation-version))
     (stop-server server-process-name))))

(assert
    (let* ((struct-in (xml-rpc-struct :foo 100 :bar ""))
           (xml (with-output-to-string (out)
                  (encode-xml-rpc-value struct-in out)))
           (struct-out (with-input-from-string (in xml)
                         (decode-xml-rpc in))))
      (xml-rpc-struct-equal struct-in struct-out)))
                              
;;;; eof