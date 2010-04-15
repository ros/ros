;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: validator1-server.lisp,v 1.1 2004/06/14 20:11:55 scaekenberghe Exp $
;;;;
;;;; This is a Common Lisp implementation of the XML-RPC 'validator1'
;;;; server test suite, as live testable from the website
;;;; http://validator.xmlrpc.com and documented on the web page
;;;; http://www.xmlrpc.com/validator1Docs
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml-rpc)

(defun |validator1.echoStructTest| (struct)
  (assert (xml-rpc-struct-p struct))
  struct)

(defun |validator1.easyStructTest| (struct)
  (assert (xml-rpc-struct-p struct))
  (+ (get-xml-rpc-struct-member struct :|moe|)
     (get-xml-rpc-struct-member struct :|larry|)
     (get-xml-rpc-struct-member struct :|curly|)))

(defun |validator1.countTheEntities| (string)
  (assert (stringp string))
  (let ((left-angle-brackets (count #\< string))
	(right-angle-brackets (count #\> string))
	(apostrophes (count #\' string))
	(quotes (count #\" string))
	(ampersands (count #\& string)))
    (xml-rpc-struct :|ctLeftAngleBrackets| left-angle-brackets
		    :|ctRightAngleBrackets| right-angle-brackets
		    :|ctApostrophes| apostrophes
		    :|ctQuotes| quotes
		    :|ctAmpersands| ampersands)))

(defun |validator1.manyTypesTest| (number boolean string double dateTime base64)
  (assert
   (and (integerp number)
	(or (null boolean) (eq boolean t))
	(stringp string)
	(floatp double)
	(xml-rpc-time-p dateTime)
	(and (arrayp base64)
	     (= (array-rank base64) 1)
	     (subtypep (array-element-type base64)
		       '(unsigned-byte 8)))))
  (list number boolean string double dateTime base64))

(defun |validator1.arrayOfStructsTest| (array)
  (assert (listp array))
  (reduce #'+
	  (mapcar #'(lambda (struct)
		      (assert (xml-rpc-struct-p struct))
		      (get-xml-rpc-struct-member struct :|curly|))
		  array)
	  :initial-value 0))

(defun |validator1.simpleStructReturnTest| (number)
  (assert (integerp number))
  (xml-rpc-struct :|times10| (* number 10)
		  :|times100| (* number 100)
		  :|times1000| (* number 1000)))

(defun |validator1.moderateSizeArrayCheck| (array)
  (assert (listp array))
  (concatenate 'string (first array) (first (last array))))

(defun |validator1.nestedStructTest| (struct)
  (assert (xml-rpc-struct-p struct))
  (let* ((year (get-xml-rpc-struct-member struct :\2000))
	 (april (get-xml-rpc-struct-member year :\04))
	 (first (get-xml-rpc-struct-member april :\01)))
    (|validator1.easyStructTest| first)))

(import '(|validator1.echoStructTest| 
          |validator1.easyStructTest| 
          |validator1.countTheEntities| 
          |validator1.manyTypesTest| 
          |validator1.arrayOfStructsTest|
          |validator1.simpleStructReturnTest|
          |validator1.moderateSizeArrayCheck|
          |validator1.nestedStructTest|)
        :s-xml-rpc-exports)

;;;; eof
