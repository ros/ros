;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: validator1-client.lisp,v 1.1 2004/06/14 20:11:55 scaekenberghe Exp $
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

(defun random-string (&optional (length 8))
  (with-output-to-string (stream)
    (dotimes (i (random length))
      (write-char (code-char (+ 32 (random 95)))
		  stream))))

(defun echo-struct-test ()
  (let* ((struct (xml-rpc-struct :|foo| (random 1000000)
				 :|bar| (random-string)
				 :|fooBar| (list (random 100) (random 100))))
	 (result (xml-rpc-call (encode-xml-rpc-call :|validator1.echoStructTest|
						    struct))))
    (format t "validator1.echoStructTest(~s)=~s~%" struct result)
    (assert (xml-rpc-struct-equal struct result))))
	     
(defun easy-struct-test ()
  (let* ((moe (random 1000))
	 (larry (random 1000))
	 (curry (random 1000))
	 (struct (xml-rpc-struct :|moe| moe
				 :|larry| larry
				 :|curly| curry))
	 (result (xml-rpc-call (encode-xml-rpc-call :|validator1.easyStructTest|
						    struct))))
    (format t "validator1.easyStructTest(~s)=~s~%" struct result) 
    (assert (= (+ moe larry curry) result))))

(defun count-the-entities ()
  (let* ((string (random-string 512))
	 (left-angle-brackets (count #\< string))
	 (right-angle-brackets (count #\> string))
	 (apostrophes (count #\' string))
	 (quotes (count #\" string))
	 (ampersands (count #\& string))
	 (result (xml-rpc-call (encode-xml-rpc-call :|validator1.countTheEntities|
						    string))))
    (format t "validator1.countTheEntitities(~s)=~s~%" string result)
    (assert
     (and (xml-rpc-struct-p result)
	  (= left-angle-brackets
	     (get-xml-rpc-struct-member result :|ctLeftAngleBrackets|))
	  (= right-angle-brackets
	     (get-xml-rpc-struct-member result :|ctRightAngleBrackets|))
	  (= apostrophes
	     (get-xml-rpc-struct-member result :|ctApostrophes|))
	  (= quotes
	     (get-xml-rpc-struct-member result :|ctQuotes|))
	  (= ampersands
	     (get-xml-rpc-struct-member result :|ctAmpersands|))))))

(defun array-of-structs-test ()
  (let ((array (make-array (random 32)))
	(sum 0))
    (dotimes (i (length array))
      (setf (aref array i)
	    (xml-rpc-struct :|moe| (random 1000)
			    :|larry| (random 1000)
			    :|curly| (random 1000)))
      (incf sum (get-xml-rpc-struct-member (aref array i)
					   :|curly|)))
    (let ((result (xml-rpc-call (encode-xml-rpc-call :|validator1.arrayOfStructsTest|
						     array))))
      (format t "validator1.arrayOfStructsTest(~s)=~s~%" array result)
      (assert (= result sum)))))

(defun random-bytes (&optional (length 16))
  (let ((bytes (make-array (random length) :element-type '(unsigned-byte 8))))
    (dotimes (i (length bytes) bytes)
      (setf (aref bytes i) (random 256)))))

(defun many-types-test ()
  (let* ((integer (random 10000))
	 (boolean (if (zerop (random 2)) t nil))
	 (string (random-string))
	 (double (random 10000.0))
	 (dateTime (xml-rpc-time))
	 (base64 (random-bytes))
	 (result (xml-rpc-call (encode-xml-rpc-call :|validator1.manyTypesTest|
						    integer
						    boolean
						    string
						    double
						    dateTime
						    base64))))
    (format t
	    "validator1.manyTypesTest(~s,~s,~s,~s,~s,~s)=~s~%"
	    integer
	    boolean
	    string
	    double
	    dateTime
	    base64
	    result)
    (assert (equal integer (elt result 0)))
    (assert (equal boolean (elt result 1)))
    (assert (equal string (elt result 2)))
    (assert (equal double (elt result 3)))
    (assert (equal (xml-rpc-time-universal-time dateTime)
		   (xml-rpc-time-universal-time (elt result 4))))
    (assert (reduce #'(lambda (x y) (and x y))
		    (map 'list #'= base64 (elt result 5))
		    :initial-value t))))

(defun simple-struct-return-test ()
  (let* ((number (random 1000))
	 (result (xml-rpc-call (encode-xml-rpc-call :|validator1.simpleStructReturnTest| number))))
    (format t "validator1.simpleStructReturnTest(~s)=~s~%" number result)
    (assert
     (and (= (* number 10)
	     (get-xml-rpc-struct-member result :|times10|))
	  (= (* number 100)
	     (get-xml-rpc-struct-member result :|times100|))
	  (= (* number 1000)
	     (get-xml-rpc-struct-member result :|times1000|))))))

(defun moderate-size-array-check ()
  (let ((array (make-array (+ 100 (random 100))
			   :element-type 'string)))
    (dotimes (i (length array))
      (setf (aref array i) (random-string)))
    (let ((result (xml-rpc-call (encode-xml-rpc-call :|validator1.moderateSizeArrayCheck|
						     array))))
      (format t "validator1.moderateSizeArrayCheck(~s)=~s~%" array result)
      (assert
       (equal (concatenate 'string (elt array 0) (elt array (1- (length array))))
	      result)))))

(defun nested-struct-test ()
  (let* ((moe (random 1000))
	 (larry (random 1000))
	 (curry (random 1000))
	 (struct (xml-rpc-struct :|moe| moe
				 :|larry| larry
				 :|curly| curry))
	 (first (xml-rpc-struct :\01 struct))
	 (april (xml-rpc-struct :\04 first))
	 (year (xml-rpc-struct :\2000 april))
	 (result (xml-rpc-call (encode-xml-rpc-call :|validator1.nestedStructTest|
						    year))))
    (format t "validator1.nestedStructTest(~s)=~s~%" year result) 
    (assert (= (+ moe larry curry) result))))

(defun test-run (&optional (runs 1))
  (dotimes (i runs t)
    (echo-struct-test)
    (easy-struct-test)
    (count-the-entities)
    (array-of-structs-test)
    (many-types-test)
    (simple-struct-return-test)
    (moderate-size-array-check)
    (nested-struct-test)))

(defun timed-test-run (&optional (runs 1))
  (dotimes (i runs t)
    (time (echo-struct-test))
    (time (easy-struct-test))
    (time (count-the-entities))
    (time (array-of-structs-test))
    (time (many-types-test))
    (time (simple-struct-return-test))
    (time (moderate-size-array-check))
    (time (nested-struct-test))))

;;;; eof
