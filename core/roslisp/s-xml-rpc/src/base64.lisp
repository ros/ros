;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: base64.lisp,v 1.1.1.1 2004/06/09 09:02:39 scaekenberghe Exp $
;;;;
;;;; This is a Common Lisp implementation of Base64 encoding and decoding.
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(defpackage s-base64
  (:use common-lisp)
  (:export
   "DECODE-BASE64"
   "ENCODE-BASE64"
   "DECODE-BASE64-BYTES"
   "ENCODE-BASE64-BYTES")
  (:documentation "An implementation of standard Base64 encoding and decoding"))

(in-package :s-base64)

(defparameter +base64-alphabet+
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/")

(defparameter +inverse-base64-alphabet+
  (let ((inverse-base64-alphabet (make-array char-code-limit)))
    (dotimes (i char-code-limit inverse-base64-alphabet)
      (setf (aref inverse-base64-alphabet i)
	    (position (code-char i) +base64-alphabet+)))))
      
(defun core-encode-base64 (byte1 byte2 byte3)
  (values (char +base64-alphabet+ (ash byte1 -2))
	  (char +base64-alphabet+ (logior (ash (logand byte1 #B11) 4)
					   (ash (logand byte2 #B11110000) -4)))
	  (char +base64-alphabet+ (logior (ash (logand byte2 #B00001111) 2)
					   (ash (logand byte3 #B11000000) -6)))
	  (char +base64-alphabet+ (logand byte3 #B111111))))

(defun core-decode-base64 (char1 char2 char3 char4)
  (let ((v1 (aref +inverse-base64-alphabet+ (char-code char1)))
	(v2 (aref +inverse-base64-alphabet+ (char-code char2)))
	(v3 (aref +inverse-base64-alphabet+ (char-code char3)))
	(v4 (aref +inverse-base64-alphabet+ (char-code char4))))
    (values (logior (ash v1 2)
		    (ash v2 -4))
	    (logior (ash (logand v2 #B1111) 4)
		    (ash v3 -2))
	    (logior (ash (logand v3 #B11) 6)
		    v4))))

(defun skip-base64-whitespace (stream)
  (loop
   (let ((char (peek-char nil stream nil nil)))
     (cond ((null char) (return nil))
	   ((null (aref +inverse-base64-alphabet+ (char-code char))) (read-char stream))
	   (t (return char))))))

(defun decode-base64-bytes (stream)
  "Decode a base64 encoded character stream, returns a byte array"
  (let ((out (make-array 256
			 :element-type '(unsigned-byte 8)
			 :adjustable t
			 :fill-pointer 0)))
    (loop
     (skip-base64-whitespace stream)
     (let ((in1 (read-char stream nil nil))
	   (in2 (read-char stream nil nil))
	   (in3 (read-char stream nil nil))
	   (in4 (read-char stream nil nil)))
       (if (null in1) (return))
       (if (or (null in2) (null in3) (null in4)) (error "input not aligned/padded")) 
       (multiple-value-bind (out1 out2 out3)
	   (core-decode-base64 in1
			       in2
			       (if (char= in3 #\=) #\A in3)
			       (if (char= in4 #\=) #\A in4))
	 (vector-push-extend out1 out)
	 (when (char/= in3 #\=)
	   (vector-push-extend out2 out)
	   (when (char/= in4 #\=)
	     (vector-push-extend out3 out))))))
    out))

(defun encode-base64-bytes (array stream &optional (break-lines t))
  "Encode a byte array into a base64b encoded character stream"
  (let ((index 0)
	(counter 0)
	(len (length array)))
    (loop
     (when (>= index len) (return))
     (let ((in1 (aref array index))
	   (in2 (if (< (+ index 1) len) (aref array (+ index 1)) nil))
	   (in3 (if (< (+ index 2) len) (aref array (+ index 2)) nil)))
       (multiple-value-bind (out1 out2 out3 out4)
	   (core-encode-base64 in1
			       (if (null in2) 0 in2)
			       (if (null in3) 0 in3))
	 (write-char out1 stream)
	 (write-char out2 stream)
	 (if (null in2)
	     (progn
	       (write-char #\= stream)
	       (write-char #\= stream))
	   (progn
	     (write-char out3 stream)
	     (if (null in3)
		 (write-char #\= stream)
	       (write-char out4 stream))))
	 (incf index 3)
	 (incf counter 4)
	 (when (and break-lines (= counter 76))
	   (terpri stream)
	   (setf counter 0)))))))

(defun decode-base64 (in out)
  "Decode a base64 encoded character input stream into a binary output stream" 
  (loop
   (skip-base64-whitespace in)
   (let ((in1 (read-char in nil nil))
	 (in2 (read-char in nil nil))
	 (in3 (read-char in nil nil))
	 (in4 (read-char in nil nil)))
     (if (null in1) (return))
     (if (or (null in2) (null in3) (null in4)) (error "input not aligned/padded")) 
     (multiple-value-bind (out1 out2 out3)
	 (core-decode-base64 in1 in2 (if (char= in3 #\=) #\A in3) (if (char= in4 #\=) #\A in4))
       (write-byte out1 out)
       (when (char/= in3 #\=)
	 (write-byte out2 out)
	 (when (char/= in4 #\=)
	   (write-byte out3 out)))))))

(defun encode-base64 (in out &optional (break-lines t))
  "Encode a binary input stream into a base64 encoded character output stream"
  (let ((counter 0))
    (loop
     (let ((in1 (read-byte in nil nil))
	   (in2 (read-byte in nil nil))
	   (in3 (read-byte in nil nil)))
       (if (null in1) (return))
       (multiple-value-bind (out1 out2 out3 out4)
	   (core-encode-base64 in1 (if (null in2) 0 in2) (if (null in3) 0 in3))
	 (write-char out1 out)
	 (write-char out2 out)
	 (if (null in2)
	     (progn
	       (write-char #\= out)
	       (write-char #\= out))
	   (progn
	     (write-char out3 out)
	     (if (null in3)
		 (write-char #\= out)
	       (write-char out4 out))))
	 (incf counter 4)
	 (when (and break-lines (= counter 76))
	   (terpri out)
	   (setf counter 0)))))))

;;;; eof
