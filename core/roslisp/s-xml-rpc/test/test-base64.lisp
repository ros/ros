;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: test-base64.lisp,v 1.1.1.1 2004/06/09 09:02:41 scaekenberghe Exp $
;;;;
;;;; Unit and functional tests for base64.lisp
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-base64)

(assert 
 (equal (multiple-value-list (core-encode-base64 0 0 0))
	(list #\A #\A #\A #\A)))

(assert 
 (equal (multiple-value-list (core-encode-base64 255 255 255))
	(list #\/ #\/ #\/ #\/)))

(assert 
 (equal (multiple-value-list (core-encode-base64 1 2 3))
	(list #\A #\Q #\I #\D)))

(assert 
 (equal (multiple-value-list (core-encode-base64 10 20 30))
	(list #\C #\h #\Q #\e)))

(assert
 (equal (multiple-value-list (core-decode-base64 #\A #\A #\A #\A))
	(list 0 0 0)))

(assert
 (equal (multiple-value-list (core-decode-base64 #\/ #\/ #\/ #\/))
	(list 255 255 255)))

(assert
 (equal (multiple-value-list (core-decode-base64 #\A #\Q #\I #\D))
	(list 1 2 3)))

(assert
 (equal (multiple-value-list (core-decode-base64 #\C #\h #\Q #\e))
	(list 10 20 30)))

(assert
 (let* ((string "Hello World!")
	(bytes (map 'vector #'char-code string))
	encoded
	decoded)
   (setf encoded (with-output-to-string (out)
		   (encode-base64-bytes bytes out)))
   (setf decoded (with-input-from-string (in encoded)
		   (decode-base64-bytes in)))
   (equal string
	  (map 'string #'code-char decoded))))

;;; this is more of a functional test

(defun same-character-file (file1 file2)
  (with-open-file (a file1 :direction :input)
    (with-open-file (b file2 :direction :input)
      (loop
       (let ((char-a (read-char a nil nil nil))
	     (char-b (read-char b nil nil nil)))
	 (cond ((not (or (and (null char-a) (null char-b))
			 (and char-a char-b)))
		(return-from same-character-file nil))     
	       ((null char-a)
		(return-from same-character-file t))
	       ((char/= char-a char-b)
		(return-from same-character-file nil))))))))

(defun same-binary-file (file1 file2)
  (with-open-file (a file1 :direction :input :element-type 'unsigned-byte)
    (with-open-file (b file2 :direction :input :element-type 'unsigned-byte)
      (loop
       (let ((byte-a (read-byte a nil nil))
	     (byte-b (read-byte b nil nil)))
	 (cond ((not (or (and (null byte-a) (null byte-b))
			 (and byte-a byte-b)))
		(return-from same-binary-file nil))
	       ((null byte-a)
		(return-from same-binary-file t))
	       ((/= byte-a byte-b)
		(return-from same-binary-file nil))))))))

(let ((original (merge-pathnames "test.b64" *load-pathname*))
      (first-gif (merge-pathnames "test.gif" *load-pathname*))
      (b64 (merge-pathnames "test2.b64" *load-pathname*))
      (second-gif (merge-pathnames "test2.gif" *load-pathname*)))
  (with-open-file (in original
		      :direction :input)
    (with-open-file (out first-gif
			 :direction :output 
			 :element-type 'unsigned-byte
			 :if-does-not-exist :create
			 :if-exists :supersede)
      (decode-base64 in out)))
  (with-open-file (in first-gif
		      :direction :input
		      :element-type 'unsigned-byte)
    (with-open-file (out b64
			 :direction :output
			 :if-does-not-exist :create
			 :if-exists :supersede)
      (encode-base64 in out nil)))
  (assert (same-character-file original b64))
  (with-open-file (in b64
		      :direction :input)
    (with-open-file (out second-gif
			 :direction :output 
			 :element-type 'unsigned-byte
			 :if-does-not-exist :create
			 :if-exists :supersede)
      (decode-base64 in out)))
  (assert (same-binary-file first-gif second-gif))
  (delete-file first-gif)
  (delete-file b64)
  (delete-file second-gif))

;;;; eof