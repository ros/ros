;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Software License Agreement (BSD License)
;; 
;; Copyright (c) 2008, Willow Garage, Inc.
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with 
;; or without modification, are permitted provided that the 
;; following conditions are met:
;;
;;  * Redistributions of source code must retain the above 
;;    copyright notice, this list of conditions and the 
;;    following disclaimer.
;;  * Redistributions in binary form must reproduce the 
;;    above copyright notice, this list of conditions and 
;;    the following disclaimer in the documentation and/or 
;;    other materials provided with the distribution.
;;  * Neither the name of Willow Garage, Inc. nor the names 
;;    of its contributors may be used to endorse or promote 
;;    products derived from this software without specific 
;;    prior written permission.
;; 
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
;; CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
;; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
;; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
;; PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
;; COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
;; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
;; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
;; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
;; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
;; OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
;; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
;; DAMAGE.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(in-package roslisp-utils)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Hash tables with general hash functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct gen-hash-table
  table
  hash-fn)
  

(defun make-hash-table* (&rest args &key (hash-fn #'identity hsupp) (test #'eql t-supp))
  "make-hash-table* &rest ARGS
Portable version of hash tables with nonstandard hash functions

ARGS are the arguments to hash table, with two additional keywords.

HASH-FN : a function of one argument that returns an object.  Defaults to #'identity.
TEST: a function of two arguments that must be one of #'equal, #'equalp, #'eql, or #'eq.  Defaults to #'eql.

The assumption will be that given two objects X and Y that are 'the same', their images under HASH-FN are equal w.r.t TEST."
  
  (assert (and args (is-standard-equality-test test)))
  (flet ((remove-if-necessary (args supp key)
	   (if supp
	       (aif (position key args)
		   (if (zerop it)
		       (cddr args)
		     (progn (setf (cdr (nthcdr (1- it) args)) (nthcdr (+ it 2) args)) args))
		 args)
	     args)))
	   
    (let ((actual-args (copy-list args)))
      (setf actual-args (remove-if-necessary actual-args hsupp ':hash-fn)
	    actual-args (remove-if-necessary actual-args t-supp ':test))
      (make-gen-hash-table :table (apply #'make-hash-table :test test actual-args) :hash-fn hash-fn))))

(defgeneric gethash* (key ht)
  (:documentation "gethash* KEY TABLE
To be used with tables created with make-hash-table*.  Also works for regular hash tables.")
  (:method (key (ht hash-table)) (gethash key ht))
  (:method (key (ht gen-hash-table))
	   (with-struct (gen-hash-table- table hash-fn) ht
	     (let ((p (gethash (funcall hash-fn key) table)))
	       (if p
		   (values (cdr p) t)
		 (values nil nil))))))


(defgeneric sethash* (key ht val)
  (:method (key (ht hash-table) val)
	   (setf (gethash key ht) val))
  (:method (key (ht gen-hash-table) val)
	   (with-struct (gen-hash-table- table hash-fn) ht
	     (let* ((k (funcall hash-fn key))
		    (p (gethash k table)))
	       (if p
		   (setf (cdr p) val)
		 (setf (gethash k table) (cons key val)))
	       val))))

(defgeneric hash-table-test* (ht)
  (:documentation "hash-table-test* GEN-HASH-TABLE.  Works for generalized hash tables.  Also, unlike hash-table-test, returns a function rather than a symbol.")
  (:method ((ht hash-table)) (symbol-function (hash-table-test ht)))
  (:method ((ht gen-hash-table)) (with-struct (gen-hash-table- table hash-fn) ht
				   (let ((test (symbol-function (hash-table-test table))))
				     #'(lambda (x y) (funcall test (funcall hash-fn x) (funcall hash-fn y)))))))
  
(defgeneric hash-table-count* (ht)
  (:method ((ht hash-table))
	   (hash-table-count ht))
  (:method ((ht gen-hash-table))
	   (hash-table-count (gen-hash-table-table ht))))

(defgeneric remhash* (key ht)
  (:method (key (ht hash-table))
	   (remhash key ht))
  (:method (key (ht gen-hash-table))
	   (with-struct (gen-hash-table- table hash-fn) ht
	     (remhash (funcall hash-fn key) table))))

(defgeneric hash-table-has-key (h k)
  (:documentation "hash-table-has-key HASHTABLE X.  Does the HASHTABLE contain (an item that satisfies the table's test together with) X?  Works with generalized hash tables also.")
  (:method ((h hash-table) x)
	   (mvbind (y pres)
	       (gethash x h)
	     (declare (ignore y))
	     pres))
  (:method ((h gen-hash-table) x)
	   (hash-table-has-key (gen-hash-table-table h) (funcall (gen-hash-table-hash-fn h) x))))

(defsetf gethash* sethash*)

(defgeneric hash-keys (h)
  (:documentation "hash-keys HASHTABLE.  Return list of keys in some order.  Works for generalized hash tables.")
  (:method ((h hash-table))
	   (loop for k being each hash-key in h collecting k))
  (:method ((h gen-hash-table))
	   (loop for v being each hash-value in (gen-hash-table-table h)
	       collect (car v))))

(defgeneric maphash* (fn h)
  (:documentation "maphash* FN H.  Like maphash* but works for generalized hash tables.")
  (:method (fn (h hash-table))
	   (maphash fn h))
  (:method (fn (h gen-hash-table))
	   (loop
	       for v being each hash-value in (gen-hash-table-table h)
	       do (funcall fn (car v) (cdr v)))))


(defmacro do-hash ((k v h) &rest body)
  `(maphash* #'(lambda (,k ,v) (declare (ignorable ,k ,v)) ,@body) ,h))

