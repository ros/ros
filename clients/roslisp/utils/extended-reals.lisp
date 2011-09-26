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

(defpackage :roslisp-extended-reals
  (:use :cl)
  (:export
   :infty :-infty
   :ext+ :ext- :extmax))


(in-package :roslisp-extended-reals)

;(declaim (inline ext+ ext- ext>))
(defun ext+ (&rest args)
  (reduce 'add-ext args))

(defun ext- (a &rest args)
  (if args
    (reduce 'subtract (cons a args))
    (negate a)))

(defun extmax (&rest args)
  (when args
    (let ((m '-infty))
      (dolist (x args m)
        (when (ext> x m)
          (setf m x))))))

(defun ext> (a b)
  (cond
    ((eql a 'infty) (not (eql b 'infty)))
    ((eql b '-infty) (not (eql a '-infty)))
    ((and (numberp a) (numberp b)) (> a b))))
    



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;(declaim (inline subtract negate))

(defun add-ext (a b)
  (cond ((symbolp a)
          (cond ((numberp b) a)
                ((eq a b) a)
                (t (assert nil nil "Can't add infty and -infty"))))
        ((symbolp b) b)
        (t (+ a b))))

(defun subtract (a b)
  (cond ((symbolp a)
          (cond ((numberp b) a)
                ((eq a b) (assert nil nil "Can't subtract infty or -infty from themselves."))
                (t a)))
        ((eql b 'infty) '-infty)
        ((eql b '-infty) 'infty)
        (t (- a b))))
	 

(defun negate (a)
  (case a
    (infty '-infty)
    (-infty 'infty)
    (otherwise (- a))))
  