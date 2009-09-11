(defpackage extended-reals
  (:use cl)
  (:export
   infty
   -infty
   ext+
   ext-
   extmax))


(in-package extended-reals)

;(declaim (inline ext+ ext- ext>))
(defun ext+ (&rest args)
  (if args
      (let ((x (first args))
	    (y (apply #'ext+ (rest args))))
	(cond
	  ((symbolp x)
	   (cond
	     ((numberp y) x)
	     ((eq x y) x)
	     (t (assert nil nil "Can't add infty and -infty"))))
	  ((symbolp y) y)
	  (t (+ x y))))))

(defun ext- (a &rest args)
  (if args
      (if (rest args)
	  (apply #'ext- (subtract a (first args)) (rest args))
	  (subtract a (first args)))
      (negate a)))

(defun extmax (&rest args)
  (let ((m '-infty))
    (dolist (x args m)
      (when (ext> x m)
	(setf m x)))))

(defun ext> (a b)
  (cond
    ((eql a 'infty) (not (eql b 'infty)))
    ((eql b '-infty) (not (eql a '-infty)))
    ((and (numberp a) (numberp b)) (> a b))))
    



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;(declaim (inline subtract negate))

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
  