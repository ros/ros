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


(defpackage roslisp-utils
  (:use :cl :sb-thread)
  (:export
   :mvbind
   :dbind
   :bind-pprint-args
   :force-format
   :until
   :while
   :repeat

   :unix-time
   :loop-at-most-every
   :spin-until
   :with-parallel-thread
   :every-nth-time

   :hash-table-has-key
   :pprint-hash
   :do-hash
   :tokens
   :serialize-int
   :deserialize-int
   :serialize-string
   :deserialize-string

   :filter

   :intern-compound-symbol
   
   :encode-single-float-bits
   :encode-double-float-bits
   :decode-single-float-bits
   :decode-double-float-bits))

(in-package :roslisp-utils)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Macros
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro aif (test-form then-form &optional else-form)
  `(let ((it ,test-form))
     (if it ,then-form ,else-form)))

(defmacro dbind (pattern object &body body)
  "abbreviation for destructuring-bind"
  `(destructuring-bind ,pattern ,object ,@body))

(defmacro mvbind (vars form &body body)
  "abbreviation for multiple-value-bind"
  `(multiple-value-bind ,vars ,form ,@body))

(defmacro until (test &body body)
  `(loop
      (when ,test (return))
      ,@body))

(defmacro while (test &body body)
  `(loop
      (unless ,test (return))
      ,@body))


(defmacro bind-pprint-args ((str obj) args &body body)
  "bind-pprint-args (STR OBJ) ARGS &rest BODY

STR, OBJ : unevaluated symbols
ARGS : a list (evaluated)

If ARGS has length 1, bind STR to t, OBJ to (FIRST ARGS).  Otherwise, bind STR to (first args) and OBJ to (second args) ARGS.  Then evaluate BDOY in this lexical context."
  
  (let ((x (gensym)))
    `(let ((,x ,args))
       (condlet
	(((= 1 (length ,x)) (,str t) (,obj (first ,x)))
	 (t (,str (first ,x)) (,obj (second ,x))))
	,@body))))

(defmacro repeat (n &body body)
  (let ((v (gensym)))
    `(dotimes (,v ,n)
       (declare (ignorable ,v))
       ,@body)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; condlet
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro condlet (clauses &body body)
  "condlet CLAUSES &rest BODY.  CLAUSES is a list of which each member is a conditional binding or otherwise clause.  There can be at most one otherwise clause and it must be the last clause.  Each conditional binding is a list where the first element is a test and the remaining elements are the bindings to be made if the test succeeds.   Each clause must bind the same set of variables.  If one of the tests succeeds, the corresponding bindings are made, and the body evaluated.  If none of the tests suceeds, the otherwise clause, if any, is evaluated instead of the body."
  (labels ((condlet-clause (vars cl bodfn)
	     `(,(car cl) (let ,(condlet-binds vars cl)
			   (,bodfn ,@(mapcar #'cdr vars)))))
	   
	   (condlet-binds (vars cl)
	     (mapcar #'(lambda (bindform)
			 (if (consp bindform)
			     (cons (cdr (assoc (car bindform) vars))
				   (cdr bindform))))
		     (cdr cl))))
	   
    (let* ((var-names (mapcar #'car (cdr (first clauses))))
	   (otherwise-clause? (eql (caar (last clauses)) 'otherwise))
	   (actual-clauses (if otherwise-clause? (butlast clauses) clauses)))
      (assert (every (lambda (cl) (equal var-names (mapcar #'car (cdr cl))))
		     actual-clauses)
	  nil "All non-otherwise-clauses in condlet must have same variables.")
      (let ((bodfn (gensym))
	    (vars (mapcar (lambda (v) (cons v (gensym)))
			  var-names)))
	`(labels ((,bodfn ,(mapcar #'car vars)
		    ,@body))
	   (cond 
	    ,@(mapcar (lambda (cl) (condlet-clause vars cl bodfn))
		      actual-clauses)
	    ,@(when otherwise-clause? `((t (progn ,@(cdar (last clauses))))))))))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Symbols
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun build-symbol-name (&rest args)
  "build-symbol-name S1 ... Sn.  Each Si is a string or symbol.  Concatenate them into a single long string (which can then be turned into a symbol using intern or find-symbol."
  (apply #'concatenate 'string (mapcar (lambda (x) (if (symbolp x) (symbol-name x) x)) args)))

(defun intern-compound-symbol (&rest args)
  "intern-compound-symbol S1 ... Sn.  Interns the result of build-symbol-name applied to the S."
  (intern (apply #'build-symbol-name args)))







(defmacro with-struct ((name . fields) s &body body)
  "with-struct (CONC-NAME . FIELDS) S &rest BODY

Example:
with-struct (foo- bar baz) s ...
is equivalent to
let ((bar (foo-bar s)) (baz (foo-baz s)))...

Note that despite the name, this is not like with-accessors or with-slots in that setf-ing bar above would not change the value of the corresponding slot in s."

  (let ((gs (gensym)))
    `(let ((,gs ,s))
       (let ,(mapcar #'(lambda (f)
			 `(,f (,(intern-compound-symbol name f) ,gs)))
	      fields)
	 ,@body))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Strings
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun tokens (string &key (start 0) (separators (list #\space #\return #\linefeed #\tab)))
  (if (= start (length string))
      '()
      (let ((p (position-if #'(lambda (char) (find char separators)) string :start start)))
        (if p
            (if (= p start)
                (tokens string :start (1+ start) :separators separators)
                (cons (subseq string start p)
                      (tokens string :start (1+ p) :separators separators)))
            (list (subseq string start))))))



(defun deserialize-int (istream)
  (let ((int 0))
    (setf (ldb (byte 8 0) int) (read-byte istream))
    (setf (ldb (byte 8 8) int) (read-byte istream))
    (setf (ldb (byte 8 16) int) (read-byte istream))
    (setf (ldb (byte 8 24) int) (read-byte istream))
    int))
		 

(defun deserialize-string (istream)
  (let* ((__ros_str_len (deserialize-int istream))
	 (str (make-string __ros_str_len)))
    (dotimes (i (length str) str)
      do (setf (char str i) (code-char (read-byte istream))))))


(defun serialize-int (int ostream)
  (write-byte (ldb (byte 8 0) int) ostream)
  (write-byte (ldb (byte 8 8) int) ostream)
  (write-byte (ldb (byte 8 16) int) ostream)
  (write-byte (ldb (byte 8 24) int) ostream))

(defun serialize-string (string ostream)
  (serialize-int (length string) ostream)
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) string))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Time, event loops
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun unix-time ()
  "seconds since 00:00:00, January 1, 1970"
  (- (get-universal-time) 2208988800))

(defun set-next-real-time-to-run (l time)
  (setf (car l) time))

(defun time-to-run (l)
  (car l))

(defun wait-until-ready (l)
  ;; Sbcl's sleep does not respect deadlines which causes severe
  ;; problems with CRAM. We don't use it directly. We explicitly take
  ;; into account deadlines.
  (let* ((current-time (get-internal-real-time))
         (time-to-run (time-to-run l))
         (deadline-seconds sb-impl::*deadline-seconds*)
         (stop-time (- time-to-run current-time)))
    (when (< current-time time-to-run)
      (tagbody
         :retry
         (cond ((or (not deadline-seconds)
                    (> deadline-seconds time-to-run))
                (sleep (/ stop-time internal-time-units-per-second)))
               (t
                (sleep deadline-seconds)
                (sb-sys:signal-deadline)
                (setq deadline-seconds sb-impl::*deadline-seconds*)
                (setf time-to-run (/ (float (- stop-time (get-internal-real-time)) 0.0d0)
                                     (float internal-time-units-per-second 0.0d0)))
                (when (plusp time-to-run)
                  (go :retry))))))))

(defun run-and-increment-delay (l d)
  (let ((next-time (+ (time-to-run l) (* d internal-time-units-per-second))))
    (wait-until-ready l)
    (set-next-real-time-to-run l next-time)))

(defmacro loop-at-most-every (d &body body)
  "Like loop, except ensures that BODY is executed at most once every D seconds (though possibly arbitrarily slower, if it takes a long time)"
  (let ((delay (gensym))
	(inc (gensym)))
    `(let ((,delay (list (get-internal-real-time)))
	   (,inc ,d))
       (loop
	  (run-and-increment-delay ,delay ,inc)
	  ,@body))))

(defmacro spin-until (test inc &body body)
  "Macro spin-until TEST INC &body BODY
or
spin-until TEST (INC TIMEOUT) &body BODY

Every INC seconds, evaluate the form TEST, and return its value it becomes true.

In the case with TIMEOUT specified, return two values: the first is the return value of TEST, and the second is T iff a timeout occurred (in which case the first value is meaningless)."
  
  (condlet (((listp inc) (actual-inc (first inc)) (timeout (second inc)))
            (t (actual-inc inc) (timeout nil)))
    (let ((init-time (gensym)) (test-var (gensym)))
      `(let ((,init-time (get-internal-real-time)))
         (declare (ignorable ,init-time))
         (loop-at-most-every ,actual-inc
           (let ((,test-var ,test))
             (when ,test-var (return ,test-var))
             ,(when timeout `(when (and ,timeout
                                        (> (get-internal-real-time)
                                           (+ ,init-time (* ,timeout ,internal-time-units-per-second))))
                               (return (values nil t))))
             ,@body))))))

       
(defmacro with-parallel-thread ((fn &optional name) &body body)
  "with-parallel-thread (FN NAME) &body BODY

Start a thread that executes FN, named NAME (which defaults to the symbol FN).  Then, in the current thread, execute BODY.  After BODY exits, terminate the newly started thread as well (typically BODY will be a long-running loop).

If FN is a symbol, it's replaced by (function FN).
"

  (unless name
    (assert (symbolp fn) nil "If name is not provided to with-parallel-thread, the (unevaluated) fn argument should be a symbol")
    (setq name `',fn))
  (let ((thread (gensym))
	(fn (if (symbolp fn) `#',fn fn)))
    `(let ((,thread (sb-thread:make-thread ,fn :name ,name)))
       (unwind-protect
	    (progn ,@body)
	 (sb-thread:terminate-thread ,thread)))))

(defvar *do-every-nth-table* (make-hash-table))
(defvar *do-every-nth-lock* (make-mutex :name "do-every-nth"))

(defun counter-value (id)
  (with-mutex (*do-every-nth-lock*)
    (let ((v (gethash id *do-every-nth-table*)))
      (setf (gethash id *do-every-nth-table*) (if v (1+ v) 1)))))

(defmacro every-nth-time (n &body body)
  (let ((id (gensym)))
    `(when (zerop (mod (counter-value ',id) ,n))
       ,@body)))
       
  
   
  
      
  



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Miscellaneous
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun force-format (str &rest args)
  "force-format STREAM &rest ARGS.  Like format, except flushes output on the stream."
  (apply #'format str args)
  (finish-output str))


(defparameter *standard-equality-tests* (list #'eq #'equal #'eql #'equalp 'eq 'eql 'equal 'equalp))

(defun is-standard-equality-test (f)
  (member f *standard-equality-tests*))

(defun filter (f l)
  (loop for x in l when (funcall f x) collect x))