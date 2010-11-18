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

(in-package :roslisp)



;; We hash from lists to numbers.  Could make more efficient using a tree of hashtables instead.
(defvar *debug-levels* (make-hash-table :test #'equal))

(defun debug-level-exists (l)
  (hash-table-has-key *debug-levels* (mapcar #'make-keyword-symbol (reverse l))))

(defun debug-level (name)  
  (or (gethash name *debug-levels*) 
      (if name
	  (debug-level (cdr name))
	  (level-code :info))))
      

(defun debug-topic-param (name)
  (concatenate-ros-names (cons "~debug" (nconc (mapcar #'string-downcase (designated-list name)) (list "level")))))

(defgeneric debug-level-string (level)
  (:method ((level symbol)) (string-downcase (symbol-name level)))
  (:method ((level fixnum)) (debug-level-string (car (rassoc level (symbol-codes 'rosgraph_msgs-msg:<Log>))))))


(defgeneric level-code (level)
  (:method ((level fixnum)) level)
  (:method ((level symbol)) (symbol-code 'rosgraph_msgs-msg:<Log> level))
  (:method ((level string)) (level-code (find-symbol level :keyword))))

(defun set-local-debug-level (topic level &optional (h *debug-levels*))
  (ros-debug (roslisp rosout) "Locally setting debug level of ~a to ~a" topic level)
  (let ((debug-topic (mapcar #'make-keyword-symbol (reverse topic))))
    (setf (gethash debug-topic h) (level-code level))))

(defun set-debug-level (name level)
  (set-local-debug-level (designated-list name) level *debug-levels*)
  (when (eq *node-status* :running)
    (ros-debug (roslisp rosout) "Setting ros parameter for debug level of ~a to ~a" (designated-list name) level)
    (set-param (debug-topic-param name) (debug-level-string level)))
  )

(defun set-debug-level-unless-exists (name level)
  (unless (debug-level-exists name)
    (set-debug-level name level)))

(defmacro set-debug-levels (&rest args)
  "set-debug-level NAME1 LEVEL1 ... NAMEk LEVELk

Each NAME (unevaluated) is a list, e.g. (roslisp tcp) denoting a debugger topic.  LEVEL is one of the keyword symbols :debug, :info, :warn, :error, or :fatal."
  
  (labels
      ((helper (args)
	 (when args
	   `((set-debug-level ',(car args) ,(cadr args))
	     ,@(helper (cddr args))))))
    `(progn ,@(helper args))))


(defun is-prefix (s1 s2)
  (when (<= (length s1) (length s2))
    (search s1 s2 :end2 (length s1))))

(defun is-suffix (s1 s2)
  (let ((l1 (length s1)) (l2 (length s2)))
    (and (<= l1 l2)
	 (search s1 s2 :start2 (- l2 l1)))))

(defun is-debug-level-param (p)
  (and (is-prefix (fully-qualified-name "~debug") p)
       (is-suffix "/level" p)))

(defun get-debug-topic (p)
  (let* ((prefix (fully-qualified-name "~debug"))
	 (tokens (tokens p :start (length prefix) :separators '(#\/))))
    (subseq tokens 0 (1- (length tokens)))))


(def-service-callback (reset-debug-levels Empty) ()
  (dolist (param (list-params "~debug"))
    (when (is-debug-level-param param)
      (let ((level (string-upcase (get-param param))))
        (if (member level '("DEBUG" "INFO" "WARN" "ERROR" "FATAL") :test #'equal)
            (set-local-debug-level (get-debug-topic param) level)
            (ros-warn (roslisp rosout) "Skipping setting debug level of ~a to unknown level ~a" param level)))))
  (make-response))

    
