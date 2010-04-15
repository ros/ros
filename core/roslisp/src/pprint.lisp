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

(defun pprint-ros-message (&rest args &aux str m)
  (if (= (length args) 1)
      (setf str t m (first args))
      (setf str (first args) m (second args)))
  
  (pprint-logical-block (str nil :prefix "[" :suffix "]")
    (let ((l (ros-message-to-list m)))
      (write (first l) :stream str)
      (dolist (f (rest l))
	(format str "~:@_  ~a:~:@_    ~w" (car f) (ros-message-to-list (cdr f))))))

)


(set-pprint-dispatch 'ros-message #'pprint-ros-message)

(defun pprint-hash (&rest args)
  (bind-pprint-args (s h) args
    (etypecase h
      (roslisp-utils::gen-hash-table (pprint-hash s (roslisp-utils::gen-hash-table-table h)))
      (hash-table
       (pprint-logical-block (s (roslisp-utils::hash-keys h)
				:prefix "["
				:suffix 
				(let ((not-shown 
				       (if *print-length*
					   (max 0 (- (hash-table-count h) *print-length*))
					   0)))
				  (if (> not-shown 0)
				      (format nil " and ~R other item~:*~P not shown here.]" not-shown)
				      "]")))

	 (when (> (roslisp-utils::hash-table-count* h) 0)
	   (loop
	      (let ((k (pprint-pop)))
		(format s "~a : ~a" k (gethash k h))
		(pprint-exit-if-list-exhausted)
		(pprint-newline :mandatory s)))))
       (values)))))

(defun print-debug-levels ()
  (let ((h (make-hash-table)))
    (loop 
      for k being each hash-key in *debug-levels*
      do (setf (gethash k h) (string-upcase (debug-level-string (gethash k *debug-levels*)))))
    (pprint-hash h)))
      
