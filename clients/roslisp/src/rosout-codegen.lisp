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


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; code generation for rosout
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun designated-list (x)
  (if (listp x) x (list x)))

(defgeneric make-keyword-symbol (s)
  (:method ((s symbol)) (intern (string-upcase (symbol-name s)) :keyword))
  (:method ((s string)) (intern (string-upcase s) :keyword)))

(defun can-write-to-log ()
  (and *ros-log-stream* (open-stream-p *ros-log-stream*)))

(defun rosout-msg (name level args)
  (let ((code (symbol-code 'rosgraph_msgs-msg:Log level)))
    (when (typep (first args) 'string) (push t args))
    (dbind (check str &rest format-args) args
      (let ((output-string (gensym)))
	`(when (and (>= ,code (debug-level ',(reverse (mapcar #'make-keyword-symbol (if (listp name) name (list name)))))) ,check)
	   (let ((,output-string (format nil ,str ,@format-args)))
	     (with-mutex (*debug-stream-lock*)
	       (format *debug-stream* "~&[~a ~a] ~,3F: ~a~&" ',(designated-list name) ,level (ros-time) ,output-string)
	       (when (can-write-to-log) 
		 (format *ros-log-stream* "~&[~a ~a] ~,3F: ~a~&" 
			 ',(designated-list name) ,level (ros-time) ,output-string)))
	     
	     (force-output *debug-stream*)
	     (when (can-write-to-log) (force-output *ros-log-stream*))

	     (when (and (eq *node-status* :running) (gethash "/rosout" *publications*))
	       (publish 
		"/rosout"
		(make-instance 'rosgraph_msgs-msg:Log :name *ros-node-name*
			       :level ,code :msg ,output-string)))))))))
