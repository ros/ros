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
;; Special case code for message headers
;; Not currently used
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

#|

(defvar *serialize-recursion-level* 0
  "Bound during calls to serialize, so we can keep track of when header time stamps need to be filled in")

(defvar *seq* 0)
(defvar *seq-lock* (make-mutex :name "lock on seq global variable")) ;; not currently used
(defvar *set-seq* nil) ;; For now not setting seq fields as doesn't seem to be necessary for ROS

(defmethod serialize :around (msg str)
  ;; Note that each thread has its own copy of the variable
  (let ((*serialize-recursion-level* (1+ *serialize-recursion-level*)))
    (call-next-method)))

(defmethod serialize :around ((msg rosgraph_msgs-msg:<Header>) str)

  ;; We save the old stamp for convenience when debugging interactively and reusing the same message object
  (let ((old-stamp (rosgraph_msgs-msg:stamp msg)))
    (unwind-protect
	 (progn
	   (when (= *serialize-recursion-level* 1)
	     (when *set-seq*
	       (setf (rosgraph_msgs-msg:seq msg) (incf *seq*)))
	     (when (= (rosgraph_msgs-msg:stamp msg) 0.0)
	       (setf (rosgraph_msgs-msg:stamp msg) (ros-time))))
	   (call-next-method))

      (setf (rosgraph_msgs-msg:stamp msg) old-stamp))))

|#