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

(in-package roslisp)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Header messages need to have timestamp autofilled
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;; Define this here rather than in client.lisp because it's needed by serialize
(declaim (inline ros-time))
(defun ros-time ()
  "If *use-sim-time* is true (which is set upon node startup by looking up the ros /use_sim_time parameter), return the last received time on the /time topic, or 0.0 if no time message received yet. Otherwise, return the unix time (seconds since epoch)."
  (if *use-sim-time*
      (if *last-time*
	  (roslib-msg:rostime-val *last-time*)
	  0.0)
      (unix-time)))

(defvar *serialize-recursion-level* 0
  "Bound during calls to serialize, so we can keep track of when header time stamps need to be filled in")

(defmethod serialize :around (msg str)
  ;; Note that each thread has its own copy of the variable
  (let ((*serialize-recursion-level* (1+ *serialize-recursion-level*)))
    (call-next-method)))

(defmethod serialize :around ((msg roslib-msg:<Header>) str)

  ;; We save the old stamp for convenience when debugging interactively and reusing the same message object
  (let ((old-stamp (roslib-msg:stamp-val msg)))
    (unwind-protect

	 (progn
	   (when (and (= *serialize-recursion-level* 1) (= (roslib-msg:stamp-val msg) 0.0))
	     (setf (roslib-msg:stamp-val msg) (ros-time)))
	   (call-next-method))

      (setf (roslib-msg:stamp-val msg) old-stamp))))