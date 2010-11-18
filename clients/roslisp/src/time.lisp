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



(defvar *time-base* (unix-time)
  "Holds unix time (rounded to the nearest second) when roslisp was started")
(defvar *internal-time-base* (get-internal-real-time)
  "Holds CL's internal time when roslisp was started")



(defun ros-time ()
  "If *use-sim-time* is true (which is set upon node startup by looking up the ros /use_sim_time parameter), return the last received time on the /time or /clock topics, or 0.0 if no time message received yet. Otherwise, return the unix time (seconds since epoch)."
  (if *use-sim-time*
      (if *last-clock*
	  (rosgraph_msgs-msg:clock *last-clock*)
	  (progn
	    (unless (mutex-owner *debug-stream-lock*)
	      (ros-debug (roslisp time) "Returning time of 0.0 as use_sim_time was true and no clock messages received"))
	    0.0))
      (float (+ *time-base* (/ (- (get-internal-real-time) *internal-time-base*) internal-time-units-per-second)) 0.0L0)))

(defun spin-until-ros-time-valid ()
  (spin-until (> (ros-time) 0.0) 0.05
    (every-nth-time 100
      (ros-warn (roslisp time) "Waiting for valid ros-time before proceeding"))))


(defun wait-duration (d)
  "Wait until time T+D, where T is the current ros-time."
  (spin-until-ros-time-valid)
  (let ((until (+ (ros-time) d)))
    (spin-until (>= (ros-time) until) .01
      (every-nth-time 100
	(ros-debug (roslisp time) "In wait-duration spin loop; waiting until ~a" until)))))
    
    