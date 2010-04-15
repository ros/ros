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
;; Called by user
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro ros-debug (name &rest args)
  "ros-debug NAME [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, and the debug level of debug topic NAME is at least :debug, output the given format string and arguments to stdout and publish on rosout if possible.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg name :debug args))

(defmacro ros-info (name &rest args)
  "ros-info NAME [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, and the debug level of debug topic NAME is at least :info, output the given format string and arguments to stdout and publish on rosout if possible.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg name :info args))

(defmacro ros-warn (name &rest args)
  "ros-warn NAME [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, and the debug level of debug topic NAME is at least :warn, output the given format string and arguments to stdout and publish on rosout if possible.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg name :warn args))

(defmacro ros-error (name &rest args)
  "ros-error NAME [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, and the debug level of debug topic NAME is at least :error, output the given format string and arguments to stdout and publish on rosout if possible.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg name :error args))

(defmacro ros-fatal (name &rest args)
  "ros-fatal NAME [CONDITION] FORMAT-STRING . ARGS
When CONDITION is true, and the debug level of debug topic NAME is at least :fatal, output the given format string and arguments to stdout and publish on rosout if possible.  
Otherwise do nothing; in particular, don't evaluate the ARGS.
CONDITION can be omitted if the FORMAT-STRING is a literal string, in which case it defaults to t."
  (rosout-msg name :fatal args))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Some standard errors that can be declared now that
;; the macros are defined
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod serialization-length ((msg t))
  (ros-error roslisp "Hmm... unexpectedly asked for serialization length of ~w.  Most likely because the aforementioned object was found some place where a (nonprimitive) ros message was expected." msg)
  42)

