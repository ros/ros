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

(defun process-command-line-remappings (l base-name)
  "Process command line remappings, including the three special cases for remapping the node name, namespace, and setting parameters.  Return alist of params to set.  Note this is order dependent since setting __ns or __name affects how remappings are interpreted."
  (setf *remapped-names* (make-hash-table :test #'equal))
  (let ((params nil))
    (dolist (x l params)
      (dbind (lhs rhs) x
	(cond
	  ((equal lhs "__ns") (setf *namespace* (postprocess-namespace rhs) *ros-node-name* (compute-node-name base-name)))
	  ((equal lhs "__name") (setf base-name rhs *ros-node-name* (compute-node-name rhs)))
	  ((equal lhs "__log") (setf *ros-log-location* rhs))
	  ((eql (char lhs 0) #\_) (push (cons (concatenate 'string "~" (subseq lhs 1)) 
					      (let ((rhs-val (read-from-string rhs)))
						(typecase rhs-val
						  (symbol rhs)
						  (otherwise rhs-val)))) params))
	  (t (setf (gethash (compute-global-name *namespace* *ros-node-name* lhs) *remapped-names*)
                   (compute-global-name *namespace* *ros-node-name* rhs))))))))

(defun postprocess-namespace (ns)
  "Ensure that namespace begins and ends with /"
  (unless (eql (char ns 0) #\/)
    (setf ns (concatenate 'string "/" ns)))
  (unless (eql (char ns (1- (length ns))) #\/)
    (setf ns (concatenate 'string ns "/")))
  ns)

(defun compute-node-name (name)
  (concatenate 'string *namespace* (string-trim '(#\/) name)))

(defun parse-remapping (string)
  "If string is of the form FOO:=BAR, return foo and bar, otherwise return nil."
  (let ((i (search ":=" string)))
    (when i
      (values (subseq string 0 i) (subseq string (+ i 2))))))



(defun handle-command-line-arguments (name args)
  "Postcondition: the variables *remapped-names*, *namespace*, and *ros-node-name* are set based on the argument list and the environment variable ROS_NAMESPACE as per the ros command line protocol.  Also, arguments of the form _foo:=bar are interpreted by setting private parameter foo equal to bar (currently bar is just read using the lisp reader; it should eventually use yaml conventions)"
  (when (stringp args)
    (setq args (tokens args)))
  (setq *namespace* (postprocess-namespace (or (sb-ext:posix-getenv "ROS_NAMESPACE") "/"))
        *ros-node-name* (compute-node-name name))
  (let ((remappings
	 (mapcan #'(lambda (s) (mvbind (lhs rhs) (parse-remapping s) (when lhs (list (list lhs rhs))))) 
		 args)))
    (process-command-line-remappings remappings name)))


(defun command-line-args-rosout (args params)
  "Separate function for debug out that's called after the debug levels are set"
  (ros-debug (roslisp top) "Command line arguments are ~a" args)
  (ros-info (roslisp top) "Node name is ~a" *ros-node-name*)
  (ros-info (roslisp top) "Namespace is ~a" *namespace*)
  (ros-info (roslisp top) "Params are ~a" params)
  (ros-info (roslisp top) "Remappings are:")
  (maphash #'(lambda (k v) (ros-info (roslisp top) "  ~a = ~a" k v)) *remapped-names*))



