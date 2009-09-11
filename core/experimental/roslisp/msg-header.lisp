(in-package roslisp)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Header messages need to have timestamp autofilled
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(load-message-types "roslib/Header")
(load-message-types "roslib/Time")


;; Define this here rather than in client.lisp because it's needed by serialize
(declaim (inline ros-time))
(defun ros-time ()
  (if *use-sim-time*
      (if *last-time*
	  (roslib:rostime-val *last-time*)
	  0.0)
      (unix-time)))

(defvar *serialize-recursion-level* 0
  "Bound during calls to serialize, so we can keep track of when header time stamps need to be filled in")

(defmethod serialize :around (msg str)
  ;; Note that each thread has its own copy of the variable
  (let ((*serialize-recursion-level* (1+ *serialize-recursion-level*)))
    (call-next-method)))

(defmethod serialize :around ((msg roslib::<Header>) str)

  ;; We save the old stamp for convenience when debugging interactively and reusing the same message object
  (let ((old-stamp (roslib::stamp-val msg)))
    (unwind-protect

	 (progn
	   (when (and (= *serialize-recursion-level* 1) (= (roslib::stamp-val msg) 0.0))
	     (setf (roslib::stamp-val msg) (ros-time)))
	   (call-next-method))

      (setf (roslib::stamp-val msg) old-stamp))))