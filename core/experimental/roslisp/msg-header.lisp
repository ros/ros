(in-package roslisp)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Header messages need to have timestamp autofilled
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros-load-message-types "roslib/Header")

(defvar *serialize-recursion-level* 0
  "Bound during calls to serialize, so we can keep track of when header time stamps need to be filled in")

(defmethod serialize :around (msg str)
  (let ((*serialize-recursion-level* (1+ *serialize-recursion-level*)))
    (call-next-method)))

(defmethod serialize :around ((msg roslib::<Header>) str)

  ;; Remove once sure it works
  (when (not (= *serialize-recursion-level* 1))
    (format *log* "in header serialize, rec level is ~a" *serialize-recursion-level*))

  (when (and (= *serialize-recursion-level* 1) (= (roslib::stamp-val msg) 0.0))

    ;; NOTE: if removing next line, don't forget about its side effects!
    (format *log* "Assuming header stamp of 0.0 on ~a means it hasn't been set, so setting it to ~a" msg (setf (roslib::stamp-val msg) (unix-time)))

    ;; TODO: instead, have a flag when header is set, or customize the initialization

    ;; Set time in seconds (not providing nanoseconds)
    )
  (call-next-method))