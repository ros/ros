;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Message ops
;; Note: if using certain versions of the SLIME interface to 
;; SBCL, there are occasional errors when loading this file 
;; within SLIME if it was compiled from the command-line 
;; (or vice versa) - apparently the pprint-logical-block 
;; below compiles differently in the two cases.  This can be 
;; fixed by just touching this file, causing it to be 
;; recompiled.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package roslisp)



(defclass ros-message () ())

(defgeneric serialize (msg str)
  (:documentation "Serialize message object MSG onto stream STR."))

(defgeneric deserialize (msg str)
  (:documentation "Deserialize from stream STR into message object MSG and also returns MSG.  MSG may also be a symbol naming a message type, in which case a new object of that type is created and returned.")
  (:method ((msg symbol) str)
    (let ((m (make-instance msg)))
      (deserialize m str)
      m)))
	   

(defgeneric serialization-length (msg)
  (:documentation "Length of this message"))

(defgeneric md5sum (msg-type)
  (:documentation "Return the md5 sum of this message type.")
  (:method ((msg-type array))
    (if (stringp msg-type)
	(md5sum (get-topic-class-name msg-type))
	(progn
	  (warn "Hmmm... unexpected topic type specifier ~a in md5sum.  Passing it on anyway..." msg-type)
	  (call-next-method)))))

(defgeneric service-request-type (srv))
(defgeneric service-response-type (srv))

(defgeneric symbol-codes (msg-type)
  (:documentation "Return an association list from symbols to numbers (the const declarations in the .msg file).")
  (:method ((msg-type symbol)) nil))


(defgeneric symbol-code (msg-type symbol)
  (:documentation "symbol-code MSG-TYPE SYMBOL.  Gets the value of a message-specific constant declared in a msg file.  The first argument is either a symbol naming the message class, or an instance of the class, and the second argument is the keyword symbol corresponding to the constant name. 

For example, to get the value of the DEBUG constant from Log.msg, use (symbol-code '<Log> :debug).")
  (:method ((m ros-message) s) (symbol-code (class-of m) s))
  (:method ((m symbol) s)
    (let ((pair (assoc s (symbol-codes m))))
      (assert pair nil "Could not get symbol code for ~a for ROS message type ~a" s m)
      (cdr pair))))

(defgeneric ros-message-to-list (msg)
  (:documentation "Return a structured list representation of the message.  For example, say message type foo has a float field x equal to 42, and a field y which is itself a message of type bar, which has a single field z=24.  This function would then return the structured list '(foo (:x . 42.0) (:y . (bar (:z . 24)))).  

The return value can be passed to list-to-ros-message to retrieve an equivalent message to the original.

As a base case, non-ros messages just return themselves.")
  (:method (msg)
    (check-type msg (not ros-message) "something that is not a ros-message")
    msg))


(defgeneric list-to-ros-message (l)
  (:method ((l list))
    (apply #'make-instance (first l) (mapcan #'(lambda (pair) (list (car pair) (list-to-ros-message (cdr pair)))) (rest l))))
  (:method (msg)
    msg))


(defun read-ros-message (stream)
  (list-to-ros-message (read stream)))

(defun pprint-ros-message (&rest args &aux str m)
  (if (= (length args) 1)
      (setf str t m (first args))
      (setf str (first args) m (second args)))
  
  (pprint-logical-block (str nil :prefix "[" :suffix "]")
    (let ((l (ros-message-to-list m)))
      (write (first l) :stream str)
      (dolist (f (rest l))
	(format str "~:@_  ~a:~:@_    ~w" (car f) (cdr f)))))

)



(set-pprint-dispatch 'ros-message #'pprint-ros-message)
