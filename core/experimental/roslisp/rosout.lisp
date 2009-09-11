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
;; At runtime, maps debug topics to numerical levels
;; Given a debug topic (foo bar baz), we'll look at 
;; (foo bar baz), then (foo bar), then (foo), then nil
;; till we find one that's in the table.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; We hash from lists to numbers.  Could make more efficient using a tree of hashtables instead.
(defvar *debug-levels* (make-hash-table :test #'equal))

(defun debug-level (name)
  (or (gethash name *debug-levels*) (debug-level (cdr name))))

(defun set-debug-level (name level)
  (when (symbolp level) (setq level (symbol-code 'roslib-msg:<Log> level)))
  (setf (gethash (reverse (designated-list name)) *debug-levels*) level)
  (ros-debug (roslisp rosout) "Setting debug level of ~a to ~a" (designated-list name) level))

(defmacro set-debug-levels (&rest args)
  "set-debug-level NAME1 LEVEL1 ... NAMEk LEVELk

Each NAME (unevaluated) is a list, e.g. (roslisp tcp) denoting a debugger topic.  LEVEL is one of the keyword symbols :debug, :info, :warn, :error, or :fatal."
  
  (labels
      ((helper (args)
	 (when args
	   `((set-debug-level ',(car args) ,(cadr args))
	     ,@(helper (cddr args))))))
    `(progn ,@(helper args))))
      

;; The default level for unknown topics is :warn
(set-debug-level nil (symbol-code 'roslib-msg:<Log> :warn))
