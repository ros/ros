;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: xml-rpc.lisp,v 1.4 2004/06/17 19:43:11 rschlatte Exp $
;;;;
;;;; This is a Common Lisp implementation of the XML-RPC protocol,
;;;; as documented on the website http://www.xmlrpc.com
;;;; This implementation includes both a client and server part.
;;;; A Base64 encoder/decoder and a minimal XML parser are required.
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml-rpc)

;;; conditions

(define-condition xml-rpc-condition (error)
  ()
  (:documentation "Parent condition for all conditions thrown by the XML-RPC package"))

(define-condition xml-rpc-fault (xml-rpc-condition)
  ((code :initarg :code :reader xml-rpc-fault-code)
   (string :initarg :string :reader xml-rpc-fault-string))
  (:report (lambda (condition stream)
	     (format stream
		     "XML-RPC fault with message '~a' and code ~d."
		     (xml-rpc-fault-string condition)
		     (xml-rpc-fault-code condition))))
  (:documentation "This condition is thrown when the XML-RPC server returns a fault"))

(setf (documentation 'xml-rpc-fault-code 'function) "Get the code from an XML-RPC fault")
(setf (documentation 'xml-rpc-fault-string 'function) "Get the string from an XML-RPC fault")

(define-condition xml-rpc-error (xml-rpc-condition)
  ((place :initarg :code :reader xml-rpc-error-place)
   (data :initarg :data :reader xml-rpc-error-data))
  (:report (lambda (condition stream)
	     (format stream
		     "XML-RPC error ~a at ~a."
		     (xml-rpc-error-data condition)
		     (xml-rpc-error-place condition))))
  (:documentation "This condition is thrown when an XML-RPC protocol error occurs"))

(setf (documentation 'xml-rpc-error-place 'function)
      "Get the place from an XML-RPC error"
      (documentation 'xml-rpc-error-data 'function)
      "Get the data from an XML-RPC error")

;;; iso8601 support (the xml-rpc variant)

(defun universal-time->iso8601 (time &optional (stream nil))
  "Convert a Common Lisp universal time to a string in the XML-RPC variant of ISO8601"
  (multiple-value-bind (second minute hour date month year)
      (decode-universal-time time)
    (format stream
	    "~d~2,'0d~2,'0dT~2,'0d:~2,'0d:~2,'0d"
	    year
	    month
	    date
	    hour
	    minute
	    second)))

(defun iso8601->universal-time (string)
  "Convert string in the XML-RPC variant of ISO8601 to a Common Lisp universal time"
  (let (year month date (hour 0) (minute 0) (second 0))
    (when (< (length string) 9)
      (error "~s is to short to represent an iso8601" string))
    (setf year (parse-integer string :start 0 :end 4)
	  month (parse-integer string :start 4 :end 6)
	  date (parse-integer string :start 6 :end 8))
    (when (and (>= (length string) 17) (char= #\T (char string 8)))
      (setf hour (parse-integer string :start 9 :end 11)
	    minute (parse-integer string :start 12 :end 14)
	    second (parse-integer string :start 15 :end 17)))
    (encode-universal-time second minute hour date month year)))

(defstruct (xml-rpc-time (:print-function print-xml-rpc-time))
  "A wrapper around a Common Lisp universal time to be interpreted as an XML-RPC-TIME"
  universal-time)

(setf (documentation 'xml-rpc-time-p 'function)
      "Return T when the argument is an XML-RPC time"
      (documentation 'xml-rpc-time-universal-time 'function)
      "Return the universal time from an XML-RPC time")

(defun print-xml-rpc-time (xml-rpc-time stream depth)
  (declare (ignore depth))
  (format stream
	  "#<XML-RPC-TIME ~a>"
	  (universal-time->iso8601 (xml-rpc-time-universal-time xml-rpc-time))))

(defun xml-rpc-time (&optional (universal-time (get-universal-time)))
  "Create a new XML-RPC-TIME struct with the universal time specified, defaulting to now"
  (make-xml-rpc-time :universal-time universal-time))

;;; a wrapper for literal strings, where escaping #\< and #\& is not
;;; desired

(defstruct (xml-literal (:print-function print-xml-literal))
  "A wrapper around a Common Lisp string that will be sent over
  the wire unescaped"
  content)

(setf (documentation 'xml-literal-p 'function)
      "Return T when the argument is an unescaped xml string"
      (documentation 'xml-literal-content 'function)
      "Return the content of a literal xml string")

(defun print-xml-literal (xml-literal stream depth)
  (declare (ignore depth))
  (format stream
	  "#<XML-LITERAL \"~a\" >"
	  (xml-literal-content xml-literal)))

(defun xml-literal (content)
  "Create a new XML-LITERAL struct with the specified content."
  (make-xml-literal :content content))

;;; an extra datatype for xml-rpc structures (associative maps)

(defstruct (xml-rpc-struct (:print-function print-xml-rpc-struct))
  "An XML-RPC-STRUCT is an associative map of member names and values"
  alist)

(setf (documentation 'xml-rpc-struct-p 'function)
      "Return T when the argument is an XML-RPC struct"
      (documentation 'xml-rpc-struct-alist 'function)
      "Return the alist of member names and values from an XML-RPC struct")

(defun print-xml-rpc-struct (xml-element stream depth)
  (declare (ignore depth))
  (format stream "#<XML-RPC-STRUCT~{ ~S~}>" (xml-rpc-struct-alist xml-element)))

(defun get-xml-rpc-struct-member (struct member)
  "Get the value of a specific member of an XML-RPC-STRUCT"
  (cdr (assoc member (xml-rpc-struct-alist struct))))

(defun (setf get-xml-rpc-struct-member) (value struct member)
  "Set the value of a specific member of an XML-RPC-STRUCT"
  (let ((pair (assoc member (xml-rpc-struct-alist struct))))
    (if pair
	(rplacd pair value)
        (push (cons member value) (xml-rpc-struct-alist struct)))
    value))

(defun xml-rpc-struct (&rest args)
  "Create a new XML-RPC-STRUCT from the arguments: alternating member names and values"
  (unless (evenp (length args))
    (error "~s must contain an even number of elements" args))
  (let (alist)
    (loop
       (if (null args)
           (return)
           (push (cons (pop args) (pop args)) alist)))
    (make-xml-rpc-struct :alist alist)))

(defun xml-rpc-struct-equal (struct1 struct2)
  "Compare two XML-RPC-STRUCTs for equality"
  (if (and (xml-rpc-struct-p struct1)
	   (xml-rpc-struct-p struct2)
	   (= (length (xml-rpc-struct-alist struct1))
	      (length (xml-rpc-struct-alist struct2))))
      (dolist (assoc (xml-rpc-struct-alist struct1) t)
	(unless (equal (get-xml-rpc-struct-member struct2 (car assoc))
		       (cdr assoc))
	  (return-from xml-rpc-struct-equal nil)))
      nil))

;;; encoding support

(defun encode-xml-rpc-struct (struct stream)
  (princ "<struct>" stream)
  (dolist (member (xml-rpc-struct-alist struct))
    (princ "<member>" stream)
    (format stream "<name>~a</name>" (car member)) ; assuming name contains no special characters
    (encode-xml-rpc-value (cdr member) stream)
    (princ "</member>" stream))
  (princ "</struct>" stream))

(defun encode-xml-rpc-array (sequence stream)
  (princ "<array><data>" stream)
  (map 'nil #'(lambda (element) (encode-xml-rpc-value element stream)) sequence)
  (princ "</data></array>" stream))

(defun encode-xml-rpc-value (arg stream)
  (princ "<value>" stream)
  (cond ((integerp arg) (format stream "<int>~d</int>" arg))
        ((floatp arg) (format stream "<double>~f</double>" arg))
        ((or (null arg) (eq arg t))
         (princ "<boolean>" stream)
         (princ (if arg 1 0) stream)
         (princ "</boolean>" stream))
        ((or (stringp arg) (symbolp arg))
         (princ "<string>" stream)
         (print-string-xml (string arg) stream)
         (princ "</string>" stream))
        ((and (arrayp arg)
              (= (array-rank arg) 1)
              (subtypep (array-element-type arg)
                        '(unsigned-byte 8)))
         (princ "<base64>" stream)
         (encode-base64-bytes arg stream)
         (princ "</base64>" stream))
        ((xml-rpc-time-p arg)
         (princ "<dateTime.iso8601>" stream)
         (universal-time->iso8601 (xml-rpc-time-universal-time arg) stream)
         (princ "</dateTime.iso8601>" stream))
        ((xml-literal-p arg)
         (princ (xml-literal-content arg) stream))
        ((or (listp arg) (vectorp arg)) (encode-xml-rpc-array arg stream))
        ((xml-rpc-struct-p arg) (encode-xml-rpc-struct arg stream))
        ;; add generic method call
        (t (error "cannot encode ~s" arg)))
  (princ "</value>" stream))

(defun encode-xml-rpc-args (args stream)
  (princ "<params>" stream)
  (dolist (arg args)
    (princ "<param>" stream)
    (encode-xml-rpc-value arg stream)
    (princ "</param>" stream))
  (princ "</params>" stream))

(defun encode-xml-rpc-call (name &rest args)
  "Encode an XML-RPC call with name and args as an XML string"
  (with-output-to-string (stream)
    (princ "<methodCall>" stream)
    ;; Spec says: The string may only contain identifier characters,
    ;; upper and lower-case A-Z, the numeric characters, 0-9,
    ;; underscore, dot, colon and slash.
    (format stream "<methodName>~a</methodName>" (string name)) ; assuming name contains no special characters
    (when args
      (encode-xml-rpc-args args stream))
    (princ "</methodCall>" stream)))

(defun encode-xml-rpc-result (value)
  (with-output-to-string (stream)
    (princ "<methodResponse>" stream)
    (encode-xml-rpc-args (list value) stream)
    (princ "</methodResponse>" stream)))

(defun encode-xml-rpc-fault-value (fault-string &optional (fault-code 0))
  ;; for system.multicall
  (with-output-to-string (stream)
    (princ "<struct>" stream)
    (format stream "<member><name>faultCode</name><value><int>~d</int></value></member>" fault-code)
    (princ "<member><name>faultString</name><value><string>" stream)
    (print-string-xml fault-string stream)
    (princ "</string></value></member>" stream)
    (princ "</struct>" stream)))

(defun encode-xml-rpc-fault (fault-string &optional (fault-code 0))
  (with-output-to-string (stream)
    (princ "<methodResponse><fault><value>" stream)
    (princ (encode-xml-rpc-fault-value fault-string fault-code) stream)
    (princ "</value></fault></methodResponse>" stream)))

;;; decoding support

(defun decode-xml-rpc-new-element (name attributes seed)
  (declare (ignore seed name attributes))
  '())

(defun decode-xml-rpc-finish-element (name attributes parent-seed seed)
  (declare (ignore attributes))
  (cons (case name
	  ((:|int| :|i4|) (parse-integer seed))
	  (:|double| (read-from-string seed))
	  (:|boolean| (= 1 (parse-integer seed)))
	  (:|string| (if (null seed) "" seed))
	  (:|dateTime.iso8601| (xml-rpc-time (iso8601->universal-time seed)))
	  (:|base64| (if (null seed)
			 (make-array 0 :element-type '(unsigned-byte 8))
		       (with-input-from-string (in seed)
			 (decode-base64-bytes in))))
	  (:|array| (car seed))
	  (:|data| (nreverse seed))
	  (:|value| (if (stringp seed) seed (car seed)))
	  (:|struct| (make-xml-rpc-struct :alist seed))
	  (:|member| (cons (cadr seed) (car seed)))
	  (:|name| (intern seed :keyword))
	  (:|params| (nreverse seed))
	  (:|param| (car seed))
	  (:|fault| (make-condition 'xml-rpc-fault
				    :string (get-xml-rpc-struct-member (car seed) :|faultString|)
				    :code (get-xml-rpc-struct-member (car seed) :|faultCode|)))
	  (:|methodName| seed)
	  (:|methodCall| (let ((pair (nreverse seed)))
			   (cons (car pair) (cadr pair))))
	  (:|methodResponse| (car seed)))
	parent-seed))

(defun decode-xml-rpc-text (string seed)
  (declare (ignore seed))
  string)

(defun decode-xml-rpc (stream)
  (car (start-parse-xml stream
                        (make-instance 'xml-parser-state
                                       :new-element-hook #'decode-xml-rpc-new-element
                                       :finish-element-hook #'decode-xml-rpc-finish-element
                                       :text-hook #'decode-xml-rpc-text))))

;;; networking basics

(defparameter *xml-rpc-host* "localhost"
  "String naming the default XML-RPC host to use")

(defparameter *xml-rpc-port* 80
  "Integer specifying the default XML-RPC port to use")

(defparameter *xml-rpc-url* "/RPC2"
  "String specifying the default XML-RPC URL to use")

(defparameter *xml-rpc-agent* (concatenate 'string
					   (lisp-implementation-type)
					   " "
					   (lisp-implementation-version))
  "String specifying the default XML-RPC agent to include in server responses")

(defvar *xml-rpc-debug* nil
  "When T the XML-RPC client and server part will be more verbose about their protocol")

(defvar *xml-rpc-debug-stream* nil
  "When not nil it specifies where debugging output should be written to")

(defparameter *xml-rpc-proxy-host* nil
  "When not null, a string naming the XML-RPC proxy host to use")

(defparameter *xml-rpc-proxy-port* nil
  "When not null, an integer specifying the XML-RPC proxy port to use")

(defparameter *xml-rpc-package* (find-package :s-xml-rpc-exports)
  "Package for XML-RPC callable functions")

(defparameter *xml-rpc-authorization* nil
  "When not null, a string to be used as Authorization header")

(defun format-debug (&rest args)
  (when *xml-rpc-debug*
    (apply #'format args)))

(defparameter +crlf+ (make-array 2
                                 :element-type 'character
                                 :initial-contents '(#\return #\linefeed)))

(defun tokens (string &key (start 0) (separators (list #\space #\return #\linefeed #\tab)))
  (if (= start (length string))
      '()
      (let ((p (position-if #'(lambda (char) (find char separators)) string :start start)))
        (if p
            (if (= p start)
                (tokens string :start (1+ start) :separators separators)
                (cons (subseq string start p)
                      (tokens string :start (1+ p) :separators separators)))
            (list (subseq string start))))))

(defun format-header (stream headers)
  (mapc #'(lambda (header)
            (cond ((null (rest header)) (write-string (first header) stream) (princ +crlf+ stream))
                  ((second header) (apply #'format stream header) (princ +crlf+ stream))))
	headers)
  (princ +crlf+ stream))

(defun debug-stream (in)
  ;; Uggh, this interacts badly with SLIME, so turning it off for now... - BMM
  #|
  (if *xml-rpc-debug*
  (make-echo-stream in *standard-output*)
  in)) |#
  in)

;;; client API

(defun xml-rpc-call (encoded &key
                     (url *xml-rpc-url*)
                     (agent *xml-rpc-agent*)
                     (host *xml-rpc-host*)
                     (port *xml-rpc-port*)
                     (authorization *xml-rpc-authorization*)
                     (proxy-host *xml-rpc-proxy-host*)
                     (proxy-port *xml-rpc-proxy-port*))
  "Execute an already encoded XML-RPC call and return the decoded result"
  (let ((uri (if proxy-host (format nil "http://~a:~d~a" host port url) url)))
    (with-open-socket-stream (connection (if proxy-host proxy-host host)
                                         (if proxy-port proxy-port port))
      (format-debug (or *xml-rpc-debug-stream* t) "POST ~a HTTP/1.0~%Host: ~a:~d~%" uri host port)
      (format-header connection `(("POST ~a HTTP/1.0" ,uri)
				  ("User-Agent: ~a" ,agent)
				  ("Host: ~a:~d" ,host ,port)
                                  ("Authorization: ~a" ,authorization)
				  ("Content-Type: text/xml")
				  ("Content-Length: ~d" ,(length encoded))))
      (princ encoded connection)
      (finish-output connection)
      (format-debug (or *xml-rpc-debug-stream* t) "Sending ~a~%~%" encoded)
      (let ((header (read-line connection nil nil)))
	(when (null header) (error "no response from server"))
	(format-debug (or *xml-rpc-debug-stream* t) "~a~%" header)
	(setf header (tokens header))
	(unless (and (>= (length header) 3)
		     (string-equal (second header) "200")
		     (string-equal (third header) "OK"))
	  (error "http-error:~{ ~a~}" header)))
      (do ((line (read-line connection nil nil)
		 (read-line connection nil nil)))
	  ((or (null line) (= 1 (length line))))
	(format-debug (or *xml-rpc-debug-stream* t) "~a~%" line))
      (let ((result (decode-xml-rpc (debug-stream connection))))
	(if (typep result 'xml-rpc-fault)
	    (error result)
            (car result))))))

(defun call-xml-rpc-server (server-keywords name &rest args)
  "Encode and execute an XML-RPC call with name and args, using the list of server-keywords"
  (apply #'xml-rpc-call
	 (cons (apply #'encode-xml-rpc-call (cons name args))
	       server-keywords)))

(defun describe-server (&key (host *xml-rpc-host*) (port *xml-rpc-port*) (url *xml-rpc-url*))
  "Tries to describe a remote server using system.* methods"
  (dolist (method (xml-rpc-call (encode-xml-rpc-call "system.listMethods")
				:host host
				:port port
				:url url))
    (format t
	    "Method ~a ~a~%~a~%~%"
	    method
	    (xml-rpc-call (encode-xml-rpc-call "system.methodSignature" method)
			  :host host
			  :port port
			  :url url)
	    (xml-rpc-call (encode-xml-rpc-call "system.methodHelp" method)
			  :host host
			  :port port
			  :url url))))


;;; server API

(defvar *xml-rpc-call-hook* 'execute-xml-rpc-call
  "A function to execute the xml-rpc call and return the result, accepting a method-name string and a optional argument list")

(defparameter +xml-rpc-method-characters+
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.:/")

(defun valid-xml-rpc-method-name-p (method-name)
  (not (find-if-not (lambda (c) (find c +xml-rpc-method-characters+))
                    method-name)))

(defun find-xml-rpc-method (method-name)
  "Looks for a method with the given name in *xml-rpc-package*,
  except that colons in the name get converted to hyphens."
  (let ((sym (find-symbol method-name *xml-rpc-package*)))
    (if (fboundp sym) sym nil)))

(defun execute-xml-rpc-call (method-name &rest arguments)
  "Execute method METHOD-NAME on ARGUMENTS, or raise an error if
  no such method exists in *XML-RPC-PACKAGE*"
  (let ((method (find-xml-rpc-method method-name)))
    (if method 
	(apply method arguments)
        ;; http://xmlrpc-epi.sourceforge.net/specs/rfc.fault_codes.php
        ;; -32601 ---> server error. requested method not found
        (error 'xml-rpc-fault :code -32601
               :string (format nil "Method ~A not found." method-name)))))

(defun handle-xml-rpc-call (in id)
  "Handle an actual call, reading XML from in and returning the
  XML-encoded result."
  ;; Try to conform to
  ;; http://xmlrpc-epi.sourceforge.net/specs/rfc.fault_codes.php
  (handler-bind ((s-xml:xml-parser-error
                  #'(lambda (c)
                      (format-debug (or *xml-rpc-debug-stream* t)
                                    "~a request parsing failed with ~a~%"
                                    id c)
                      (return-from handle-xml-rpc-call
                        ;; -32700 ---> parse error. not well formed
                        (encode-xml-rpc-fault (format nil "~a" c) -32700))))
                 (xml-rpc-fault
                  #'(lambda (c)
                      (format-debug (or *xml-rpc-debug-stream* t)
                                    "~a call failed with ~a~%" id c)
                      (return-from handle-xml-rpc-call
                        (encode-xml-rpc-fault (xml-rpc-fault-string c)
                                              (xml-rpc-fault-code c)))))
                 (error
                  #'(lambda (c)
                      (format-debug (or *xml-rpc-debug-stream* t)
                                    "~a call failed with ~a~%" id c)
                      (return-from handle-xml-rpc-call
                        ;; -32603 ---> server error. internal xml-rpc error
                        (encode-xml-rpc-fault (format nil "~a" c) -32603)))))
    (let ((call (decode-xml-rpc (debug-stream in))))
      (format-debug (or *xml-rpc-debug-stream* t) "~a received call ~s~%" id call)
      (let ((result (apply *xml-rpc-call-hook*
                           (first call)
			   (rest call))))
	(format-debug (or *xml-rpc-debug-stream* t) "~a call result is ~s~%" id result)
	(encode-xml-rpc-result result)))))

(defun xml-rpc-implementation-version ()
  "Identify ourselves"
  (concatenate 'string
	       "$Id: xml-rpc.lisp,v 1.4 2004/06/17 19:43:11 rschlatte Exp $"
	       " "
	       (lisp-implementation-type)
	       " "
	       (lisp-implementation-version)))

(defun xml-rpc-server-connection-handler (connection id agent url)
  "Handle an incoming connection, doing both all HTTP and XML-RPC stuff"
  (handler-bind ((error #'(lambda (c)
			    (format-debug (or *xml-rpc-debug-stream* t)
                                          "xml-rpc server connection handler failed with ~a~%" c)
                            (error c)
			    (return-from xml-rpc-server-connection-handler nil))))
    (let* ((header (read-line connection nil nil))
	   ;; (header-string header)
	   )
      (when (null header) (error "no request from client"))
      (setf header (tokens header))
      (unless (and (>= (length header) 3)
		   (string-equal (first header) "POST")
		   (string-equal (second header) url))
	;; (warn "Header ~a looks malformed, but proceeding - temporary fix BMM" header-string)
	#|(progn
	(format-header connection
	`(("HTTP/1.0 400 Bad Request")
	("Server: ~a" ,agent)
	("Connection: close")))
	(format-debug (or *xml-rpc-debug-stream* t) "~d got a bad request ~a~%" id header))|#
	)

      ;; Originally this was only done if the header looked ok
      (progn
	(do ((line (read-line connection nil nil)
		   (read-line connection nil nil)))
	    ((or (null line) (= 1 (length line))))
	  (format-debug (or *xml-rpc-debug-stream* t) "~d ~a~%" id line))
	(let ((xml (handle-xml-rpc-call connection id)))
	  (format-header connection
			 `(("HTTP/1.0 200 OK")
			   ("Server: ~a" ,agent)
			   ("Connection: close")
			   ("Content-Type: text/xml")
			   ("Content-Length: ~d" ,(length xml))))
	  (princ xml connection)
	  (format-debug (or *xml-rpc-debug-stream* t) "~d sending ~a~%" id xml))))
    (force-output connection)
    (close connection)))

(defparameter *counter* 0 "Unique ID for incoming connections")

(defun start-xml-rpc-server (&key (port *xml-rpc-port*) (url *xml-rpc-url*) (agent *xml-rpc-agent*))
  "Start an XML-RPC server in a separate process"
  (start-standard-server
   :name (format nil "xml-rpc server ~a:~d" url port)
   :port port
   :connection-handler #'(lambda (client-stream)
                           (let ((id (incf *counter*)))
                             (format-debug (or *xml-rpc-debug-stream* t) "spawned connection handler ~d~%" id)
                             (run-process (format nil "xml-rpc-server-connection-handler-~d" id)
                                          #'xml-rpc-server-connection-handler
                                          client-stream
                                          id
                                          agent
                                          url)))))

;;;; eof
