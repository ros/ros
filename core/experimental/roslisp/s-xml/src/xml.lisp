;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: xml.lisp,v 1.14 2005/11/20 14:24:34 scaekenberghe Exp $
;;;;
;;;; This is a Common Lisp implementation of a basic but usable XML parser.
;;;; The parser is non-validating and not complete (no CDATA).
;;;; Namespace and entities are handled.
;;;; The API into the parser is a pure functional parser hook model that comes from SSAX,
;;;; see also http://pobox.com/~oleg/ftp/Scheme/xml.html or http://ssax.sourceforge.net
;;;; Different DOM models are provided, an XSML, an LXML and a xml-element struct based one.
;;;;
;;;; Copyright (C) 2002, 2003, 2004, 2005 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

;;; error reporting

(define-condition xml-parser-error (error)
  ((message :initarg :message :reader xml-parser-error-message)
   (args :initarg :args :reader xml-parser-error-args)
   (stream :initarg :stream :reader xml-parser-error-stream :initform nil))
  (:report (lambda (condition stream)
	     (format stream
		     "XML parser ~?~@[ near stream position ~d~]."
		     (xml-parser-error-message condition)
		     (xml-parser-error-args condition)
		     (and (xml-parser-error-stream condition)
			  (file-position (xml-parser-error-stream condition))))))
  (:documentation "Thrown by the XML parser to indicate errorneous input"))

(setf (documentation 'xml-parser-error-message 'function)
      "Get the message from an XML parser error"
      (documentation 'xml-parser-error-args 'function)
      "Get the error arguments from an XML parser error"
      (documentation 'xml-parser-error-stream 'function)
      "Get the stream from an XML parser error")

(defun parser-error (message &optional args stream)
  (make-condition 'xml-parser-error
		  :message message
		  :args args
		  :stream stream))
   
;;; utilities

(defun whitespace-char-p (char)
  "Is char an XML whitespace character ?"
  (or (char= char #\space)
      (char= char #\tab)
      (char= char #\return)
      (char= char #\linefeed)))

(defun identifier-char-p (char)
  "Is char an XML identifier character ?"
  (or (and (char<= #\A char) (char<= char #\Z))
      (and (char<= #\a char) (char<= char #\z))
      (and (char<= #\0 char) (char<= char #\9))
      (char= char #\-)
      (char= char #\_)
      (char= char #\.)
      (char= char #\:)))

(defun skip-whitespace (stream)
  "Skip over XML whitespace in stream, return first non-whitespace
  character which was peeked but not read, return nil on eof"
  (loop
   (let ((char (peek-char nil stream nil nil)))
     (if (and char (whitespace-char-p char))
	 (read-char stream)
       (return char)))))

(defun make-extendable-string (&optional (size 10))
  "Make an extendable string which is a one-dimensional character
  array which is adjustable and has a fill pointer"
  (make-array size
	      :element-type 'character
	      :adjustable t
	      :fill-pointer 0))

(defun print-string-xml (string stream &key (start 0) end)
  "Write the characters of string to stream using basic XML conventions"
  (loop for offset upfrom start below (or end (length string))
        for char = (char string offset)
	do (case char
	     (#\& (write-string "&amp;" stream))
	     (#\< (write-string "&lt;" stream))
	     (#\> (write-string "&gt;" stream))
	     (#\" (write-string "&quot;" stream))
             ((#\newline #\return #\tab) (write-char char stream))
	     (t (if (and (<= 32 (char-code char))
			 (<= (char-code char) 126))
		    (write-char char stream)
		  (progn
		    (write-string "&#x" stream)
		    (write (char-code char) :stream stream :base 16)
		    (write-char #\; stream)))))))

(defun make-standard-entities ()
  "A hashtable mapping XML entity names to their replacement strings,
  filled with the standard set"
  (let ((entities (make-hash-table :test #'equal)))
    (setf (gethash "amp" entities) (string #\&)
	  (gethash "quot" entities) (string #\")
	  (gethash "apos" entities) (string #\')
	  (gethash "lt" entities) (string #\<)
	  (gethash "gt" entities) (string #\>)
	  (gethash "nbsp" entities) (string #\space))
    entities))

(defun resolve-entity (stream extendable-string entities &optional (entity (make-extendable-string)))
  "Read and resolve an XML entity from stream, positioned after the '&' entity marker, 
  accepting &name; &#DEC; and &#xHEX; formats,
  destructively modifying string, which is also returned,
  destructively modifying entity, incorrect entity formats result in errors"
  (loop
   (let ((char (read-char stream nil nil)))
     (cond ((null char) (error (parser-error "encountered eof before end of entity")))
	   ((char= #\; char) (return))
	   (t (vector-push-extend char entity)))))
  (if (char= (char entity 0) #\#)
      (let ((code (if (char= (char entity 1) #\x)
		      (parse-integer entity :start 2 :radix 16 :junk-allowed t)
		    (parse-integer entity :start 1 :radix 10 :junk-allowed t))))
	(when (null code) 
          (error (parser-error "encountered incorrect entity &~s;" (list entity) stream)))
	(vector-push-extend (code-char code) extendable-string))
    (let ((value (gethash entity entities)))
      (if value
	  (loop :for char :across value 
                :do (vector-push-extend char extendable-string))
	(error (parser-error "encountered unknown entity &~s;" (list entity) stream)))))
  extendable-string)

;;; namespace support

(defvar *ignore-namespaces* nil
  "When t, namespaces are ignored like in the old version of S-XML")

(defclass xml-namespace ()
  ((uri :documentation "The URI used to identify this namespace"
        :accessor get-uri
        :initarg :uri)
   (prefix :documentation "The preferred prefix assigned to this namespace"
           :accessor get-prefix
           :initarg :prefix
           :initform nil)
   (package :documentation "The Common Lisp package where this namespace's symbols are interned"
            :accessor get-package
            :initarg :package
            :initform nil))
  (:documentation "Describes an XML namespace and how it is handled"))

(defmethod print-object ((object xml-namespace) stream)
  (print-unreadable-object (object stream :type t :identity t)
    (format stream "~A - ~A" (get-prefix object) (get-uri object))))

(defvar *local-namespace* (make-instance 'xml-namespace 
                                         :uri "local"
                                         :prefix "" 
                                         :package (find-package :keyword))
  "The local (global default) XML namespace")

(defvar *xml-namespace* (make-instance 'xml-namespace 
                                       :uri "http://www.w3.org/XML/1998/namespace" 
                                       :prefix "xml"
                                       :package (or (find-package :xml)
                                                    (make-package :xml :nicknames '("XML"))))
  "REC-xml-names-19990114 says the prefix xml is bound to the namespace http://www.w3.org/XML/1998/namespace.")

(defvar *known-namespaces* (list *local-namespace* *xml-namespace*)
  "The list of known/defined namespaces")

(defvar *namespaces* `(("xml" . ,*xml-namespace*) ("" . ,*local-namespace*))
  "Ordered list of (prefix . XML-namespace) bindings currently in effect - special variable")

(defun find-namespace (uri)
  "Find a registered XML namespace identified by uri"
  (find uri *known-namespaces* :key #'get-uri :test #'string-equal))

(defun register-namespace (uri prefix package)
  "Register a new or redefine an existing XML namespace defined by uri with prefix and package"
  (let ((namespace (find-namespace uri)))
    (if namespace
        (setf (get-prefix namespace) prefix
              (get-package namespace) (find-package package))
      (push (setf namespace (make-instance 'xml-namespace
                                           :uri uri
                                           :prefix prefix
                                           :package (find-package package)))
            *known-namespaces*))
    namespace))

(defun find-namespace-binding (prefix namespaces)
  "Find the XML namespace currently bound to prefix in the namespaces bindings"
  (cdr (assoc prefix namespaces :test #'string-equal)))

(defun split-identifier (identifier)
  "Split an identifier 'prefix:name' and return (values prefix name)"
  (when (symbolp identifier)
    (setf identifier (symbol-name identifier)))
  (let ((colon-position (position #\: identifier :test #'char=)))
    (if colon-position
        (values (subseq identifier 0 colon-position)
                (subseq identifier (1+ colon-position)))
      (values nil identifier))))

(defvar *require-existing-symbols* nil
  "If t, each XML identifier must exist as symbol already")

(defvar *auto-export-symbols* t
  "If t, export newly interned symbols form their packages")

(defun resolve-identifier (identifier namespaces &optional as-attribute)
  "Resolve the string identifier in the list of namespace bindings"
  (if *ignore-namespaces*
      (intern identifier :keyword)
    (flet ((intern-symbol (string package) ; intern string as a symbol in package
             (if *require-existing-symbols*
                 (let ((symbol (find-symbol string package)))
                   (or symbol
                       (error "Symbol ~s does not exist in ~s" string package)))
               (let ((symbol (intern string package)))
                 (when (and *auto-export-symbols* 
                            (not (eql package (find-package :keyword))))
                   (export symbol package))
                 symbol))))
      (multiple-value-bind (prefix name)
          (split-identifier identifier)
        (if (or (null prefix) (string= prefix "xmlns"))
            (if as-attribute
                (intern (if (string= prefix "xmlns") identifier name) (get-package *local-namespace*))
              (let ((default-namespace (find-namespace-binding "" namespaces)))
                (intern-symbol name (get-package default-namespace))))
          (let ((namespace (find-namespace-binding prefix namespaces)))
            (if namespace
                (intern-symbol name (get-package namespace))
              (error "namespace not found for prefix ~s" prefix))))))))

(defvar *auto-create-namespace-packages* t
  "If t, new packages will be created for namespaces, if needed, named by the prefix")

(defun new-namespace (uri &optional prefix)
  "Register a new namespace for uri and prefix, creating a package if necessary"
  (if prefix
      (register-namespace uri
                          prefix
                          (or (find-package prefix)
                              (if *auto-create-namespace-packages*
                                  (make-package prefix :nicknames `(,(string-upcase prefix)))
                                (error "Cannot find or create package ~s" prefix))))
    (let ((unique-name (loop :for i :upfrom 0
                             :do (let ((name (format nil "ns-~d" i)))
                                   (when (not (find-package name))
                                     (return name))))))
      (register-namespace uri
                          unique-name
                          (if *auto-create-namespace-packages*
                              (make-package (string-upcase unique-name) :nicknames `(,unique-name))
                            (error "Cannot create package ~s" unique-name))))))

(defun extend-namespaces (attributes namespaces)
  "Given possible 'xmlns[:prefix]' attributes, extend the namespaces bindings"
  (unless *ignore-namespaces*
    (let (default-namespace-uri)
      (loop :for (key . value) :in attributes
            :do (if (string= key "xmlns")
                    (setf default-namespace-uri value)
                  (multiple-value-bind (prefix name)
                      (split-identifier key)
                    (when (string= prefix "xmlns")
                      (let* ((uri value)
                             (prefix name)
                             (namespace (find-namespace uri)))
                        (unless namespace
                          (setf namespace (new-namespace uri prefix)))
                        (push `(,prefix . ,namespace) namespaces))))))
      (when default-namespace-uri
        (let ((namespace (find-namespace default-namespace-uri)))
          (unless namespace
            (setf namespace (new-namespace default-namespace-uri)))
          (push `("" . ,namespace) namespaces)))))
  namespaces)

(defun print-identifier (identifier stream &optional as-attribute)
  "Print identifier on stream using namespace conventions"
  (declare (ignore as-attribute) (special *namespaces*))
  (if *ignore-namespaces*
      (princ identifier stream)
    (if (symbolp identifier)
        (let ((package (symbol-package identifier))
              (name (symbol-name identifier)))
          (let* ((namespace (find package *known-namespaces* :key #'get-package))
                 (prefix (or (car (find namespace *namespaces* :key #'cdr))
                             (get-prefix namespace))))
            (if (string= prefix "")
                (princ name stream)
              (format stream "~a:~a" prefix name))))
      (princ identifier stream))))

;;; the parser state

(defclass xml-parser-state ()
  ((entities :documentation "A hashtable mapping XML entity names to their replacement stings"
	     :accessor get-entities
	     :initarg :entities
	     :initform (make-standard-entities))
   (seed :documentation "The user seed object"
	 :accessor get-seed
	 :initarg :seed
	 :initform nil)
   (buffer :documentation "The main reusable character buffer"
	   :accessor get-buffer
	   :initform (make-extendable-string))
   (mini-buffer :documentation "The secondary, smaller reusable character buffer"
		:accessor get-mini-buffer
		:initform (make-extendable-string))
   (new-element-hook :documentation "Called when new element starts"
		     ;; Handle the start of a new xml element with name and attributes,
		     ;; receiving seed from previous element (sibling or parent)
		     ;; return seed to be used for first child (content) 
                     ;; or directly to finish-element-hook
		     :accessor get-new-element-hook
		     :initarg :new-element-hook
		     :initform #'(lambda (name attributes seed)
				   (declare (ignore name attributes))
                                   seed))
   (finish-element-hook :documentation "Called when element ends"
			;; Handle the end of an xml element with name and attributes,
			;; receiving parent-seed, the seed passed to us when this element started,
                        ;; i.e. passed to our corresponding new-element-hook
			;; and receiving seed from last child (content) 
                        ;; or directly from new-element-hook
			;; return final seed for this element to next element (sibling or parent)
			:accessor get-finish-element-hook
			:initarg :finish-element-hook
			:initform #'(lambda (name attributes parent-seed seed)
				      (declare (ignore name attributes parent-seed))
                                      seed))
   (text-hook :documentation "Called when text is found"
	      ;; Handle text in string, found as contents,
	      ;; receiving seed from previous element (sibling or parent), 
              ;; return final seed for this element to next element (sibling or parent)
	      :accessor get-text-hook
	      :initarg :text-hook
	      :initform #'(lambda (string seed)
			    (declare (ignore string))
                            seed)))
  (:documentation "The XML parser state passed along all code making up the parser"))

(setf (documentation 'get-seed 'function)
      "Get the initial user seed of an XML parser state"
      (documentation 'get-entities 'function)
      "Get the entities hashtable of an XML parser state"
      (documentation 'get-new-element-hook 'function)
      "Get the new element hook of an XML parser state"
      (documentation 'get-finish-element-hook 'function)
      "Get the finish element hook of an XML parser state"
      (documentation 'get-text-hook 'function)
      "Get the text hook of an XML parser state")

#-allegro
(setf (documentation '(setf get-seed) 'function)
      "Set the initial user seed of an XML parser state"
      (documentation '(setf get-entities) 'function)
      "Set the entities hashtable of an XML parser state"
      (documentation '(setf get-new-element-hook) 'function)
      "Set the new element hook of an XML parser state"
      (documentation '(setf get-finish-element-hook) 'function)
      "Set the finish element hook of an XML parser state"
      (documentation '(setf get-text-hook) 'function)
      "Set the text hook of an XML parser state")

(defmethod get-mini-buffer :after ((state xml-parser-state))
  "Reset and return the reusable mini buffer"
  (with-slots (mini-buffer) state
    (setf (fill-pointer mini-buffer) 0)))

(defmethod get-buffer :after ((state xml-parser-state))
  "Reset and return the main reusable buffer"
  (with-slots (buffer) state
    (setf (fill-pointer buffer) 0)))
  
;;; parser support

(defun parse-whitespace (stream extendable-string)
  "Read and collect XML whitespace from stream in string which is
  destructively modified, return first non-whitespace character which
  was peeked but not read, return nil on eof"
  (loop
   (let ((char (peek-char nil stream nil nil)))
     (if (and char (whitespace-char-p char))
	 (vector-push-extend (read-char stream) extendable-string)
       (return char)))))

(defun parse-string (stream state &optional (string (make-extendable-string)))
  "Read and return an XML string from stream, delimited by either
  single or double quotes, the stream is expected to be on the opening
  delimiter, at the end the closing delimiter is also read, entities
  are resolved, eof before end of string is an error"
  (let ((delimiter (read-char stream nil nil))
	(char))
    (when (or (null delimiter) (not (or (char= delimiter #\') (char= delimiter #\"))))
      (error (parser-error "expected string delimiter" nil stream)))
    (loop
     (setf char (read-char stream nil nil))
     (cond ((null char) (error (parser-error "encountered eof before end of string")))
	   ((char= char delimiter) (return))
	   ((char= char #\&) (resolve-entity stream string (get-entities state) (get-mini-buffer state)))
	   (t (vector-push-extend char string))))
    string))

(defun parse-text (stream state extendable-string)
  "Read and collect XML text from stream in string which is
  destructively modified, the text ends with a '<', which is peeked and
  returned, entities are resolved, eof is considered an error"
  (let (char)
    (loop
     (setf char (peek-char nil stream nil nil))
     (when (null char) (error (parser-error "encountered unexpected eof in text")))
     (when (char= char #\<) (return))
     (read-char stream)
     (if (char= char #\&)
	 (resolve-entity stream extendable-string (get-entities state) (get-mini-buffer state))
       (vector-push-extend char extendable-string)))
    char))

(defun parse-identifier (stream &optional (identifier (make-extendable-string)))
  "Read and returns an XML identifier from stream, positioned at the
  start of the identifier, ending with the first non-identifier
  character, which is peeked, the identifier is written destructively
  into identifier which is also returned"
  (loop
   (let ((char (peek-char nil stream nil nil)))
     (cond ((and char (identifier-char-p char))
	    (read-char stream)
	    (vector-push-extend char identifier))
	   (t
	    (return identifier))))))
	 
(defun skip-comment (stream)
  "Skip an XML comment in stream, positioned after the opening '<!--',
  consumes the closing '-->' sequence, unexpected eof or a malformed
  closing sequence result in a error"
  (let ((dashes-to-read 2))
    (loop
     (if (zerop dashes-to-read) (return))
     (let ((char (read-char stream nil nil)))
       (if (null char)
	   (error (parser-error "encountered unexpected eof for comment")))
       (if (char= char #\-)
	   (decf dashes-to-read)
	 (setf dashes-to-read 2)))))
  (if (char/= (read-char stream nil nil) #\>)
      (error (parser-error "expected > ending comment" nil stream))))

(defun read-cdata (stream state &optional (string (make-extendable-string)))
  "Reads in the CDATA and calls the callback for CDATA if it exists"
  ;; we already read the <![CDATA[ stuff
  ;; continue to read until we hit ]]>
  (let ((char #\space)
	(last-3-characters (list #\[ #\A #\T))
	(pattern (list #\> #\] #\])))
    (loop
     (setf char (read-char stream nil nil))
     (when (null char) (error (parser-error "encountered unexpected eof in text")))
     (push char last-3-characters)
     (setf (cdddr last-3-characters) nil)
     (cond
       ((equal last-3-characters
	       pattern)
	(setf (fill-pointer string)
	      (- (fill-pointer string) 2))
	(setf (get-seed state)
	      (funcall (get-text-hook state)
		       (copy-seq string)
		       (get-seed state)))
	(return-from read-cdata))
       (t
	(vector-push-extend char string))))))

(defun skip-special-tag (stream state)
  "Skip an XML special tag (comments and processing instructions) in
  stream, positioned after the opening '<', unexpected eof is an error"
  ;; opening < has been read, consume ? or !
  (read-char stream)
  (let ((char (read-char stream nil nil)))
    ;; see if we are dealing with a comment
    (when (char= char #\-)
      (setf char (read-char stream nil nil))
      (when (char= char #\-)
	(skip-comment stream)
	(return-from skip-special-tag)))
    ;; maybe we are dealing with CDATA?
    (when (and (char= char #\[)
	       (loop :for pattern :across "CDATA["
		     :for char = (read-char stream nil nil)
		     :when (null char) :do
		     (error (parser-error "encountered unexpected eof in cdata"))
		     :always (char= char pattern)))
      (read-cdata stream state (get-buffer state))
      (return-from skip-special-tag))
    ;; loop over chars, dealing with strings (skipping their content)
    ;; and counting opening and closing < and > chars
    (let ((taglevel 1)
	  (string-delimiter))
      (loop
       (when (zerop taglevel) (return))
       (setf char (read-char stream nil nil))
       (when (null char)
	 (error (parser-error "encountered unexpected eof for special (! or ?) tag" nil stream)))
       (if string-delimiter
	   ;; inside a string we only look for a closing string delimiter
	   (when (char= char string-delimiter)
	     (setf string-delimiter nil))
	 ;; outside a string we count < and > and watch out for strings
	 (cond ((or (char= char #\') (char= char #\")) (setf string-delimiter char))
	       ((char= char #\<) (incf taglevel))
	       ((char= char #\>) (decf taglevel))))))))
	
;;; the XML parser proper

(defun parse-xml-element-attributes (stream state)
  "Parse XML element attributes from stream positioned after the tag
  identifier, returning the attributes as an assoc list, ending at
  either a '>' or a '/' which is peeked and also returned"
  (declare (special *namespaces*))
  (let (char attributes)
    (loop
     ;; skip whitespace separating items
     (setf char (skip-whitespace stream))
     ;; start tag attributes ends with > or />
     (when (and char (or (char= char #\>) (char= char #\/))) (return))
     ;; read the attribute key
     (let ((key (copy-seq (parse-identifier stream (get-mini-buffer state)))))
       ;; skip separating whitespace
       (setf char (skip-whitespace stream))
       ;; require = sign (and consume it if present)
       (if (and char (char= char #\=))
	   (read-char stream)
	 (error (parser-error "expected =" nil stream)))
       ;; skip separating whitespace
       (skip-whitespace stream)
       ;; read the attribute value as a string
       (push (cons key (copy-seq (parse-string stream state (get-buffer state))))
	     attributes)))
    ;; return attributes peek char ending loop
    (values attributes char)))

(defun parse-xml-element (stream state)
  "Parse and return an XML element from stream, positioned after the opening '<'"
  (declare (special *namespaces*))
  ;; opening < has been read
  (when (char= (peek-char nil stream nil nil) #\!)
    (skip-special-tag stream state)
    (return-from parse-xml-element))
  (let (char buffer open-tag parent-seed has-children)
    (setf parent-seed (get-seed state))
    ;; read tag name (no whitespace between < and name ?)
    (setf open-tag (copy-seq (parse-identifier stream (get-mini-buffer state))))
    ;; tag has been read, read attributes if any
    (multiple-value-bind (attributes peeked-char)
	(parse-xml-element-attributes stream state)
      (let ((*namespaces* (extend-namespaces attributes *namespaces*)))
        (setf open-tag (resolve-identifier open-tag *namespaces*)
              attributes (loop :for (key . value) :in attributes
                               :collect (cons (resolve-identifier key *namespaces* t) value)))
        (setf (get-seed state) (funcall (get-new-element-hook state)
                                        open-tag attributes (get-seed state)))
        (setf char peeked-char)
        (when (char= char #\/)
          ;; handle solitary tag of the form <tag .. />
          (read-char stream)
          (setf char (read-char stream nil nil))
          (if (char= #\> char)
              (progn
                (setf (get-seed state) (funcall (get-finish-element-hook state)
                                                open-tag attributes parent-seed (get-seed state)))
                (return-from parse-xml-element))
            (error (parser-error "expected >" nil stream))))
        ;; consume >
        (read-char stream)
        (loop
         (setf buffer (get-buffer state))
         ;; read whitespace into buffer
         (setf char (parse-whitespace stream buffer))
         ;; see what ended the whitespace scan
         (cond ((null char) (error (parser-error "encountered unexpected eof handling ~a" (list open-tag))))
               ((char= char #\<)
                ;; consume the <
                (read-char stream)
                (if (char= (peek-char nil stream nil nil) #\/)
                    (progn
                      ;; handle the matching closing tag </tag> and done
                      ;; if we read whitespace as this (leaf) element's contents, it is significant
                      (when (and (not has-children) (plusp (length buffer)))
                        (setf (get-seed state) (funcall (get-text-hook state)
                                                        (copy-seq buffer) (get-seed state))))
                      (read-char stream)
                      (let ((close-tag (resolve-identifier (parse-identifier stream (get-mini-buffer state)) 
                                                           *namespaces*)))
                        (unless (eq open-tag close-tag)
                          (error (parser-error "found <~a> not matched by </~a> but by <~a>"
                                               (list open-tag open-tag close-tag) stream)))
                        (unless (char= (read-char stream nil nil) #\>)
                          (error (parser-error "expected >" nil stream)))
                        (setf (get-seed state) (funcall (get-finish-element-hook state)
                                                        open-tag attributes parent-seed (get-seed state))))
                      (return))
                  ;; handle child tag and loop, no hooks to call here
                  ;; whitespace between child elements is skipped
                  (progn
                    (setf has-children t)
                    (parse-xml-element stream state))))
               (t
                ;; no child tag, concatenate text to whitespace in buffer
                ;; handle text content and loop
                (setf char (parse-text stream state buffer))
                (setf (get-seed state) (funcall (get-text-hook state) 
                                                (copy-seq buffer) (get-seed state))))))))))

(defun start-parse-xml (stream &optional (state (make-instance 'xml-parser-state)))
  "Parse and return a toplevel XML element from stream, using parser state"
  (loop
   (let ((char (skip-whitespace stream)))
     (when (null char) (return-from start-parse-xml))
     ;; skip whitespace until start tag
     (unless (char= char #\<)
       (error (parser-error "expected <" nil stream)))
     (read-char stream)			; consume peeked char
     (setf char (peek-char nil stream nil nil))
     (if (or (char= char #\!) (char= char #\?))
	 ;; deal with special tags
	 (skip-special-tag stream state)
       (progn
	 ;; read the main element
	 (parse-xml-element stream state)
	 (return-from start-parse-xml (get-seed state)))))))

;;;; eof
