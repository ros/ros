;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: test-xml.lisp,v 1.3 2005/11/06 12:44:48 scaekenberghe Exp $
;;;;
;;;; Unit and functional tests for xml.lisp
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

(assert
 (whitespace-char-p (character " ")))

(assert
 (whitespace-char-p (character "	")))

(assert
 (whitespace-char-p (code-char 10)))

(assert
 (whitespace-char-p (code-char 13)))

(assert
 (not (whitespace-char-p #\A)))

(assert
 (char= (with-input-from-string (stream "  ABC")
	  (skip-whitespace stream))
	#\A))

(assert
 (char= (with-input-from-string (stream "ABC")
	  (skip-whitespace stream))
	#\A))

(assert
 (string-equal (with-output-to-string (stream) (print-string-xml "<foo>" stream))
	       "&lt;foo&gt;"))

(assert
 (string-equal (with-output-to-string (stream) (print-string-xml "' '" stream))
               "' '"))

(assert
 (let ((string (map 'string #'identity '(#\return #\tab #\newline))))
   (string-equal (with-output-to-string (stream) (print-string-xml string stream))
                 string)))

(defun simple-echo-xml (in out)
  (start-parse-xml
   in
   (make-instance 'xml-parser-state
		  :new-element-hook #'(lambda (name attributes seed)
					(declare (ignore seed))
					(format out "<~a~:{ ~a='~a'~}>"
						name
						(mapcar #'(lambda (p) (list (car p) (cdr p)))
							(reverse attributes))))
		  :finish-element-hook #'(lambda (name attributes parent-seed seed)
					   (declare (ignore attributes parent-seed seed))
					   (format out "</~a>" name))
		  :text-hook #'(lambda (string seed)
				 (declare (ignore seed))
				 (princ string out)))))

(defun simple-echo-xml-string (string)
  (with-input-from-string (in string)
      (with-output-to-string (out)
	(simple-echo-xml in out))))

(assert
 (let ((xml "<FOO ATT1='1' ATT2='2'><B>Text</B><EMPTY></EMPTY>More text!<SUB><SUB></SUB></SUB></FOO>"))
   (equal (simple-echo-xml-string xml)
          xml)))

(assert 
  (let ((xml "<p> </p>"))
    (equal (simple-echo-xml-string xml)
           xml)))

;;;; eof