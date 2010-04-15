;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: test-sxml-dom.lisp,v 1.1.1.1 2004/06/07 18:49:59 scaekenberghe Exp $
;;;;
;;;; Unit and functional tests for sxml-dom.lisp
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

(assert
 (equal (with-input-from-string (stream " <foo/>")
	  (parse-xml stream :output-type :sxml))
	'(:|foo|)))

(assert
 (equal (parse-xml-string "<tag1><tag2 att1='one'/>this is some text</tag1>"
			  :output-type :sxml)
	'(:|tag1|
	   (:|tag2| (:@ (:|att1| "one")))
	   "this is some text")))

(assert
 (equal (parse-xml-string "<TAG>&lt;foo&gt;</TAG>"
			  :output-type :sxml)
	'(:TAG "<foo>")))

(assert
 (equal (parse-xml-string
	 "<P><INDEX ITEM='one'/> This is some <B>bold</B> text, with a leading &amp; trailing space </P>"
	 :output-type :sxml)
	'(:p
	  (:index (:@ (:item "one")))
	  " This is some "
	  (:b "bold")
	  " text, with a leading & trailing space ")))

(assert
 (consp (parse-xml-file (merge-pathnames "xhtml-page.xml" *load-pathname*)
			:output-type :sxml)))

(assert
 (consp (parse-xml-file (merge-pathnames "ant-build-file.xml" *load-pathname*)
			:output-type :sxml)))

(assert
 (consp (parse-xml-file (merge-pathnames "plist.xml" *load-pathname*)
			:output-type :sxml)))

(assert
 (string-equal (print-xml-string '(:|foo|) :input-type :sxml)
	       "<foo/>"))

(assert
 (string-equal (print-xml-string '(:|foo| (:@ (:|bar| "1"))) :input-type :sxml)
	       "<foo bar=\"1\"/>"))

(assert
 (string-equal (print-xml-string '(:foo "some text") :input-type :sxml)
	       "<FOO>some text</FOO>"))

(assert
 (string-equal (print-xml-string '(:|foo| (:|bar|)) :input-type :sxml)
	       "<foo><bar/></foo>"))

;;;; eof