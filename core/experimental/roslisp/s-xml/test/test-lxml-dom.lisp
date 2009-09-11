;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: test-lxml-dom.lisp,v 1.2 2005/11/06 12:44:48 scaekenberghe Exp $
;;;;
;;;; Unit and functional tests for lxml-dom.lisp
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

(assert
 (equal (with-input-from-string (stream " <foo/>")
	  (parse-xml stream :output-type :lxml))
	:|foo|))

(assert
 (equal (parse-xml-string "<tag1><tag2 att1='one'/>this is some text</tag1>"
			  :output-type :lxml)
	'(:|tag1|
	   ((:|tag2| :|att1| "one"))
	   "this is some text")))

(assert
 (equal (parse-xml-string "<TAG>&lt;foo&gt;</TAG>"
			  :output-type :lxml)
	'(:TAG "<foo>")))

(assert
 (equal (parse-xml-string
	 "<P><INDEX ITEM='one'/> This is some <B>bold</B> text, with a leading &amp; trailing space </P>"
	 :output-type :lxml)
	'(:p
	  ((:index :item "one"))
	  " This is some "
	  (:b "bold")
	  " text, with a leading & trailing space ")))

(assert
 (consp (parse-xml-file (merge-pathnames "xhtml-page.xml" *load-pathname*)
			:output-type :lxml)))

(assert
 (consp (parse-xml-file (merge-pathnames "ant-build-file.xml" *load-pathname*)
			:output-type :lxml)))

(assert
 (consp (parse-xml-file (merge-pathnames "plist.xml" *load-pathname*)
			:output-type :lxml)))

(assert
 (string-equal (print-xml-string :|foo| :input-type :lxml)
	       "<foo/>"))

(assert
 (string-equal (print-xml-string '((:|foo| :|bar| "1")) :input-type :lxml)
	       "<foo bar=\"1\"/>"))

(assert
 (string-equal (print-xml-string '(:foo "some text") :input-type :lxml)
	       "<FOO>some text</FOO>"))

(assert
 (string-equal (print-xml-string '(:|foo| :|bar|) :input-type :lxml)
	       "<foo><bar/></foo>"))

(assert (string-equal (second
                       (with-input-from-string (stream "<foo><![CDATA[<greeting>Hello, world!</greeting>]]></foo>")
                         (parse-xml stream :output-type :lxml)))
                      "<greeting>Hello, world!</greeting>"))
	   
(assert (string-equal (second
                       (with-input-from-string (stream "<foo><![CDATA[<greeting>Hello, < world!</greeting>]]></foo>")
                         (parse-xml stream :output-type :lxml)))
                      "<greeting>Hello, < world!</greeting>"))

;;;; eof