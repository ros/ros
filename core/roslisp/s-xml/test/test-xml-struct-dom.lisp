;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: test-xml-struct-dom.lisp,v 1.2 2005/08/29 15:01:49 scaekenberghe Exp $
;;;;
;;;; Unit and functional tests for xml-struct-dom.lisp
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml)

(assert
 (xml-equal (with-input-from-string (stream " <foo/>")
	      (parse-xml stream :output-type :xml-struct))
	    (make-xml-element :name :|foo|)))

(assert
 (xml-equal (parse-xml-string "<tag1><tag2 att1='one'/>this is some text</tag1>"
			      :output-type :xml-struct)
	    (make-xml-element :name :|tag1|
			      :children (list (make-xml-element :name :|tag2|
								:attributes '((:|att1| . "one")))
					      "this is some text"))))

(assert
 (xml-equal (parse-xml-string "<tag>&lt;foo&gt;</tag>"
			      :output-type :xml-struct)
	    (make-xml-element :name :|tag|
			      :children (list "<foo>"))))

(assert
 (xml-equal (parse-xml-string
	     "<P><INDEX ITEM='one'/> This is some <B>bold</B> text, with a leading &amp; trailing space </P>"
	     :output-type :xml-struct)
	    (make-xml-element :name :p
			      :children (list (make-xml-element :name :index
								:attributes '((:item . "one")))
					      " This is some "
					      (make-xml-element :name :b
								:children (list "bold"))
					      " text, with a leading & trailing space "))))

(assert
 (xml-element-p (parse-xml-file (merge-pathnames "xhtml-page.xml" *load-pathname*)
				:output-type :xml-struct)))

(assert
 (xml-element-p (parse-xml-file (merge-pathnames "ant-build-file.xml" *load-pathname*)
				:output-type :xml-struct)))

(assert
 (xml-element-p (parse-xml-file (merge-pathnames "plist.xml" *load-pathname*)
				:output-type :xml-struct)))

(assert
 (string-equal (print-xml-string (make-xml-element :name "foo")
				 :input-type :xml-struct)
	       "<foo/>"))

(assert
 (string-equal (print-xml-string (make-xml-element :name "foo" :attributes '((:|bar| . "1")))
				 :input-type :xml-struct)
	       "<foo bar=\"1\"/>"))

(assert
 (string-equal (print-xml-string (make-xml-element :name "foo" :children (list "some text"))
				 :input-type :xml-struct)
	       "<foo>some text</foo>"))

(assert
 (string-equal (print-xml-string (make-xml-element :name "foo" :children (list (make-xml-element :name "bar")))
				 :input-type :xml-struct)
	       "<foo><bar/></foo>"))

;;;; eof