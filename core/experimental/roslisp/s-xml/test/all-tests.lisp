;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: all-tests.lisp,v 1.1.1.1 2004/06/07 18:49:58 scaekenberghe Exp $
;;;;
;;;; Load and execute all unit and functional tests
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(load (merge-pathnames "test-xml" *load-pathname*) :verbose t)
(load (merge-pathnames "test-xml-struct-dom" *load-pathname*) :verbose t)
(load (merge-pathnames "test-lxml-dom" *load-pathname*) :verbose t)
(load (merge-pathnames "test-sxml-dom" *load-pathname*) :verbose t)

;;;; eof