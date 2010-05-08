;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: all-tests.lisp,v 1.2 2004/06/17 19:43:11 rschlatte Exp $
;;;;
;;;; Load and execute all unit and functional tests
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(load (merge-pathnames "test-base64" *load-pathname*) :verbose t)
(load (merge-pathnames "test-xml-rpc" *load-pathname*) :verbose t)
(load (merge-pathnames "test-extensions" *load-pathname*) :verbose t)

;;;; eof
