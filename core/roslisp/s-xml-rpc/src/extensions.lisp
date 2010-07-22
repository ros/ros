;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: extensions.lisp,v 1.1 2004/06/17 19:43:11 rschlatte Exp $
;;;;
;;;; Extensions for xml-rpc:
;;;;
;;;; Server introspection:
;;;; http://xmlrpc.usefulinc.com/doc/reserved.html
;;;;
;;;; Multicall:
;;;; http://www.xmlrpc.com/discuss/msgReader$1208
;;;;
;;;; Capabilities:
;;;; http://groups.yahoo.com/group/xml-rpc/message/2897
;;;; 
;;;;
;;;; Copyright (C) 2004 Rudi Schlatte
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml-rpc)

;;; Introspection

(defun |system.listMethods| ()
  "List the methods that are available on this server."
  (let ((result nil))
    (do-symbols (sym *xml-rpc-package* (sort result #'string-lessp))
      (when (and (fboundp sym) (valid-xml-rpc-method-name-p (symbol-name sym)))
        (push (symbol-name sym) result)))))

(defun |system.methodSignature| (method-name)
  "Dummy system.methodSignature implementation.  There's no way
  to get (and no concept of) required argument types in Lisp, so
  this function always returns nil or errors."
  (let ((method (find-xml-rpc-method method-name)))
    (if method
        ;; http://xmlrpc.usefulinc.com/doc/sysmethodsig.html says to
        ;; return a non-array if the signature is not available
        "n/a"
        (error "Method ~A not found." method-name))))

(defun |system.methodHelp| (method-name)
  "Returns the function documentation for the given method."
  (let ((method (find-xml-rpc-method method-name)))
    (if method
        (or (documentation method 'function) "")
        (error "Method ~A not found." method-name))))

;;; system.multicall

(defun do-one-multicall (call-struct)
  (let ((name (get-xml-rpc-struct-member call-struct :|methodName|))
        (params (get-xml-rpc-struct-member call-struct :|params|)))
    (handler-bind
        ((xml-rpc-fault
          #'(lambda (c)
              (format-debug (or *xml-rpc-debug-stream* t)
                            "Call to ~A in system.multicall failed with ~a~%"
                            name c)
              (return-from do-one-multicall
                (xml-literal
                 (encode-xml-rpc-fault-value (xml-rpc-fault-string c)
                                             (xml-rpc-fault-code c))))))
         (error
          #'(lambda (c)
              (format-debug
               (or *xml-rpc-debug-stream* t)
               "Call to ~A in system.multicall failed with ~a~%" name c)
              (return-from do-one-multicall
                (xml-literal
                 (encode-xml-rpc-fault-value
                  ;; -32603 ---> server error. internal xml-rpc error
                  (format nil "~a" c) -32603))))))
      (format-debug (or *xml-rpc-debug-stream* t)
                    "system.multicall calling ~a with ~s~%" name params)
      (let ((result (apply *xml-rpc-call-hook* name params)))
        (list result)))))

(defun |system.multicall| (calls)
  "Implement system.multicall; see http://www.xmlrpc.com/discuss/msgReader$1208
  for the specification."
  (mapcar #'do-one-multicall calls))

;;; system.getCapabilities

(defun |system.getCapabilities| ()
  "Get a list of supported capabilities; see
  http://groups.yahoo.com/group/xml-rpc/message/2897 for the
  specification."
  (let ((capabilities
         '("xmlrpc" ("specUrl" "http://www.xmlrpc.com/spec"
                     "specVersion" 1)
           "introspect" ("specUrl" "http://xmlrpc.usefulinc.com/doc/reserved.html"
                         "specVersion" 1)
           "multicall" ("specUrl" "http://www.xmlrpc.com/discuss/msgReader$1208"
                        "specVersion" 1)
           "faults_interop" ("specUrl" "http://xmlrpc-epi.sourceforge.net/specs/rfc.fault_codes.php"
                             "specVersion" 20010516))))
    (apply #'xml-rpc-struct
           (loop for (name description) on capabilities by #'cddr
                collecting name
                collecting (apply #'xml-rpc-struct description)))))

;;;; eof
