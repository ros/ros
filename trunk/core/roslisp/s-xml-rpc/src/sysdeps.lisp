;;;; -*- mode: lisp -*-
;;;;
;;;; $Id: sysdeps.lisp,v 1.1.1.1 2004/06/09 09:02:39 scaekenberghe Exp $
;;;;
;;;; These are the system dependent part of S-XML-RPC.
;;;; Ports to OpenMCL, LispWorks and SBCL are provided.
;;;; Porting to another CL requires implementating these definitions.
;;;;
;;;; Copyright (C) 2002, 2004 Sven Van Caekenberghe, Beta Nine BVBA.
;;;; SBCL port Copyright (C) 2004, Brian Mastenbrook & Rudi Schlatte.
;;;;
;;;; You are granted the rights to distribute and use this software
;;;; as governed by the terms of the Lisp Lesser General Public License
;;;; (http://opensource.franz.com/preamble.html), also known as the LLGPL.

(in-package :s-xml-rpc)

(defmacro with-open-socket-stream ((var host port) &body body)
  "Execute body with a bidirectional socket stream opened to host:port"
  #+openmcl
  `(ccl:with-open-socket (,var :remote-host ,host :remote-port ,port)
    ,@body)
  #+lispworks
  `(with-open-stream (,var (comm:open-tcp-stream ,host ,port))
    ,@body)
  #+sbcl
  (let ((socket-object (gensym)))
    `(let ((,socket-object (make-instance 'sb-bsd-sockets:inet-socket
                                          :type :stream
                                          :protocol :tcp)))
      (sb-bsd-sockets:socket-connect ,socket-object
       (car (sb-bsd-sockets:host-ent-addresses
             (sb-bsd-sockets:get-host-by-name ,host))) ,port)
      (let ((,var (sb-bsd-sockets:socket-make-stream ,socket-object
                                                     :element-type 'character
                                                     :input t
                                                     :output t
                                                     :buffering :none)))
        (unwind-protect
             (progn ,@body)
          (close ,var))))))

(defun run-process (name function &rest arguments)
  "Create and run a new process with name, executing function on arguments"
  #+lispworks (apply #'mp:process-run-function name '(:priority 3) function arguments)
  #+openmcl (apply #'ccl:process-run-function name function arguments)
  #+sbcl (declare (ignore name))
  #+sbcl (apply function arguments))

(defvar *server-processes* nil)

(defun start-standard-server (&key port name connection-handler)
  "Start a server process with name, listening on port, delegating to connection-handler with stream as argument"
  #+lispworks (progn
                (comm:start-up-server
               :function #'(lambda (socket-handle)
                             (let ((client-stream (make-instance 'comm:socket-stream
                                                                 :socket socket-handle
                                                                 :direction :io
                                                                 :element-type 'base-char)))
                               (funcall connection-handler client-stream)))
               :service port
               :announce t
               :error t
               :wait t
               :process-name name)
                name)
  #+openmcl (progn
              (ccl:process-run-function
             name
             #'(lambda ()
                 (let ((server-socket (ccl:make-socket :connect :passive
                                                       :local-port port
                                                       :reuse-address t)))
                   (unwind-protect
                       (loop 
                        (let ((client-stream (ccl:accept-connection server-socket)))
                          (funcall connection-handler client-stream))) 
                     (close server-socket)))))
              name)
  #+sbcl (let* ((socket
                 (make-instance 'sb-bsd-sockets:inet-socket :type :stream
                                :protocol :tcp))
                (handler-fn (lambda (fd)
                              (declare (ignore fd))
                              (let ((stream
                                     (sb-bsd-sockets:socket-make-stream
                                      (sb-bsd-sockets:socket-accept socket)
                                      :element-type 'character
                                      :input t
                                      :output t
                                      :buffering :none)))
                                (funcall connection-handler stream)))))
           (setf (sb-bsd-sockets:sockopt-reuse-address socket) t)
           (sb-bsd-sockets:socket-bind socket #(0 0 0 0) port)
           (sb-bsd-sockets:socket-listen socket 15)
           (push (list name socket
                       (sb-sys:add-fd-handler 
                        (sb-bsd-sockets:socket-file-descriptor socket)
                        :input handler-fn)) *server-processes*)
           (values name socket)))

(defun stop-server (name)
  "Kill a server process by name (as started by start-standard-server)"
  #+lispworks
  (let ((server-process (mp:find-process-from-name name)))
    (when server-process
      (mp:process-kill server-process)))
  #+openmcl
  (let ((server-process (find name (ccl:all-processes) 
                              :key #'ccl:process-name :test #'string-equal)))
    (when server-process
      (ccl:process-kill server-process)))
  #+sbcl
  (progn
    (destructuring-bind (name socket handler)
        (assoc name *server-processes* :test #'string=)
      (declare (ignore name))
      (sb-sys:remove-fd-handler handler)
      (sb-bsd-sockets:socket-close socket))
    (setf *server-processes* (delete name *server-processes*
                                     :key #'car :test #'string=)))
  name)

;;;; eof
