;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :roslisp)

(defclass persistent-service ()
  ((stream :reader persistent-service-stream)
   (socket :reader persistent-service-socket)
   (request-type :reader persistent-service-request-type)
   (response-type :reader persistent-service-response-type)
   (service-name :initarg :service-name)
   (service-type :initarg :service-type)))

(defgeneric call-persistent-service (service &rest request)
  (:method ((service persistent-service) &rest request)
    (with-slots (stream socket request-type response-type)
        service
      (block nil
        (loop do
          (restart-case
              (cond ((and (eql (length request) 1)
                          (typep (car request) request-type))
                     (return
                       (tcpros-do-service-request stream (car request) response-type)))
                    (t
                     (return
                       (tcpros-do-service-request
                        stream (apply #'make-instance request-type request)
                        response-type))))
            (reconnect ()
              :report "Try reconnecting persistent service and execute
             the call again."
              (close-peristent-service service)
              (establish-persistent-service-connection service))))))))

(defgeneric close-peristent-service (persistent-service)
  (:method ((service persistent-service))
    (close (persistent-service-stream service) :abort t)))

(defgeneric persistent-service-ok (persistent-service)
  (:documentation "Returns T if the service is still ok, i.e. can be
  called, NIL otherwise.")
  (:method ((service persistent-service))
    (with-slots (socket) service
      (and socket (socket-open-p socket)))))

(defgeneric establish-persistent-service-connection (service)
  (:method ((service persistent-service))
    (with-slots (service-name service-type stream socket
                 request-type response-type)
        service
      (let* ((service-type (etypecase service-type
                             (symbol service-type)
                             (string (make-service-symbol service-type)))))
        (with-fully-qualified-name service-name
          (multiple-value-bind (host port)
              (parse-rosrpc-uri (lookup-service service-name))
            (multiple-value-bind (service-stream service-socket)
                (tcpros-establish-service-connection
                 host port service-name (service-request-type service-type) t)
              (setf stream service-stream)
              (setf socket service-socket)
              (setf request-type (service-request-type service-type))
              (setf response-type (service-response-type service-type)))))))))

(defmethod initialize-instance :after ((service persistent-service) &key)
  (establish-persistent-service-connection service))
