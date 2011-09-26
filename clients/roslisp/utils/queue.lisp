;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Software License Agreement (BSD License)
;; 
;; Copyright (c) 2008, Willow Garage, Inc.
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with 
;; or without modification, are permitted provided that the 
;; following conditions are met:
;;
;;  * Redistributions of source code must retain the above 
;;    copyright notice, this list of conditions and the 
;;    following disclaimer.
;;  * Redistributions in binary form must reproduce the 
;;    above copyright notice, this list of conditions and 
;;    the following disclaimer in the documentation and/or 
;;    other materials provided with the distribution.
;;  * Neither the name of Willow Garage, Inc. nor the names 
;;    of its contributors may be used to endorse or promote 
;;    products derived from this software without specific 
;;    prior written permission.
;; 
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
;; CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
;; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
;; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
;; PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
;; COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
;; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
;; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
;; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
;; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
;; OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
;; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
;; DAMAGE.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defpackage :roslisp-queue
  (:documentation "A threadsafe implementation of queues")
  (:export
   :queue
   :make-queue
   :queue-empty
   :enqueue
   :queue-size
   :dequeue
   :dequeue-wait
   :wakeup
   :peek-front)
  (:use :cl 
	:roslisp-utils
	:roslisp-extended-reals
	:sb-thread))

(in-package :roslisp-queue)

(defstruct (queue (:conc-name nil) (:constructor create-queue))
  lock
  read-cv
  max-size
  queue-size
  head
  tail)

(defun make-queue (&key (sequence nil) (max-size 'infty))
  "make-queue &key (SEQUENCE nil) (MAX-SIZE 'infty)

Makes a queue out of SEQUENCE as if enqueueing the elements in sequence."
  (assert (or (eql max-size 'infty) (< (length sequence) max-size)))
  (let* ((head (etypecase sequence (list (copy-list sequence)) (vector (map 'list #'identity sequence))))
         (tail (last head))
         (queue
           (create-queue :head head
                         :tail tail
                         :lock (make-mutex :name "queue lock")
                         :max-size max-size
                         :queue-size (length head)
                         :read-cv (make-waitqueue :name "queue reader waitqueue"))))
    queue))

(defun queue-empty (queue)
  "queue-empty QUEUE.  Return t iff the queue has no elements."
  (null (head queue)))

(defun enqueue (item q)
  "enqueue ITEM QUEUE.  Add ITEM to the end of the QUEUE.  If any dequeue-wait calls are waiting, one of them will wake up.  If MAX-SIZE is exceeded, dequeue until not.  Return the number of dropped items."
  (with-recursive-lock ((lock q))
    (incf (queue-size q))
    (let ((new-pair (list item))
	  (num-dropped 0))
      (cond
	((queue-empty q)
	 (setf (head q) (setf (tail q) new-pair))
	 (condition-notify (read-cv q) 1))
	(t
	 (setf (tail q)
	       (setf (cdr (tail q))
		     new-pair))
	 (repeat (extmax 0 (ext- (queue-size q) (max-size q)))
	   (dequeue q)
	   (incf num-dropped))))
      num-dropped)))

		 
	 
      

(defun dequeue (q)
  "dequeue QUEUE.  Return 1) the first item on the queue if it exists (in this case the item is removed from the queue) 2) t if the queue was nonempty, nil otherwise.  See also dequeue-wait."
  (with-recursive-lock ((lock q))
    (decf (queue-size q))
    (if (queue-empty q)
	(values nil nil)
	(multiple-value-prog1 
	    (values (car (head q)) t)
	  (setf (head q) (cdr (head q)))))))

(defun dequeue-wait (q &key (allow-wakeup nil))
  "dequeue-wait QUEUE &key (ALLOW-WAKEUP t).  Sleep till QUEUE is nonempty, then return the first item and t, or wakeup is called on the queue, in which case return nil and nil. If there are multiple threads doing this, there is no guarantee about which enqueuing this one will wake up upon.  However, the overall ordering of dequeued items will be FIFO.  If wakeup is called when ALLOW-WAKEUP is nil, then an assert happens."
  ;; TODO wakeup isn't working
  (with-recursive-lock ((lock q))
    (loop
       (if (queue-empty q)
	   (condition-wait (read-cv q) (lock q))
	   (multiple-value-bind (item exists) (dequeue q)
	     (if exists
		 (return (values item t))
		 (unless allow-wakeup
		   (assert nil nil "Thread woken up in call to dequeue-wait with allow-wakeup set to false"))))))))
  

(defun wakeup (q)
  "Wakeup all dequeuers and tell them they aren't getting anything."
  ;; TODO wakeup isn't working
  (with-recursive-lock ((lock q))
    (condition-broadcast (read-cv q))))

(defun peek-front (q)
  "peek-front QUEUE.  Has the same return values as dequeue, but does not modify the queue."
  (with-recursive-lock ((lock q))
    (if (queue-empty q)
	(values nil nil)
	(car (head q)))))

(defmethod print-object ((q queue) str)
  (with-recursive-lock ((lock q))
    (print-unreadable-object (q str :type t :identity nil)
      (format str "with elements ~a" (head q)))))