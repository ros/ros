
(in-package :asdf)

(defsystem :roslisp-msg-protocol
  :name "roslisp-msg"

  :serial t
  :components
  ((:file "package")
   (:file "msg-protocol")))

