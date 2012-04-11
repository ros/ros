
(defsystem :roslisp-utils
  :name "roslisp-utils"

  :components
  ((:file "utils")
   (:file "float-bytes" :depends-on ("utils"))
   (:file "extended-reals")
   (:file "queue" :depends-on ("utils" "extended-reals"))
   (:file "hash-utils" :depends-on ("utils"))))
