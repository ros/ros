
(in-package :asdf)

(defsystem "test_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    ))
