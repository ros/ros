(defpackage :roslisp-test
    (:use :cl :roslisp :roslisp-srv :roslisp-msg)
  (:import-from :std_msgs-msg
		:<String>)
  (:export :service-server
	   :service-client
	   :complex-talker
	   :complex-listener
	   :talker
	   :listener))