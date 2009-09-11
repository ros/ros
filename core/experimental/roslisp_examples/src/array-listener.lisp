(roslisp:load-message-types "robot_msgs/ChannelFloat32")

(defpackage :roslisp-array-listener
  (:use :cl :roslisp :robot_msgs)
  (:export :main))

(in-package roslisp-array-listener)
  
(defun main ()
  "Like listener, except illustrates an array message."
  (with-ros-node ("listener" :spin t)
    (subscribe "array_chatter" "robot_msgs/ChannelFloat32" #'print)))


  
  