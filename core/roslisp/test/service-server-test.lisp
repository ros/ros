(in-package :roslisp-test)

(def-service-callback (add Test) (m)
  (with-fields ((a (data a)) (b (data b))) m
    (make-response :sum (+ a b))))

(defun service-server ()
  (with-ros-node ("test_server" :spin t)
    (register-service-fn "adder" #'add 'Test)))

