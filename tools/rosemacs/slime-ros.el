
(require 'slime-asdf)
(require 'rosemacs)

(defcustom slime-ros-completion-function 'completing-read
  "The completion function to be used for package and system
  completions. This variable can be set to `ido-completing-read'
  to enable `ido-mode' for ros packages."
  :type 'function
  :group 'rosemacs)

(defvar slime-ros-package-history nil)

(defun slime-ros-read-pkg-name (&optional prompt default-value)
  (cond ((not (slime-current-connection))
          (message "Not connected."))
        (t
         (let ((default (slime-eval `(cl:identity ros-load:*current-ros-package*))))
           (ros-completing-read-package nil default slime-ros-completion-function)))))

(defun slime-ros-get-systems-in-pkg (package &optional default-value prompt)
  (let* ((package-path (ros-package-path package))
         (asd-files (ros-files-in-package package-path "asd" "asdf"))
         (default (when (member default-value asd-files)
                    default-value))
         (prompt (concat (or prompt (format "ROS Package `%s', System" package))
                         (if default
                             (format " (default `%s'): " default)
                             ": "))))
    (funcall slime-ros-completion-function
             prompt (slime-bogus-completion-alist asd-files)
             nil nil nil nil default)))

(defslime-repl-shortcut slime-repl-load-ros-system ("ros-load-system")
  (:handler (lambda ()
              (interactive)
              (let* ((ros-pkg-name (slime-ros-read-pkg-name))
                     (system-name (slime-ros-get-systems-in-pkg ros-pkg-name ros-pkg-name)))
                (slime-eval `(cl:setf ros-load:*current-ros-package* ,ros-pkg-name))
                (slime-oos system-name 'load-op)))))

(defslime-repl-shortcut slime-repl-load-ros-system ("ros-test-system")
  (:handler (lambda ()
              (interactive)
              (let* ((ros-pkg-name (slime-ros-read-pkg-name))
                     (system-name (slime-ros-get-systems-in-pkg ros-pkg-name ros-pkg-name)))
                (slime-eval `(cl:setf ros-load:*current-ros-package* ,ros-pkg-name))
                (slime-oos system-name 'test-op)))))

(defun slime-ros ()
  (interactive)
  (let* ((sbcl-pkg-path (ros-package-dir "sbcl"))
         (roslisp-path (ros-package-dir "roslisp"))
         (sbcl-binary (if sbcl-pkg-path
                          (concat sbcl-pkg-path "/scripts/run-sbcl.sh")
                          "/usr/bin/sbcl"))
         (inferior-lisp-program (concat sbcl-binary " --load " roslisp-path
                                        "/scripts/roslisp-sbcl-init"))

         ;; Override user's implementations if set
         (slime-lisp-implementations nil))

    (slime)))

(provide 'slime-ros)
