
(require 'slime-asdf)
(require 'rosemacs)

(defvar slime-ros-package-history nil)

(defun slime-ros-read-pkg-name (&optional prompt default-value)
  (unless ros-packages
    (ros-load-package-locations))
  (cond ((not (slime-current-connection))
          (message "Not connected."))
        (t
         (let* ((completions ros-packages)
                (default (slime-eval `(cl:identity ros-load:*current-ros-package*)))
                (prompt (concat (or prompt "ROS Package")
                                (if default
                                    (format " (default `%s'): " default)
                                    ": "))))
           (completing-read prompt (slime-bogus-completion-alist completions)
                            nil nil nil
                            'slime-ros-package-history default)))))

(defun slime-ros-get-systems-in-pkg (package &optional default-value prompt)
  (let* ((package-path (ros-package-path package))
         (asd-files (ros-files-in-package package-path "asd" "asdf"))
         (default (when (member default-value asd-files)
                    default-value))
         (prompt (concat (or prompt (format "ROS Package `%s', System" package))
                         (if default
                             (format " (default `%s'): " default)
                             ": "))))
    (completing-read prompt (slime-bogus-completion-alist asd-files)
                     nil nil nil nil default)))

(defslime-repl-shortcut slime-repl-load-ros-system ("ros-load-system")
  (:handler (lambda ()
              (interactive)
              (let* ((ros-pkg-name (slime-ros-read-pkg-name))
                     (system-name (slime-ros-get-systems-in-pkg ros-pkg-name ros-pkg-name)))
                (slime-eval `(cl:setf ros-load:*current-ros-package* ,ros-pkg-name))
                (slime-oos system-name 'load-op)))))

(provide 'slime-ros)
