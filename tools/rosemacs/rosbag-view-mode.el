(define-derived-mode rosbag-view-mode
  view-mode "Rosbag view mode"
  "Major mode for viewing ROS bag files.  See view-mode documentation for more info.

\\{rosbag-view-mode-map}"
  (let ((f (buffer-file-name)))
    (let ((buffer-read-only nil))
      (erase-buffer)
      (message "Calling rosbag info")
      (call-process "rosbag" nil (current-buffer) nil
                    "info" f)
      (set-buffer-modified-p nil))))





(provide 'rosbag-view-mode)