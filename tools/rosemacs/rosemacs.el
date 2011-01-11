;;; rosemacs.el --- Tools for ROS users

;; Copyright (c) 2008, Willow Garage, Inc.
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with or without
;; modification, are permitted provided that the following conditions are met:
;;
;;     * Redistributions of source code must retain the above copyright
;;       notice, this list of conditions and the following disclaimer.
;;     * Redistributions in binary form must reproduce the above copyright
;;       notice, this list of conditions and the following disclaimer in the
;;       documentation and/or other materials provided with the distribution.
;;     * Neither the name of the Willow Garage, Inc. nor the names of its
;;       contributors may be used to endorse or promote products derived from
;;       this software without specific prior written permission.
;;
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;; POSSIBILITY OF SUCH DAMAGE.
;;

;; Author: Bhaskara Marthi
;; Keywords: tools, convenience

;;; Commentary:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Installation instructions
;; 1. Put this file somewhere (if it's not already in 
;;    tools/rosemacs in your ros tree)
;;
;; 2. (Optional) From emacs do M-x byte-compile followed by 
;;    this file's full path 
;;
;; 3. Add the following lines to your .emacs
;;    (add-to-list 'load-path "/path/to/rosemacs")
;;    (require 'rosemacs)
;;    (invoke-rosemacs)
;;
;; 4. Add the following line or equivalent to 
;;    .emacs to activate keyboard shortcuts for the added 
;;    commands (\C-x\C-r means control-x control-r):
;;    (global-set-key "\C-x\C-r" ros-keymap)
;;
;; 5. Make sure the standard ROS variables are set in the
;;    emacs process environment.  If you follow the standard
;;    ROS installation instructions about sourcing .bashrc.ros
;;    in your .bashrc, then this will automatically happen
;;    if you launch emacs from a bash shell.
;; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Usage
;;
;; 0. If you did 4 above, you can type the prefix followed
;;    by \C-h to see the list of added commands.
;;
;; 1. Directory tracking and tab completion for rosbash 
;;    commands, including rostopic and rosnode, should now
;;    work correctly in shell mode
;;
;; 2. The commands {find|view}-ros-{file|message|service}, 
;;    and view-most-recent-ros-log for navigating the ros
;;    libraries are available.  Tab completion should work 
;;    for all of them.
;;
;; 3. The customization option ros-topic-update-interval governs
;;    how frequently rosemacs polls the list of ros topics
;;    and nodes.  Assuming this is positive, it will enable tab 
;;    completion of ros topics in the shell and for commands
;;    such as echo-ros-topic.  Additionally, you can use 
;;    add-hz-update to define a list of topics for which the 
;;    hz rate is tracked in the background, viewable using 
;;    display-ros-topic-info.
;;
;; 4. Similarly, set ros-node-update-interval to set up
;;    tracking and completion of nodes.  
;;
;; 5. ros-core starts a core.  ros-run runs a node.  In
;;    either case, an appropriately named buffer is created
;;    for the new process.
;;
;; 6. ros-launch to start a launch file in a new buffer.
;;    Within that buffer, k to kill, r to relaunch.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;; Code:

(require 'shell)
(require 'cl)
(require 'warnings)
(require 'time-stamp)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Parameters
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Moved to end of file

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; State
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defvar ros-packages nil "Vector of ros packages")
(defvar ros-package-locations nil "Vector of directories containing the items in ros-packages")
(defvar ros-messages nil "Vector of ros messages")
(defvar ros-message-packages nil "Vector of packages corresponding to each ros message")
(defvar ros-services nil "Vector of ros services")
(defvar ros-service-packages nil "Vector of packages corresponding to each service")
(defvar ros-root (getenv "ROS_ROOT"))
(defvar ros-topics nil "Vector of current published ros topics")
(defvar ros-subscribed-topics nil "Subscribed topics")
(defvar ros-all-topics nil "All topics (generated from published and subscribed)")
(defvar ros-topic-hertz-processes nil "Alist from topic name to process running rostopic hz on that topic")
(defvar ros-topic-publication-rates nil "Hash table from topic name to hertz rate of that topic")
(defvar ros-topic-last-hz-rate nil "Alist from topic name to last time we saw output from rostopic hz")
(defvar ros-topic-buffer (get-buffer-create "*ros-topics*") "Holds the buffer *ros-topics* if it exists")
(defvar ros-events-buffer (get-buffer-create "*ros-events*"))
(defvar ros-hz-topic-regexps nil "If a topic name matches one of these, it is hz tracked")
(defvar ros-topic-timer nil "If non-nil, equals timer object used to schedule calls to rostopic list")
(defvar ros-num-publishers (make-hash-table :test 'equal) "num publishers of a topic")
(defvar ros-num-subscribers (make-hash-table :test 'equal) "num subscribers of a topic")
(defvar ros-find-args nil)
(defvar ros-find-args-history nil)
(defvar rosemacs/pathname nil "Will hold the path containing this file")
(defvar rosemacs/invoked t)
(defvar rosemacs/nodes nil "List of nodes")
(defvar rosemacs/nodes-vec (vector) "Vector of nodes")

(defvar ros-buffer-package nil "A buffer-local variable for caching the current buffer's ros package.")
(make-variable-buffer-local 'ros-buffer-package)
(with-current-buffer (get-buffer-create "*ros-topics*") (insert "Uninitialized (use the display-ros-topic-info command rather than just switching to this buffer)"))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Preloading
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ros-load-package-locations ()
  "Reload locations of ros packages by calling out to rospack list"
  (interactive)
  (with-temp-buffer
    (let ((l nil))
      (message "Calling rospack")
      (call-process "rospack" nil t nil "list")
      (goto-char (point-min)) 
      (message "Parsing rospack output")
      (let ((done nil))
	;; Loop over lines; each line contains a package and directory
	(while (not done)
	  (let ((p (point)))
	    ;; Search for string terminated by space
	    (setq done (not (re-search-forward "[[:space:]]" (point-max) t)))
	    (unless done
	      (let ((package (buffer-substring p (1- (point)))))
		(setq p (point))
		;; Search for following string terminated by newline
		(re-search-forward "\n")
		(let ((dir (buffer-substring p (1- (point)))))
		  (push (cons package dir) l)))))))
      (let ((package-alist (sort* (vconcat l) (lambda (pair1 pair2) (string< (car pair1) (car pair2))))))
	(setq ros-packages (map 'vector #'car package-alist)
	      ros-package-locations (map 'vector #'cdr package-alist)
	      ros-messages nil
	      ros-message-packages nil
	      ros-services nil
	      ros-service-packages nil))
      (message "Done loading ROS package info"))))

(defun ros-files-in-package (dir ext &optional subdir)
  "Return list of files in subdirectory ext/ of dir whose extension is .ext"
  (with-temp-buffer
    (let ((l nil)
	  (done nil)
	  (p nil))
      (call-process "ls" nil t nil (concat dir "/" (or subdir ext) "/"))
      (goto-char (point-min))
      (while (not done)
	(setq p (point))
	(setq done (not (re-search-forward "\\([^[:space:]]+\\)[[:space:]]+" (point-max) t)))
	(unless done
	  (let ((str (buffer-substring (match-beginning 1) (match-end 1))))
            (let ((m (string-match (concat "\." ext "$") str)))
              (when m
                (push (substring str 0 m) l))))))
      l)))


(defun all-files-in-packages (ext)
  "Look in each package for files with a extension .ext in subdirectory ext/"
  (unless ros-package-locations
    (ros-load-package-locations))
  (let ((l nil))
    (dotimes-with-progress-reporter (i (length ros-package-locations)) (concat "Caching locations of ." ext " files: ")
      (let ((package (aref ros-packages i))
	    (dir (aref ros-package-locations i)))
	(dolist (m (ros-files-in-package dir ext))
	  (push (cons m package) l))))
    (sort* (vconcat l) (lambda (pair1 pair2) (string< (car pair1) (car pair2))))))

(defun cache-ros-message-locations ()
  "Look in each package directory for .msg files"
  (let ((v (all-files-in-packages "msg")))
    (setq ros-messages (map 'vector #'car v)
	  ros-message-packages (map 'vector #'cdr v))))

(defun cache-ros-service-locations ()
  "Look in each package directory for .srv files"
  (let ((v (all-files-in-packages "srv")))
    (setq ros-services (map 'vector #'car v)
	  ros-service-packages (map 'vector #'cdr v))))


(defun get-rosemacs-path ()
  (message load-file-name)
  (let ((ind (string-match "\\(.*\\)rosemacs.el$" load-file-name)))
    (if (not ind)
        (warn "Could not determine rosemacs path")
      (match-string 1 load-file-name))))

(setq rosemacs/pathname (get-rosemacs-path))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Lookup
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ros-package-dir (package)
  (unless ros-package-locations
    (ros-load-package-locations))
  (rosemacs-lookup-vectors package ros-packages ros-package-locations))

(defun ros-message-package (m)
  (unless ros-message-packages
    (cache-ros-message-locations))
  (rosemacs-lookup-vectors m ros-messages ros-message-packages))

(defun ros-service-package (m)
  (unless ros-service-packages
    (cache-ros-service-locations))
  (rosemacs-lookup-vectors m ros-services ros-service-packages))

(defun ros-package-for-path (path &optional allow-nonexistent)
  (let ((path (cond ((file-directory-p path)
                     (directory-file-name path))
                    ((or (file-exists-p path) allow-nonexistent)
                     (directory-file-name (file-name-directory path)))
                    (t nil))))
    (catch 'done
      (while (and path (not (equal path "/")))
        (let ((files (directory-files path)))
          (if (member "manifest.xml" files)
              (throw 'done (file-name-nondirectory path))
            (setf path (directory-file-name (file-name-directory path)))))))))

(defun ros-package-for-buffer (buffer &optional allow-nonexistent)
  (let ((fn (buffer-file-name buffer)))
    (when fn
      (ros-package-for-path fn allow-nonexistent))))

(defun get-buffer-ros-package ()
  (or ros-buffer-package
      (setq ros-buffer-package (ros-package-for-buffer (current-buffer)))))

(defun ros-current-pkg-modeline-entry ()
  (interactive)
  (let ((pkg (or ros-buffer-package (ros-package-for-buffer (current-buffer)))))
    (unless ros-buffer-package
      (if pkg
          (setf ros-buffer-package pkg)
        (setf ros-buffer-package :none)))
    (if (and pkg (not (eq pkg :none)))
        (format "(ROS Pkg: %s)" pkg)
      "")))


(defun parse-ros-file-prefix (str)
  "Divide something of the form PACKAGE/DIRS/FILE-PREFIX into its three pieces.  Or, if it's just a package prefix, return just that."
  (if (string-match "\\([^/]+\\)\\(/.*\\)" str)
      (let ((package (match-string 1 str))
	    (path (match-string 2 str)))
	(if (string-match "\\(/.*/\\)\\([^/]*\\)" path)
	    (values package (match-string 1 path) (match-string 2 path))
	  (values package "/" (substring path 1))))
    (values str nil nil)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Completion
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setq topic-completor (dynamic-completion-table (lambda (str) (rosemacs-bsearch str ros-all-topics))))
(setq node-completor (dynamic-completion-table (lambda (str) (rosemacs-bsearch str rosemacs/nodes-vec))))
(setq ros-package-completor 
      ;; Longer because it has to deal with the case of PACKAGE/PATH-PREFIX in addition to PACKAGE-PREFIX
      (dynamic-completion-table 
       (lambda (str) 
	 (unless ros-packages (ros-load-package-locations))
	 (multiple-value-bind (package dir-prefix dir-suffix) (parse-ros-file-prefix str)
	   (if dir-prefix
	       (let ((dir (concat (ros-package-dir package) dir-prefix)))
		 (let* ((files (directory-files dir nil (concat "^" dir-suffix)))
			(comps (all-completions dir-suffix files)))
		   (mapcar (lambda (comp) (concat package dir-prefix comp)) comps)))
	     (rosemacs-bsearch package ros-packages))))))



(defun comint-dynamic-complete-ros-package ()
  ;; Like the above, except in the shell
  (unless ros-packages (ros-load-package-locations))
  (let ((prefix (comint-get-ros-package-prefix)))
    (when prefix 
      (multiple-value-bind (package dir-prefix dir-suffix) (parse-ros-file-prefix prefix)
	(if dir-prefix
	    (let ((dir (concat (ros-package-dir package) dir-prefix)))
	      (let ((completions (all-completions dir-suffix (directory-files dir nil (concat "^" dir-suffix)))))
		(comint-dynamic-simple-complete dir-suffix completions)
		(skip-syntax-backward " ")))
	  (progn
            (comint-dynamic-simple-complete prefix (all-completions prefix ros-package-completor))
            (skip-syntax-backward " ")))))))

(defun comint-dynamic-complete-ros-topic ()
  (let ((prefix (comint-get-ros-topic-prefix)))
    (when prefix
      (comint-dynamic-simple-complete prefix (all-completions prefix topic-completor))
      (skip-syntax-backward " "))))

(defun comint-dynamic-complete-ros-node ()
  (let ((prefix (comint-get-ros-node-prefix)))
    (when prefix
      (comint-dynamic-simple-complete prefix (all-completions prefix node-completor))
      (skip-syntax-backward " "))))


(defun ros-completing-read-package (&optional prompt default completion-function)
  (unless ros-packages
    (ros-load-package-locations))
  (let ((completion-function (or completion-function ros-completion-function))
        (prompt (concat (or prompt "Enter package")
                        (if default
                            (format " (default `%s'): " default)
                          ": "))))
    (funcall completion-function
             prompt (map 'list (lambda (x)
                                 (cons x nil))
                         ros-packages)
             nil nil nil nil default)))

(defun ros-completing-read-pkg-file (prompt &optional default-pkg)
  (if (eq ros-completion-function 'ido-completing-read)
      (ros-ido-completing-read-pkg-file prompt default-pkg)
    (funcall ros-completion-function prompt ros-package-completor nil nil default-pkg)))

;; Ido completion
(defun ros-ido-completing-read-pkg-file (prompt &optional default-pkg)
  (unless ros-packages
    (ros-load-package-locations))
  (let ((old-ido-make-file-list (symbol-function 'ido-make-file-list-1))
        (ros-packages-list (map 'list #'identity ros-packages)))
    (flet ((pkg-expr->path (str)
                           (let ((pkg-name (second (split-string str "/"))))
                             (unless (= (length pkg-name) 0)
                               (concat (ros-package-dir pkg-name)
                                       (substring str (string-match "/" str 1)))))))
      (flet ((ido-make-file-list-1 (dir &optional merged)
                                   (let ((path (pkg-expr->path dir)))
                                     (if path
                                         (funcall old-ido-make-file-list path merged)
                                       (mapcar (lambda (pkg) (if merged 
                                                                 (cons (concat pkg "/") "/")
                                                               (concat pkg "/"))) 
                                               ros-packages-list)))))
        (substring (ido-read-file-name prompt "/"
                                       (when (member default-pkg ros-packages-list)
                                         default-pkg))
                   1)))))

(defun ros-completing-read-message (prompt &optional default)
  (unless ros-messages
    (cache-ros-message-locations))
  (let* ((ros-messages-list (map 'list 'identity ros-messages))
         (result (funcall ros-completion-function prompt
                          (map 'list (lambda (m pkg)
                                       (cons (if (> (count m ros-messages-list :test 'equal) 1)
                                                 (format "%s (%s)" m pkg)
                                               m)
                                             nil))
                               ros-messages-list ros-message-packages)
                          nil nil nil nil (when (member default ros-messages-list)
                                            default)))
         (ws-pos (position ?\s result))
         (message (substring result 0 ws-pos))
         (package (when ws-pos
                    (let ((package-str (substring result ws-pos)))
                      (substring package-str 2 (- (length package-str) 1))))))
    (if package
        (concatenate 'string package "/" message)
      message)))

(defun ros-completing-read-service (prompt &optional default)
  (unless ros-services
    (cache-ros-service-locations))
  (let* ((ros-services-list (map 'list 'identity ros-services))
         (result (funcall ros-completion-function prompt
                          (map 'list (lambda (m pkg)
                                       (cons (if (> (count m ros-services-list :test 'equal) 1)
                                                 (format "%s (%s)" m pkg)
                                               m)
                                             nil))
                               ros-services-list ros-service-packages)
                          nil nil nil nil (when (member default ros-services-list)
                                            default)))
         (ws-pos (position ?\s result))
         (service (substring result 0 ws-pos))
         (package (when ws-pos
                    (let ((package-str (substring result ws-pos)))
                      (substring package-str 2 (- (length package-str) 1))))))
    (if package
        (concatenate 'string package "/" service)
      service)))

(defun ros-completing-read-topic (prompt &optional default)
  (funcall ros-completion-function prompt (map 'list (lambda (m)
                                                       (cons m nil))
                                               ros-all-topics)
           nil nil nil nil (when (member default (map 'list 'identity ros-all-topics))
                             default)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Navigation commands
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun find-ros-file (package-name &optional dont-reload)
  "Open up the directory corresponding to PACKAGE-NAME in dired mode.  If used interactively, tab completion will work."
  (interactive (list (ros-completing-read-pkg-file "Enter ros path: ") nil))
  (multiple-value-bind (package dir-prefix dir-suffix) (parse-ros-file-prefix package-name)
    (let* ((package-dir (ros-package-dir package))
	   (path (if dir-prefix (concat package-dir dir-prefix dir-suffix) package-dir)))
      (if path
	  (find-file path)
	(if dont-reload
	    (error "Did not find %s in the ros package list." package-name)
	  (progn
            (lwarn '(rosemacs) :debug "Did not find %s.  Reloading ros package list and trying again..." package-name)
            (ros-load-package-locations)
            (find-ros-file package-name t)))))))

(defun view-ros-file (ros-file-name &optional dont-reload)
  "View (open in read-only mode with simpler editing commands — see emacs help) the file corresponding to ROS-FILE-NAME (in form packagename/filename).  If used interactively, tab completion will work."
  (interactive (list (ros-completing-read-pkg-file "Enter ros path: ") nil))
  (multiple-value-bind (package dir-prefix dir-suffix) (parse-ros-file-prefix ros-file-name)
    (let* ((package-dir (ros-package-dir package))
	   (path (if dir-prefix (concat package-dir dir-prefix dir-suffix) package-dir)))
      (if path
	  (view-file-other-window path)
	(if dont-reload
	    (error "Did not find %s in the ros package list." ros-file-name)
	  (progn
            (lwarn '(rosemacs) :debug "Did not find %s.  Reloading ros package list and trying again..." ros-file-name)
            (ros-load-package-locations)
            (view-ros-file ros-file-name t)))))))

(defun find-ros-message (message)
  "Open definition of a ros message.  If used interactively, tab completion will work."
  (interactive (list (ros-completing-read-message
                      (if (current-word t t)
                          (format "Enter message name (default %s): " (current-word t t))
                        "Enter message name: ")
                      (current-word t t))))
  (let* ((p+m (split-string message "/"))
         (p (if (cdr p+m)
                (car p+m)
              (ros-message-package message)))
         (m (car (last p+m))))
    (unless p
      (error "Could not find package for message %s" message))
    (let ((dir (ros-package-dir p)))
      (unless dir
        (error "Could not find directory corresponding to package %s" p))
      (find-file (concat dir "/msg/" m ".msg")))))

(defun find-ros-service (service)
  "Open definition of a ros service.  If used interactively, tab completion will work."
  (interactive (list (ros-completing-read-service
                      (if (current-word t t)
                          (format "Enter service name (default %s): " (current-word t t))
                        "Enter service name: ")
                      (current-word t t))))
  (let* ((p+m (split-string service "/"))
         (p (if (cdr p+m)
                (car p+m)
              (ros-service-package service)))
         (m (car (last p+m))))
    (unless p
      (error "Could not find package for service %s" service))
    (let ((dir (ros-package-dir p)))
      (unless dir
        (error "Could not find directory corresponding to package %s" p))
      (find-file (concat dir "/srv/" m ".srv")))))


;; (defun view-ros-message (message)
;;   "Open definition of a ros message in view mode.  If used interactively, tab completion will work."
;;   (interactive (list (completing-read
;; 		      (if (current-word t t)
;; 			  (format "Enter message name (default %s): " (current-word t t))
;; 			"Enter message name: ")
;; 		      message-completor nil nil nil nil (current-word t t))))
;;   (let ((p (ros-message-package message)))
;;     (if p
;; 	(let ((dir (ros-package-dir p)))
;; 	  (if dir
;; 	      (view-file-other-window (concat dir "/msg/" message ".msg"))
;; 	    (error "Could not find directory corresponding to package %s" p)))
;;       (error "Could not find package for message %s" message))))

(defun view-ros-message (message)
  "Open definition of a ros message in view mode.  If used interactively, tab completion will work."
  (interactive (list (ros-completing-read-message
                      (if (current-word t t)
                          (format "Enter message name (default %s): " (current-word t t))
                        "Enter message name: ")
                      (current-word t t))))
  (let ((max-mini-window-height 0))
    (shell-command (format "rosmsg show %s" message))))

(defun view-ros-service (service)
  "Open definition of a ros service in view mode.  If used interactively, tab completion will work."
  (interactive (list (ros-completing-read-service
                      (if (current-word t t)
                          (format "Enter service name (default %s): " (current-word t t))
                        "Enter service name: ")
                      (current-word t t))))
  (let ((max-mini-window-height))
    (shell-command (format "rossrv show %s" service))))

(defun ros-rgrep-package (ros-pkg regexp files)
  "Run a recursive grep in `ros-pkg', with `regexp' as search
pattern and `files' as file pattern. This function is similar to
RGREP but with a ros package instead of a directory as
parameter."
  (interactive (progn (grep-compute-defaults)
                      (let ((package (ros-completing-read-package
                                      nil (get-buffer-ros-package)))
                            (regexp (grep-read-regexp)))
                        (list
                         package
                         regexp
                         (grep-read-files regexp)))))
  (rgrep regexp files (ros-package-path ros-pkg)))

(defun ros-find-dired (ros-pkg args)
  "Run find in ros package `ros-pkg' with arguments `args' and
load the result in a dired buffer. This function is similar to
FIND-DIRED but with a ros package instead of a directory as
parameter."
  (interactive (list (ros-completing-read-package
                      nil (get-buffer-ros-package))
                     (read-string "Run find (within args): " ros-find-args
                                  '(ros-find-args-history . 1))))
  (find-dired (ros-package-path ros-pkg) args))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Core
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ros-core ()
  (interactive)
  (if (get-buffer "*roscore*")
      (switch-to-buffer (get-buffer "*roscore*"))
    (progn (start-process "roscore" (get-buffer-create "*roscore*") "roscore")
	   (message "roscore started"))))

(defun ros-set-master-uri (host port)
  "Set the master uri used by other commands (e.g. rostopic)"
  (interactive "sEnter master uri host: \nnEnter master uri port: ")
  (let ((uri (format "http://%s:%d" host port)))
    (setenv "ROS_MASTER_URI" uri)
    (message "Set ROS_MASTER_URI to %s" uri)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; rosnode
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun rosemacs/parse-node-list (start finish)
  (lwarn '(rosemacs) :debug "Parsing node list")
  (goto-char start)
  (let ((current-nodes nil))
    (while (re-search-forward "^\\/\\(.*\\)$" nil t)
      (when (> (match-end 0) finish)
        (return))
      (push (match-string 1) current-nodes))
    (let ((sorted-nodes (sort* current-nodes 'string<)))
     
      (destructuring-bind (added deleted)
          (rosemacs-list-diffs rosemacs/nodes sorted-nodes)
        (setq rosemacs/nodes sorted-nodes
              rosemacs/nodes-vec (vconcat rosemacs/nodes))
        (update-ros-node-buffer)
        (when added
          (lwarn '(rosemacs) :debug "New nodes: %s" added)
          (rosemacs/add-event (format "New nodes: %s" added)))
        (let ((l (length deleted)))
          (when (> l 0)
            (if (= l 1)
                (rosemacs/add-event (format "Ros node %s exited" (first deleted)) t)
              (rosemacs/add-event (format "%s ros nodes exited: %s" l deleted) t))))))))

(defun rosemacs/rosnode-filter (proc str)
  (with-current-buffer (process-buffer proc)
    (goto-char (point-max))
    (insert str)
    (let ((found-start (re-search-backward "BEGIN ROSNODE LIST$" nil t)))
      (if found-start
          (let* ((start-pt (match-end 0))
                 (found-finish (re-search-forward "END ROSNODE LIST$" nil t))
                 (end-pt (match-end 0)))
            (when found-finish
              (rosemacs/parse-node-list start-pt (match-beginning 0))
              (delete-region (point-min) end-pt)))))))


(defun rosemacs/track-nodes (interval)
  (interactive "nEnter rosnode update interval in seconds (0 to stop tracking): ")
  (let ((name "*rosnode-tracker*"))
    (let ((old-proc (get-process name)))
      (when old-proc
        (message "Cancelling existing rosnode tracker")
        ;; doesn't seem to respond to sigint reliably
        (delete-process old-proc))
      (when (> interval 0)
        (let ((proc (start-process name name (concat rosemacs/pathname "poll-rosnode") (format "%s" interval))))
          (set-process-query-on-exit-flag proc nil)
          (set-process-filter proc 'rosemacs/rosnode-filter)))
      )))

(defun rosemacs/get-stamp-string ()
  (goto-char (point-min))
  (let ((pos (re-search-forward "^\\(Last updated.*\\)$" nil t)))
    (if pos
	(match-string 1)
      "Last updated: <>")))

(defun update-ros-node-buffer ()
  (let ((ros-node-buffer (get-buffer-create "*ros-nodes*")))
    (save-excursion
      (set-buffer ros-node-buffer)
      (let ((old-stamp (rosemacs/get-stamp-string)))
        (erase-buffer)
        (princ (format "Master uri: %s\n" (getenv "ROS_MASTER_URI")) ros-node-buffer)
        (princ old-stamp ros-node-buffer)
        (let ((time-stamp-pattern "5/^Last updated: <%02H:%02M:%02S")) (time-stamp))
        (princ "\n\n" ros-node-buffer)
        (dolist (n rosemacs/nodes)
          (princ (format "%s\n" n) ros-node-buffer))))))


(defun rosemacs/display-nodes (&optional other-window)
  (interactive)
  (let ((buf (get-buffer-create "*ros-nodes*")))
    (if other-window
        (display-buffer buf)
      (switch-to-buffer buf))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; rostopic
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun rosemacs-topic-filter (proc str)
  (with-current-buffer (process-buffer proc)
    (goto-char (point-max))
    (insert str)
    (let ((found-start (re-search-backward "BEGIN ROSTOPIC LIST$" nil t)))
      (if found-start
          (let* ((start-point (match-end 0))
                 (found-finish (re-search-forward "END ROSTOPIC LIST" nil t)))
            (if found-finish
                (let ((finish-pt (match-beginning 0)))
                  (rosemacs/parse-topic-list start-point finish-pt)
                  (delete-region (point-min) finish-pt))
              ))))))

(defun rosemacs/track-topics (interval)
  (interactive "nEnter rostopic update interval in seconds (0 to stop tracking).")
  (let ((name "*rostopic-tracker*"))
    (let ((old-proc (get-process name)))
      (when old-proc
        (message "Cancelling existing rostopic tracker")
        (delete-process old-proc)
        ))
    (when (> interval 0)
      (let ((proc (start-process name name (concat rosemacs/pathname "poll-rostopic")  (format "%s" interval))))
        (set-process-query-on-exit-flag proc nil)
        (set-process-filter proc 'rosemacs-topic-filter))
      )))

(defun display-ros-topic-info ()
  "Display current ros topic info in *ros-topics* buffer"
  (interactive)
  (let ((buf (get-buffer "*ros-topics*")))
    (if buf
	(switch-to-buffer buf)
      (progn
	(setq ros-topic-buffer (get-buffer-create "*ros-topics*"))
	(switch-to-buffer ros-topic-buffer)))
    (ros-topic-list-mode 1)
    (update-ros-topic-buffer)))

(defun add-hz-update (topic-regexp)
  (interactive (list (ros-completing-read-topic "Enter topic name or regexp to track: ")))
  (push topic-regexp ros-hz-topic-regexps)
  (dolist (topic ros-topics)
    (when (string-match topic-regexp topic)
      (unless (assoc topic ros-topic-last-hz-rate)
	(start-hz-tracker topic)))))


(defun remove-hz-update (topic-regexp)
  (interactive (list (funcall ros-completion-function "Enter regexp to stop tracking: " ros-hz-topic-regexps)))
  (setq ros-hz-topic-regexps (delete topic-regexp ros-hz-topic-regexps))
  (dolist (pair ros-topic-last-hz-rate)
    (let ((topic (car pair)))
      (when (string-match topic-regexp topic)
	(stop-hz-tracker topic)))))


(defun echo-ros-topic (topic)
  "Create a new buffer in which rostopic echo is done on the given topic (read interactively, with tab-completion)"
  (interactive (list (let ((word (current-word)))
                       (ros-completing-read-topic
                        (if word
                            (format "Enter topic name (default %s): " word)
                          "Enter topic name: ")
                        word))))
  (let* ((topic-full-name (if (string-match "^/" topic) topic (concat "/" topic)))
         (buffer-name (concat "*rostopic:" topic-full-name "*"))
         (process (start-process buffer-name buffer-name "rostopic" "echo" topic-full-name)))
    (view-buffer-other-window (process-buffer process))
    (ros-topic-echo-mode 1)))

(defun ros-topic-info (topic)
  "Print info about topic, using rostopic info"
  (interactive (list (let ((word (current-word)))
                       (ros-completing-read-topic
                        (if word
                            (format "Enter topic name (default %s): " word)
                          "Enter topic name: ")
                        word))))
  (let* ((topic-full-name (if (string-match "^/" topic) topic (concat "/" topic)))
         (buffer-name (format "*rostopic-info:%s" topic))
         (buf (get-buffer-create buffer-name)))
    (with-current-buffer buf
      (let ((buffer-read-only nil))
        (erase-buffer)))
    (view-buffer-other-window buf)
    (call-process "rostopic" nil buf t "info" topic-full-name)))


(defun rosemacs/interrupt-process ()
  (interactive)
  (interrupt-process))

(defun rosemacs/terminate-process ()
  (interactive)
  (signal-process (get-buffer-process (current-buffer)) 'SIGTERM)
  )

(defun rosemacs/kill-process-buffer ()
  (interactive)
  (let ((process (get-buffer-process (current-buffer))))
    (if process
        (progn
          (when (eq (process-status process) 'run)
            (interrupt-process))
      ;; Give it time to shutdown cleanly
          (set-process-sentinel process '(lambda (proc event)
                                           (let ((buf (process-buffer proc)))
                                             (message "Killing %s in response to process event %s" buf event)
                                             (kill-buffer buf)))))
      (kill-buffer (current-buffer)))))

(defvar ros-topic-echo-keymap (make-sparse-keymap))
(define-key ros-topic-echo-keymap "k" 'rosemacs/terminate-process)
(define-key ros-topic-echo-keymap "q" 'rosemacs/kill-process-buffer)

(define-minor-mode ros-topic-echo-mode 
  "Mode used for rostopic echo.  

k kills the process (sends SIGINT).
q kills the buffer and process"
  :init-value nil
  :lighter " ros-topic-echo"
  :keymap ros-topic-echo-keymap
  (message "ros-topic-echo-mode: k to stop, q to quit"))


(defvar ros-topic-list-keymap (make-sparse-keymap))
(define-key ros-topic-list-keymap [?q] 'rosemacs/kill-process-buffer)
(define-key ros-topic-list-keymap [?\r] 'echo-current-topic)
(define-key ros-topic-list-keymap [?h] 'hz-current-topic)
(define-key ros-topic-list-keymap [?H] 'unhz-current-topic)
(define-key ros-topic-list-keymap [?i] 'ros-topic-info)

(defun hz-current-topic ()
  (interactive)
  (add-hz-update (ros-emacs-current-word)))

(defun unhz-current-topic ()
  (interactive)
  (remove-hz-update (ros-emacs-current-word)))

(defun echo-current-topic ()
  (interactive)
  (echo-ros-topic (ros-emacs-current-word)))

(define-minor-mode ros-topic-list-mode
  "Mode used for *ros-topics* buffer
q kills buffer"
  :init-value nil
  :keymap ros-topic-list-keymap
  (message "ros-topic-mode: enter to echo, h/H to start/stop hertz tracking, q to quit"))


(defun update-ros-topic-buffer ()
  "Use the current value of ros-topic related variables to reset the contents of the *ros-topics* buffer"
  (when (and ros-topic-buffer (get-buffer-window ros-topic-buffer))
    (if (equal (current-buffer) ros-topic-buffer)
	(let ((old-point (point)))
	  (update-ros-topic-buffer-helper)
	  (goto-char (min (point-max) old-point)))
      (save-excursion
	(update-ros-topic-buffer-helper)))))



(defun update-ros-topic-buffer-helper ()
  (setq ros-topic-buffer (get-buffer-create "*ros-topics*"))
  (set-buffer ros-topic-buffer)
  (let ((old-stamp (rosemacs/get-stamp-string)))
    (erase-buffer)
    (princ (format "Master uri: %s\n" (getenv "ROS_MASTER_URI")) ros-topic-buffer)
    (princ old-stamp ros-topic-buffer))
  (when ros-topic-publication-rates
    (princ (format "\nHz-tracked topics:\n") ros-topic-buffer)
    (dolist (topic ros-topics)
      (let ((rate-pair (assoc topic ros-topic-publication-rates)))
	(when rate-pair
	  (let ((rate (cdr rate-pair))
		(diff (- (second (current-time)) (or (cdr (assoc topic ros-topic-last-hz-rate)) 0.0))))
	    (if rate
		(if (> diff ros-topic-timeout-rate)
		    (princ (format " %s : %s (as of %s seconds ago)" topic rate diff) ros-topic-buffer)
		  (princ (format " %s : %s" topic rate) ros-topic-buffer))
	      (princ (format " %s : not yet known" topic) ros-topic-buffer)))
	  (terpri ros-topic-buffer))))
    (terpri ros-topic-buffer))

  (princ (format "\nTopic, #pubs, #subs\n\n") ros-topic-buffer)

  (dotimes (i (length ros-all-topics))
    (let ((topic (aref ros-all-topics i)))
      (princ (format " %s %s %s" topic (gethash topic ros-num-publishers 0) (gethash topic ros-num-subscribers 0)) ros-topic-buffer)
      (terpri ros-topic-buffer))))


(defun rosemacs/get-topics (start end h)
  (let ((done nil) (current-topics nil))
    (goto-char start)
    (while (not done)
      (let ((pos (re-search-forward "^\\s-*\\*\\s-*\\(\\S-*\\) \\[.*\\] \\(\\S-*\\)" end t)))
	(if pos
	    (let ((topic (match-string 1)))
	      (push topic current-topics)
	      (setf (gethash topic h) (match-string 2)))
	  (setq done t))))
    (sort* current-topics 'string<)))



(defun rosemacs/parse-topic-list (start finish)
  (lwarn '(rosemacs) :debug "Parsing rostopic list")
  (goto-char start)
  (let ((pub-start (re-search-forward "Published topics:" nil t))
        (sub-start (or (re-search-forward "Subscribed topics:" nil t) (point-max))))
    (if (and pub-start sub-start)
        (let ((new-published-topics (rosemacs/get-topics pub-start sub-start ros-num-publishers)))
          (setq ros-subscribed-topics (rosemacs/get-topics sub-start (point-max) ros-num-subscribers))
          (let ((new-topics (sort* (remove-duplicates (vconcat new-published-topics ros-subscribed-topics)
                                                      :test 'equal)
                                   'string<)))
            (destructuring-bind (added deleted) (rosemacs-list-diffs ros-topics (append  new-topics nil))
              (lwarn '(rosemacs) :debug "added topics : %s" added)
              (dolist (topic added)
                (add-ros-topic topic))
              (dolist (topic deleted)
                (remove-ros-topic topic)))))
      (lwarn '(rosemacs) :debug "rostopic output did not look as expected; could just be that the master is not up.")))
  (lwarn '(rosemacs) :debug "Done parsing rostopic list")
  (setq ros-all-topics 
        (sort* (remove-duplicates (vconcat ros-topics ros-subscribed-topics) :test 'equal) 'string<))
  ;; update display
  (save-excursion
    (unless (and ros-topic-buffer (buffer-name ros-topic-buffer))
      (setq ros-topic-buffer (get-buffer-create "*ros-topics*")))
    
    (set-buffer ros-topic-buffer)
    (update-ros-topic-buffer)
    (let ((time-stamp-pattern "5/^Last updated: <%02H:%02M:%02S"))
      (time-stamp))))



(defun remove-ros-topic (topic)
  "Remove this topic and all associated entries from topic list, completion list, hertz processes, publication rates"
  (lwarn '(rosemacs) :debug "removing ros topic %s" topic)
  (stop-hz-tracker topic) 
  (setq ros-topics (delete topic ros-topics))
  )

(defun stop-hz-tracker (topic)
  (let ((pair (assoc topic ros-topic-hertz-processes)))
    (when pair (kill-buffer (process-buffer (cdr pair)))))
  (setq ros-topic-hertz-processes (delete-if (lambda (pair) (equal (car pair) topic)) ros-topic-hertz-processes)
	ros-topic-publication-rates (delete-if (lambda (pair) (equal (car pair) topic)) ros-topic-publication-rates)
	ros-topic-last-hz-rate (delete-if (lambda (pair) (equal (car pair) topic)) ros-topic-last-hz-rate)))


(defun set-ros-topic-hz (topic rate)
  "Set hertz rate of topic.  Also, update the last-published-hertz-rate timestamp of the topic"
  (let ((rate-pair (assoc topic ros-topic-publication-rates)))
    (if rate-pair
	(setf (cdr rate-pair) rate)
      (push (cons topic rate) ros-topic-publication-rates)))
  (let ((last-time-pair (assoc topic ros-topic-last-hz-rate))
	(time-in-seconds (second (current-time))))
    (if last-time-pair
	(setf (cdr last-time-pair) time-in-seconds)
      (push (cons topic time-in-seconds) ros-topic-last-hz-rate)))
  (lwarn '(rosemacs) :debug "Updated hz for topic %s" topic))

(defun ros-topic-hz-filter (proc string)
  "Given the hertz process and string containing output from it, update the entry for the corresponding topic's publication rate"
  (let ((pair (rassoc proc ros-topic-hertz-processes)))
    (if pair
	(let ((hz (ros-topic-extract-hz string))
	      (topic (car pair)))
	  (cond 
	   ((eql hz 'not-published) (remove-ros-topic topic))
	   (hz (set-ros-topic-hz topic hz))))
      (lwarn '(rosemacs) :warning "Unexpectedly could not find topic corresponding to process %s" (process-name proc)))))


(defun ros-topic-extract-hz (string)
  "Given a string, if it contains something that looks like a hz reading, return the first such one, else if it contains error, return the string error, else if not published yet, return the number -1, else return nil"
  ;; This will not work well if the process filter ends up being called too frequently, in which case no one call will match the regexp.
  ;; Empirically, that does not seem to happen.
  (let ((case-fold-search t))
    (cond
     ((string-match "error" string) "error")
     ((string-match "average rate: \\([0-9]+\\.[0-9]*\\)" string) (match-string 1 string))
     ((string-match "does not appear to be published yet" string) 'not-published))))


(defun start-hz-tracker (topic)
  (let* ((name (concat "rostopic-hz-" topic))
	 (proc (start-process name name "rostopic" "hz" topic)))
    (push (list topic) ros-topic-last-hz-rate)
    (push (list topic) ros-topic-publication-rates)
    (let ((old-proc-pair (assoc topic ros-topic-hertz-processes)))
      (if old-proc-pair
	  (progn
	    (kill-buffer (process-buffer (cdr old-proc-pair)))
	    (setf (cdr old-proc-pair) proc))
	(push (cons topic proc) ros-topic-hertz-processes)))
    (set-process-filter proc 'ros-topic-hz-filter)))

(defun satisfies-hz-regexps (topic)
  (some (lambda (regexp) (string-match regexp topic)) ros-hz-topic-regexps))

(defun add-ros-topic (topic)
  "Post: topic is added to the list of topics and related bookkeeping done: list of completions is regenerated, hertz process is started and added to list, publication-rate entry is added to list"
  (let ((pair (assoc topic ros-topic-publication-rates)))
    (when pair (setq ros-topic-publication-rates (delete pair ros-topic-publication-rates))))
  (when (satisfies-hz-regexps topic)
    (start-hz-tracker topic))
  (push topic ros-topics)
  (setq ros-topics (sort* ros-topics 'string<))
  
  )



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Shell mode
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ros-directory-tracker (str)
  "Keep buffer current directory up-to-date in the presence of roscd.  This is analogous to shell-directory-tracker in shell.el though not as thorough about special cases.  It will not work with variable substitution, any sort of program control, etc - only for simple commands and pipelines of the same.  If it gets confused, do M-x shell-resync-dirs."
  ;; Might be simpler to just call shell-resync-dirs at some point

  ;; skip whitespace
  (let ((start (progn (string-match 
		       (concat "^" shell-command-separator-regexp) str)
		      (match-end 0)))
	end cmd arg1)
    (while (string-match shell-command-regexp str start)

      (setq end (match-end 0)
	    cmd (comint-arguments (substring str start end) 0 0)
	    arg1 (comint-arguments (substring str start end) 1 1))
      (when arg1 (setq arg1 (shell-unquote-argument arg1)))
      
      (cond ((string-match "^ros[cp]d\\([[:space:]]\\|$\\)" cmd)
	     (if (string-match "\\([^/]*\\)/\\(.*\\)" arg1)
		 (let ((package (match-string 1 arg1))
		       (subdir (match-string 2 arg1)))
		   (message "Package is %s, subdir is %s" package subdir)
		   (let ((dir (ros-package-dir package)))
		     (if dir
			 (shell-process-cd (concat dir "/" subdir))
		       (lwarn '(rosemacs) :debug "Unable to find directory of ros package %s." arg1))))
	       (let ((dir (ros-package-dir arg1)))
		 (if dir
		     (shell-process-cd dir)
		   (lwarn '(rosemacs) :debug "Unable to find directory of ros package %s." arg1))))))
      ;; TODO deal with popd

      (setq start (progn (string-match shell-command-separator-regexp str end)
			 (match-end 0))))))



(defun ros-emacs-current-word ()
  (save-excursion
    (skip-syntax-backward "w_.()")
    (let ((start (point)))
      (skip-syntax-forward "w_.()")
      (buffer-substring-no-properties start (point)))))

(defun ros-emacs-last-word ()
  (let ((end (point)))
    (skip-syntax-backward "w_.()")
    (buffer-substring-no-properties (point) end)))

(defvar *ros-commands-starting-with-package* '("roscd" "rosmake" "rosrun" "rospd"))

(defun comint-get-ros-package-prefix ()
  (ros-command-prefix *ros-commands-starting-with-package*))

(defun ros-command-prefix (commands)
  (save-excursion
    (block match-block
      (let ((arg (ros-emacs-last-word)))
	(skip-syntax-backward " ")
	(dolist (cmd commands nil)
	  (when (string-equal cmd (buffer-substring-no-properties (- (point) (length cmd)) (point)))
	    (return-from match-block arg)))))))

(defun comint-get-ros-topic-prefix ()
  (save-excursion
    (let ((arg (ros-emacs-last-word)))
      (skip-syntax-backward " ")
      (ros-emacs-last-word)
      (skip-syntax-backward " ")
      (let ((start (- (point) 8)))
	(when (and (>= start 0) (string-equal "rostopic" (buffer-substring-no-properties start (point))))
	  arg)))))

(defun comint-get-ros-node-prefix ()
  (save-excursion
    (let ((arg (ros-emacs-last-word)))
      (skip-syntax-backward " ")
      (ros-emacs-last-word)
      (skip-syntax-backward " ")
      (let ((start (- (point) 7)))
	(when (and (>= start 0) (string-equal "rosnode" (buffer-substring-no-properties start (point))))
	  arg)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; rosrun
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defvar ros-run-temp-var "")
(defvar ros-run-exec-names nil)
(defvar ros-run-pkg nil "Package of the executable being rosrun")
(make-variable-buffer-local 'ros-run-pkg)
(defvar ros-run-executable nil "Executable being rosrun")
(make-variable-buffer-local 'ros-run-executable)
(defvar ros-run-args nil "Arguments to current rosrun")
(make-variable-buffer-local 'ros-run-args)

(defun extract-exec-name (path)
  (string-match "\\([^\/]+\\)$" path)
  (match-string 1 path))

(defun ros-find-executables (pkg)
  (let ((ros-run-exec-paths nil)
	(path (ros-package-path pkg)))
    (save-excursion
      (with-temp-buffer 
	(call-process "find" nil t nil path "-perm" "-100" "!" "-type" "d")
	(goto-char (point-min))
	(loop
	 (let ((pos (re-search-forward "^\\(.+\\)$" (point-max) t)))
	   (if pos
	       (let ((str (match-string 1)))
		 (push str ros-run-exec-paths))
	     (return))))))
    (sort* (map 'vector 'extract-exec-name ros-run-exec-paths) 'string<)))

(defun ros-package-path (pkg)
  (save-excursion
    (with-temp-buffer
      (call-process "rospack" nil t nil "find" pkg)
      (goto-char (point-min))
      (re-search-forward "^\\(.*\\)$")
      (match-string 1))))

(defvar ros-run-keymap (make-sparse-keymap))
(define-key ros-run-keymap "k" 'rosemacs/terminate-process)
(define-key ros-run-keymap "q" 'rosemacs/kill-process-buffer)
(define-key ros-run-keymap "r" 'rosrun/restart-current)
(define-key ros-run-keymap "x" 'rosrun/kill-and-restart)

(define-minor-mode ros-run-mode
  "Mode used for rosrun

k kills the process (sends SIGINT).
q kills the buffer and process."
  :init-value nil
  :lighter " ros-run"
  :keymap ros-run-keymap
  (message "ros-run mode: k to stop, q to quit, r to restart"))

(defun rosemacs/contains-running-process (name)
  (let ((buf (get-buffer name)))
    (and buf
         (let ((proc (get-buffer-process buf)))
           (and proc
                (member (process-status proc) '(run stop)))))))

(defun ros-run (pkg exec &rest args)
  "pkg is a ros package name and exec is the executable name.  Tab completes package name.  Exec defaults to package name itself."
  (interactive (list (setq ros-run-temp-var (ros-completing-read-package
                                             nil (get-buffer-ros-package)))
                     (funcall ros-completion-function (format "Enter executable (default %s): " ros-run-temp-var)
                              (mapcar (lambda (pkg)
                                        (cons pkg nil))
                                      (ros-find-executables ros-run-temp-var))
                              nil nil nil nil ros-run-temp-var)))
  (let* ((name (format "*rosrun:%s/%s" pkg exec))
         (buf (get-buffer-create name)))
    (save-excursion
      (set-buffer buf)
      (setq ros-run-pkg pkg
            ros-run-executable exec
            ros-run-args args))
    (rosrun/restart buf)
    ))

(defun rosrun/restart (buf)
  (if (rosemacs/contains-running-process buf)
      (warn "Rosrun buffer %s already exists: not creating a new one." (buffer-name buf))
    (progn
      (save-excursion
        (set-buffer buf)
        (ros-run-mode 1)
        (apply 'start-process (buffer-name buf) buf "rosrun" ros-run-pkg ros-run-executable ros-run-args))
      (switch-to-buffer buf))))

(defun rosrun/restart-current ()
  (interactive)
  (rosrun/restart (current-buffer)))

(defun rosemacs/kill-process-buffer ()
  (interactive)
  (let ((process (get-buffer-process (current-buffer))))
    (if process
        (progn
          (when (eq (process-status process) 'run)
            (interrupt-process))
      ;; Give it time to shutdown cleanly
          (set-process-sentinel process '(lambda (proc event)
                                           (let ((buf (process-buffer proc)))
                                             (message "Killing %s in response to process event %s" buf event)
                                             (kill-buffer buf)))))
      (kill-buffer (current-buffer)))))

(defun rosrun/kill-and-restart ()
  (interactive)
  (let ((process (get-buffer-process (current-buffer))))
    (if (and process (eq (process-status process) 'run))
        (progn
          (set-process-sentinel process '(lambda (proc event)
                                           (let ((buf (process-buffer proc)))
                                             (message "%s has terminated; restarting" ros-run-executable)
                                             (rosrun/restart))))
          (rosemacs/terminate-process process))
      (error "Buffer does not contain a running process"))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; roslaunch
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar ros-launch-path nil "The path to the file being launched")
(make-variable-buffer-local 'ros-launch-path)
(defvar ros-launch-filename nil "The file being launched")
(make-variable-buffer-local 'ros-launch-filename)

(defun ros-launch (package-name &optional other-window)
  "Launch a ros launch file in a separate buffer.  See ros-launch-mode for details."
  (interactive (list (ros-completing-read-pkg-file "Enter ros path: ")))
  (multiple-value-bind (package dir-prefix dir-suffix) (parse-ros-file-prefix package-name)
    (let* ((package-dir (ros-package-dir package))
	   (path (if dir-prefix (concat package-dir dir-prefix dir-suffix) package-dir)))
      (if path
	  (let ((name (format "roslaunch:%s/%s" package dir-suffix)))
            (if (rosemacs/contains-running-process name)
                (warn "Roslaunch buffer %s already exists: not creating a new one." name)
              (let ((buf (get-buffer-create name)))
                (save-excursion
                  (set-buffer buf)
                  (setq ros-launch-path path)
                  (setq ros-launch-filename dir-suffix)
                  (ros-launch-mode 1)
                  (rosemacs/relaunch (current-buffer)))
                
                (if other-window (display-buffer buf) (switch-to-buffer buf))
                buf)))
	(error "Did not find %s in the ros package list." package-name)))))

(defun rosemacs/open-launch-file ()
  (interactive)
  (unless ros-launch-path
    (error "Not in a ros launch buffer"))
  (find-file ros-launch-path)
  )


(defun rosemacs/relaunch (buf)
  (let ((proc (get-buffer-process buf)))
    (if (and proc (eq (process-status proc) 'run))
        (warn "Can't relaunch since process %s is still running" proc)
      (save-excursion
        (set-buffer buf)
        (erase-buffer)
        (start-process (buffer-name buf) buf "roslaunch" ros-launch-path)
        (rosemacs/add-event (format "Ros launch of %s" ros-launch-path))
        )
      )))

(defun rosemacs/kill-and-relaunch ()
  "Interrupt the current roslaunch process, and when it has terminated, relaunch."
  (interactive)
  (let ((process (get-buffer-process (current-buffer))))
    (if (and process (eq (process-status process) 'run))
        (progn
          (interrupt-process)
          (set-process-sentinel process
                                '(lambda (proc event)
                                   (message "Roslaunch has terminated; relaunching")
                                   (rosemacs/relaunch (process-buffer proc))
                                   )))
      (error "Buffer doesn't contain a running process"))))


(defun rosemacs/relaunch-current-process ()
  (interactive)
  (rosemacs/relaunch (current-buffer)))

(defvar ros-launch-keymap (make-sparse-keymap))
(define-key ros-launch-keymap "k" 'rosemacs/interrupt-process)
(define-key ros-launch-keymap "q" 'rosemacs/kill-process-buffer)
(define-key ros-launch-keymap "r" 'rosemacs/relaunch-current-process)
(define-key ros-launch-keymap "x" 'rosemacs/kill-and-relaunch)
(define-key ros-launch-keymap "f" 'rosemacs/open-launch-file)



(define-minor-mode ros-launch-mode
  "Mode used for roslaunch

k kills the process (sends SIGINT)
q kills the process and the associated buffer
r relaunches once the previous roslaunch has been killed
x terminates the currently active launch, then relaunches once it has cleanly shutdown
f opens the launch file

All roslaunches of the launch file are appended to the same buffer (until you kill that buffer).
The page delimiter in this buffer matches the start, so you can use forward/backward pagewise navigation.
"
  :init-value nil
  :lighter " ros-launch"
  :keymap ros-launch-keymap
  (make-local-variable 'page-delimiter)
  (setq page-delimiter "SUMMARY")
  )



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Event buffer
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun rosemacs/add-event (str &optional display-in-minibuffer)
  (save-excursion
    (when display-in-minibuffer (message str))
    (set-buffer ros-events-buffer)
    (goto-char (point-max))
    (princ (format "\n[%s] %s" (substring (current-time-string) 11 19) str) ros-events-buffer)
    )
  )

(defun rosemacs/display-event-buffer (&optional other-window)
  (interactive)
  (if other-window
      (display-buffer ros-events-buffer)
    (switch-to-buffer ros-events-buffer)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Keymap
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar ros-keymap (make-sparse-keymap))
(define-key ros-keymap "\C-f" 'find-ros-file)
(define-key ros-keymap "f" 'view-ros-file)
(define-key ros-keymap "\C-m" 'find-ros-message)
(define-key ros-keymap "m" 'view-ros-message)
(define-key ros-keymap "\C-s" 'find-ros-service)
(define-key ros-keymap "s" 'view-ros-service)
(define-key ros-keymap "\C-r" 'ros-run)
(define-key ros-keymap "r" 'ros-load-package-locations)
(define-key ros-keymap "\C-c" 'ros-core)
(define-key ros-keymap "\C-t" 'display-ros-topic-info)
(define-key ros-keymap "t" 'echo-ros-topic)
(define-key ros-keymap "h" 'add-hz-update)
(define-key ros-keymap "H" 'remove-hz-update)
(define-key ros-keymap "T" 'ros-topic-info)
(define-key ros-keymap "g" 'ros-rgrep-package)
(define-key ros-keymap "\C-l" 'ros-launch)
(define-key ros-keymap "\C-e" 'rosemacs/display-event-buffer)
(define-key ros-keymap "\C-n" 'rosemacs/display-nodes)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Invoking the mode
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun set-rosemacs-shell-hooks ()
  (add-hook 'comint-input-filter-functions 'ros-directory-tracker nil t)
  (add-hook 'comint-dynamic-complete-functions 'comint-dynamic-complete-ros-package nil t)
  (add-hook 'comint-dynamic-complete-functions 'comint-dynamic-complete-ros-topic nil t)
  (add-hook 'comint-dynamic-complete-functions 'comint-dynamic-complete-ros-node nil t))


(defun invoke-rosemacs ()
  (interactive)
  (add-hook 'shell-mode-hook 'set-rosemacs-shell-hooks)
  (setq rosemacs/invoked t)
  (rosemacs/track-topics ros-topic-update-interval)
  (rosemacs/track-nodes ros-node-update-interval)

  ;; nxml mode
  (when (>= emacs-major-version 23)
    (require 'rng-loc)
    (push (concat (ros-package-path "rosemacs") "/rng-schemas.xml") rng-schema-locating-files)
    (add-to-list 'auto-mode-alist '("\.launch$" . nxml-mode))
    (add-to-list 'auto-mode-alist '("manifest.xml" . nxml-mode))
    (add-to-list 'auto-mode-alist '("\\.urdf" . xml-mode))
    (add-to-list 'auto-mode-alist '("\\.xacro" . xml-mode)))

  ;; msg and srv files: for now use gdb-script-mode
  (add-to-list 'auto-mode-alist '("\\.msg\\'" . gdb-script-mode))
  (add-to-list 'auto-mode-alist '("\\.srv\\'" . gdb-script-mode))
  (add-to-list 'auto-mode-alist '("\\.action\\'" . gdb-script-mode))
  (font-lock-add-keywords 'gdb-script-mode
                          '(("\\<\\(bool\\|byte\\|int8\\|uint8\\|int16\\|uint16\\|int32\\|uint32\\|int64\\|uint64\\|float32\\|float64\\|string\\|time\\|duration\\)\\>" . font-lock-builtin-face)))


  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun rosemacs-get-comp (completions i)
  (let ((comp (aref completions i)))
    (string-match "^/*\\(.*\\)" comp)
    (match-string 1 comp)))

(defun rosemacs-bsearch (str completions)
  "str is a string, completions is a sorted vector of strings.  Return list of strings in completions that str is a prefix of."
  (let ((num-completions (length completions)))
    (unless (or (= num-completions 0)
                (string< (rosemacs-get-comp completions (1- num-completions)) str))
      (let ((i 0)
            (j (1- num-completions)))
        (while (< (1+ i) j)
          (let ((k (floor (+ i j) 2)))
            (if (string< str (rosemacs-get-comp completions k))
                (setq j k)
              (setq i k))))

        (when (not (rosemacs-is-prefix str (rosemacs-get-comp completions i)))
          (incf i))
        ;; Postcondition: completions of str, if they exist, begin at i
      
        (let ((returned-completions nil))
          (while (and (< i (length completions)) (rosemacs-is-prefix str (rosemacs-get-comp completions i)))
            (push (rosemacs-get-comp completions i) returned-completions)
            (incf i))
          returned-completions)))))

(defun rosemacs-is-prefix (str1 str2)
  (let ((m (length str1)))
    (eq t (compare-strings str1 0 m str2 0 m))))

(defun rosemacs-lookup-vectors (str v1 v2)
  (let ((i (position str v1 :test #'string-equal)))
    (when i
      (aref v2 i)))) 

(defun rosemacs-list-diffs (l1 l2)
  "Given two sorted lists of strings, return a list with 1) l2 - l1 2) l1 - l2"
  (let ((added nil) (deleted nil) (remaining1 l1) (remaining2 l2))
    (while (or remaining1 remaining2)
      (cond
       ((or (null remaining1) (and remaining2 (string< (car remaining2) (car remaining1))))
	(push (car remaining2) added)
	(setq remaining2 (cdr remaining2)))
       ((or (null remaining2) (and remaining1 (string< (car remaining1) (car remaining2))))
	(push (car remaining1) deleted)
	(setq remaining1 (cdr remaining1)))
       (t (setq remaining1 (cdr remaining1)
		remaining2 (cdr remaining2)))))
    (lwarn '(rosemacs) :debug "Diffs of %s and %s are %s and %s" l1 l2 added deleted)
    (list added deleted)))

(defun set-ros-topic-update-interval (n)
  (warn "The function set-ros-topic-update-interval is deprecated; please check the wiki/instructions for how to track topics (summary: it happens by default, and you can customize ros-topic-update-interval to change the frequency, so you just need to remove the set-ros-topic-update-interval call from your .emacs)")
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Parameters
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgroup rosemacs nil
  "Customizations for rosemacs"
  :group 'external)

(defcustom ros-completion-function 'completing-read
  "The completion function to be used for package
  completions. This variable can be set to `ido-completing-read'
  to enable `ido-mode' for ros packages."
  :type 'function
  :group 'rosemacs)

(defcustom ros-topic-update-interval 8
  "How often (seconds) to poll the list of ros topics.  0 means never."
  :type 'integer
  :group 'rosemacs
  :require 'rosemacs
  :set #'(lambda (s val)
           (set-default s val)
           (when rosemacs/invoked
             (rosemacs/track-topics val)))
  )

(defcustom ros-node-update-interval 8
  "How often (seconds) to poll the list of ros nodes.  0 means never."
  :type 'integer
  :group 'rosemacs
  :require 'rosemacs
  :set #'(lambda (s val)
           (set-default s val)
           (when rosemacs/invoked
             (rosemacs/track-nodes val)))
  )

(defvar ros-topic-timeout-rate 5 "Number of seconds before info from rostopic hz is considered out-of-date" )


    
(provide 'rosemacs)


;;; rosemacs.el ends here