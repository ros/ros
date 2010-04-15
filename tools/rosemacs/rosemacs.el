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
;; 4. (Optional) Add the following line or equivalent to 
;;    .emacs to activate keyboard shortcuts for the added 
;;    commands (\C-x\C-r means control-x control-r):
;;    (global-set-key "\C-x\C-r" ros-keymap)
;;
;; 5. (Optional) Add the following line or equivalent to 
;;    .emacs to initiate background tracking of the set 
;;    of active ros topics.
;;    (set-ros-topic-update-interval 5)
;;
;; 6. Make sure the standard ROS variables are set in the
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
;;    commands should now work correctly in shell mode
;;
;; 2. The commands {find|view}-ros-{file|message|service}, 
;;    and view-most-recent-ros-log for navigating the ros
;;    libraries are available.  Tab completion should work 
;;    for all of them.
;;
;; 3. Use ros-update-topic-list to make rosemacs update its 
;;    list of topics, and set-ros-topic-update-interval to do 
;;    so periodically in the background. This will enable tab 
;;    completion of ros topics in the shell and for commands
;;    such as echo-ros-topic.  Additionally, you can use 
;;    add-hz-update to define a list of topics for which the 
;;    hz rate is tracked in the background, viewable using 
;;    display-ros-topic-info.
;;
;; 4. ros-core starts a core.  ros-run runs a node.  In
;;    either case, an appropriately named buffer is created
;;    for the new process.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(require 'shell)
(require 'cl)
(require 'warnings)
(require 'time-stamp)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Parameters
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar ros-ignored-packages '("test_msgs" "test_roscpp_serialization_perf" "turtlesim") "List of packages to ignore when searching")
(defvar ros-topic-timeout-rate 5 "Number of seconds before info from rostopic hz is considered out-of-date" )
(defvar ros-topic-display-update-interval 3 "Number of seconds between updates to the *rostopic* buffer (when it's visible)")
(defvar ros-topic-update-interval nil "Gap in seconds between calls to rostopic list (end of one call to beginning of next).  nil means never call.")

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
(defvar ros-topic-buffer nil "Holds the buffer *ros-topics* if it exists")
(defvar ros-hz-topic-regexps nil "If a topic name matches one of these, it is hz tracked")
(defvar ros-topic-timer nil "If non-nil, equals timer object used to schedule calls to rostopic list")
(defvar ros-num-publishers (make-hash-table :test 'equal) "num publishers of a topic")
(defvar ros-num-subscribers (make-hash-table :test 'equal) "num subscribers of a topic")

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
		  (unless (member package ros-ignored-packages)
		    (push (cons package dir) l))))))))
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


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; parsing
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

(setq message-completor (dynamic-completion-table (lambda (str) (unless ros-messages (cache-ros-message-locations)) (rosemacs-bsearch str ros-messages))))
(setq service-completor (dynamic-completion-table (lambda (str) (unless ros-services (cache-ros-service-locations)) (rosemacs-bsearch str ros-services))))
(setq topic-completor (dynamic-completion-table (lambda (str) (rosemacs-bsearch str ros-all-topics))))
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


	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Navigation commands
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun find-ros-file (package-name &optional dont-reload)
  "Open up the directory corresponding to PACKAGE-NAME in dired mode.  If used interactively, tab completion will work."
  (interactive (list (completing-read "Enter ros path: " ros-package-completor) nil))
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
  (interactive (list (completing-read "Enter ros path: " ros-package-completor) nil))
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
  (interactive (list (completing-read 
		      (if (current-word t t)
			  (format "Enter message name (default %s): " (current-word t t))
			"Enter message name: ")
		      message-completor nil nil nil nil (current-word t t))))
  (let ((p (ros-message-package message)))
    (if p
	(let ((dir (ros-package-dir p)))
	  (if dir
	      (find-file (concat dir "/msg/" message ".msg"))
	    (error "Could not find directory corresponding to package %s" p)))
      (error "Could not find package for message %s" message))))

(defun find-ros-service (service)
  "Open definition of a ros service.  If used interactively, tab completion will work."
  (interactive (list (completing-read 
		      (if (current-word t t)
			  (format "Enter service name (default %s): " (current-word t t))
			"Enter service name: ")
		      service-completor nil nil nil nil (current-word t t))))
  (let ((p (ros-service-package service)))
    (if p
	(let ((dir (ros-package-dir p)))
	  (if dir
	      (find-file (concat dir "/srv/" service ".srv"))
	    (error "Could not find directory corresponding to package %s" p)))
      (error "Could not find package for service %s" service))))


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
   (interactive (list (completing-read
 		      (if (current-word t t)
 			  (format "Enter message name (default %s): " (current-word t t))
 			"Enter message name: ")
 		      message-completor nil nil nil nil (current-word t t))))
   (shell-command (format "rosmsg show %s" message)))

(defun view-ros-service (service)
  "Open definition of a ros service in view mode.  If used interactively, tab completion will work."
  (interactive (list (completing-read 
		      (if (current-word t t)
			  (format "Enter service name (default %s): " (current-word t t))
			"Enter service name: ")
		      service-completor nil nil nil nil (current-word t t))))
  (let ((p (ros-service-package service)))
    (if p
	(let ((dir (ros-package-dir p)))
	  (if dir
	      (view-file-other-window (concat dir "/srv/" service ".srv"))
	    (error "Could not find directory corresponding to package %s" p)))
      (error "Could not find package for service %s" service))))



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
;; rosrun
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defvar ros-run-temp-var "")
(defvar ros-run-exec-names nil)

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
  
(defun ros-run-complete-exec-name (str)
  (bsearch-completions str ros-run-exec-names))

(defun ros-package-path (pkg)
  (save-excursion
   (with-temp-buffer
     (call-process "rospack" nil t nil "find" pkg)
     (goto-char (point-min))
     (re-search-forward "^\\(.*\\)$")
     (match-string 1))))

(defun ros-run (pkg exec)
  "pkg is a ros package name and exec is the executable name.  Tab completes package name.  Exec defaults to package name itself."
  (interactive (list (setq ros-run-temp-var (completing-read "Enter package: " ros-package-completor))
		     (let ((ros-run-exec-names (ros-find-executables ros-run-temp-var)))
		       (completing-read (format "Enter executable (default %s): " ros-run-temp-var) (dynamic-completion-table 'ros-run-complete-exec-name)
					nil nil nil nil ros-run-temp-var))))
  (let ((name (format "*rosrun:%s/%s*" pkg exec)))
    (start-process name (get-buffer-create name) "rosrun" pkg exec)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; rostopic
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Top-level
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun ros-update-topic-list ()
  "Makes rosemacs call rostopic list and update its list of topics (ros-topics)"
  (interactive)

  (let ((ros-topic-update-interval 0)) ;; dynamic binding
    (ros-update-topic-list-internal)))

(defun set-ros-topic-update-interval (n)
  "Make rostopic list be called every n seconds starting now.  0 means never update."
  (interactive "nEnter rostopic update interval in seconds (0 means never update) : ")
  (cond
   ((= n 0)
    (setq ros-topic-update-interval nil))
   (t
    (setq ros-topic-update-interval n)
    (ros-update-topic-list-internal))))
  


(defun display-ros-topic-info ()
  "Display current ros topic info in *ros-topics* buffer"
  (interactive)
  (let ((buf (get-buffer "*ros-topics*")))
    (if buf
	(switch-to-buffer buf)
      (progn
	(setq ros-topic-buffer (get-buffer-create "*ros-topics*"))
	(switch-to-buffer ros-topic-buffer)
	(ros-topic-list-mode)))
    (ros-update-topic-list-internal)
    (update-ros-topic-buffer)))

(defun add-hz-update (topic-regexp)
  (interactive (list (completing-read "Enter topic name or regexp to track: " topic-completor)))

  ;; Asynchronously start re-gathering topic list, in case things have recently changed
  (ros-update-topic-list)
  
  (push topic-regexp ros-hz-topic-regexps)
  (dolist (topic ros-topics)
    (when (string-match topic-regexp topic)
      (unless (assoc topic ros-topic-last-hz-rate)
	(start-hz-tracker topic)))))
  

(defun remove-hz-update (topic-regexp)
  (interactive (list (completing-read "Enter regexp to stop tracking: " ros-hz-topic-regexps)))
  (setq ros-hz-topic-regexps (delete topic-regexp ros-hz-topic-regexps))
  (dolist (pair ros-topic-last-hz-rate)
    (let ((topic (car pair)))
      (when (string-match topic-regexp topic)
	(stop-hz-tracker topic)))))
  


(defun echo-ros-topic (topic)
  "Create a new buffer in which rostopic echo is done on the given topic (read interactively, with tab-completion)"
  (interactive (list (let ((word (current-word)))
		       (completing-read
			(if word
			(format "Enter topic name (default %s): " word)
			"Enter topic name: ")
			topic-completor nil nil nil nil word))))
  (let* ((topic-full-name (if (string-match "^/" topic) topic (concat "/" topic)))
	 (buffer-name (concat "*rostopic:" topic-full-name "*"))
	 (process (start-process buffer-name buffer-name "rostopic" "echo" topic-full-name)))
    (switch-to-buffer (process-buffer process))
    (ros-topic-echo-mode)
    ))

(defun ros-topic-info (topic)
  "Print info about topic, using rostopic list"
  (interactive (list (let ((word (current-word)))
		       (completing-read
			(if word
			 (format "Enter topic name (default %s): " word)
			 "Enter topic name: ")
			topic-completor nil nil nil nil word))))
  (let* ((topic-full-name (if (string-match "^/" topic) topic (concat "/" topic)))
	 (proc-name (format "*rostopic-list:%s" topic))
	 (buf (get-buffer-create proc-name)))
    (start-process proc-name buf "rostopic" "list" topic-full-name)
    (view-buffer-other-window buf)
    ))
    
    


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun interrupt-ros-topic-echo ()
  (interactive)
  (interrupt-process))

(defun buffer-process (b)
  (find-if (lambda (proc) (equal (process-buffer proc) b)) (process-list)))

(defun kill-current-buffer ()
  (interactive)
  (let ((process (buffer-process (current-buffer))))
    (when (and process (eq (process-status process) 'run))
      (interrupt-process)))
  (kill-buffer nil))

(defvar ros-topic-echo-keymap (make-sparse-keymap))
(define-key ros-topic-echo-keymap "k" 'interrupt-ros-topic-echo)
(define-key ros-topic-echo-keymap "q" 'kill-current-buffer)

(define-minor-mode ros-topic-echo-mode 
  "Mode used for rostopic echo.  

k kills the process (sends SIGINT).
q kills the buffer and process"
  :init-value nil
  :lighter " ros-topic-echo"
  :keymap ros-topic-echo-keymap
  (message "ros-topic-echo-mode: k to stop, q to quit"))


(defvar ros-topic-list-keymap (make-sparse-keymap))
(define-key ros-topic-list-keymap [?q] 'kill-current-buffer)
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



(defun ros-update-topic-list-internal ()
  (lwarn '(rosemacs) :debug "calling rostopic list")
  (let* ((b (generate-new-buffer "rostopic"))
	 (p (start-process "rostopic" b "rostopic" "list" "-v")))
    (set-process-sentinel p 'schedule-parse-rostopic-list)))



(defun update-ros-topic-buffer ()
  "Use the current value of ros-topic related variables to reset the contents of the *ros-topics* buffer, if it's visible"
  (when (and ros-topic-buffer (get-buffer-window ros-topic-buffer))
    (if (equal (current-buffer) ros-topic-buffer)
	(let ((old-point (point)))
	  (update-ros-topic-buffer-helper)
	  (goto-char (min (point-max) old-point)))
      (save-excursion
	(update-ros-topic-buffer-helper)))))

(defun ros-topic-get-stamp-string ()
  (goto-char (point-min))
  (let ((pos (re-search-forward "^\\(Last updated.*\\)$" nil t)))
    (if pos
	(match-string 1)
      "Last updated: <>")))

(defun update-ros-topic-buffer-helper ()
  (set-buffer ros-topic-buffer)
  (let ((old-stamp (ros-topic-get-stamp-string)))
    (erase-buffer)
    (princ (format "Master uri: %s\n" (getenv "ROS_MASTER_URI")) ros-topic-buffer)
    (princ old-stamp ros-topic-buffer))
 ;; (princ (format "Topics updated every %s seconds\n" ros-topic-update-interval) ros-topic-buffer)

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



(defun schedule-parse-rostopic-list (process event)
  "Use the output of rostopic list to recompute the list of published topics, and add and remove new/removed topics.  Kill the buffer of the rostopic process at the end."
  (lwarn '(rosemacs) :debug "rostopic list returned.")
  (progn
    (sit-for (round (* .75 (or ros-topic-update-interval 0.0))))
    (when ros-topic-timer (cancel-timer ros-topic-timer))
    (setq ros-topic-timer (run-with-idle-timer (+ .1 (round (* .25 (or ros-topic-update-interval 0.0)))) nil 'parse-rostopic-list process event))))


(defun get-topics (start end h)
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


(defun parse-rostopic-list (process event)
  (lwarn '(rosemacs) :debug "Parsing rostopic list")
  (unwind-protect
      (save-excursion
	(unless (active-minibuffer-window)
	  (set-buffer (process-buffer process))
	  (goto-char (point-min))
	  (let ((pub-start (re-search-forward "Published topics:" nil t))
		(sub-start (or (re-search-forward "Subscribed topics:" nil t) (point-max))))
	    (if (and pub-start sub-start)
		(let ((new-published-topics (get-topics pub-start sub-start ros-num-publishers)))
		  (setq ros-subscribed-topics (get-topics sub-start (point-max) ros-num-subscribers))
		  (destructuring-bind (added deleted) (rosemacs-list-diffs ros-topics new-published-topics)
		    (lwarn '(rosemacs) :debug "added topics : %s" added)
		    (dolist (topic added)
		      (add-ros-topic topic))
		    (dolist (topic deleted)
		      (remove-ros-topic topic))))
	      (lwarn '(rosemacs) :debug "rostopic output did not look as expected")))))
    

    (lwarn '(rosemacs) :debug "Done parsing rostopic list")
    (setq ros-all-topics 
	  (sort* (remove-duplicates (vconcat ros-topics ros-subscribed-topics) :test 'equal) 'string<))

    ;; update display
    (save-excursion
      (when ros-topic-buffer
	(set-buffer ros-topic-buffer)
	(let ((time-stamp-pattern "5/^Last updated: <%02H:%02M:%02S"))
	  (time-stamp))))

    ;; Start the next round of topic updates
    (when ros-topic-update-interval
      (ros-update-topic-list-internal))
 
    (kill-buffer (process-buffer process))))



(defun remove-ros-topic (topic)
  "Remove this topic and all associated entries from topic list, completion list, hertz processes, publication rates"
  (message "removing ros topic %s" topic)
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; rosrun
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defvar ros-run-temp-var "")
(defvar ros-run-exec-names nil)

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
  
(defun complete-exec-name (str)
  (rosemacs-bsearch str ros-run-exec-names))

(defun ros-package-path (pkg)
  (save-excursion
   (with-temp-buffer
     (call-process "rospack" nil t nil "find" pkg)
     (goto-char (point-min))
     (re-search-forward "^\\(.*\\)$")
     (match-string 1))))

(define-minor-mode ros-run-mode
  "Mode used for rosrun

k kills the process (sends SIGINT).
q kills the buffer and process."
  :init-value nil
  :lighter " ros-run"
  :keymap ros-topic-echo-keymap
  (message "ros-run mode: k to stop, q to quit"))

(defun ros-run (pkg exec &rest args)
  "pkg is a ros package name and exec is the executable name.  Tab completes package name.  Exec defaults to package name itself."
  (interactive (list (setq ros-run-temp-var (completing-read "Enter package: " ros-package-completor))
		     (let ((ros-run-exec-names (ros-find-executables ros-run-temp-var)))
		       (completing-read (format "Enter executable (default %s): " ros-run-temp-var) (dynamic-completion-table 'complete-exec-name)
					nil nil nil nil ros-run-temp-var))))
  (let* ((name (format "*rosrun:%s/%s" pkg exec))
	 (buf (generate-new-buffer name)))
    (apply #'start-process name buf "rosrun" pkg exec args)
    (save-excursion
      (set-buffer buf)
      (ros-run-mode))
    buf))





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
(define-key ros-keymap "r" 'ros-run)
(define-key ros-keymap "\C-r" 'ros-load-package-locations)
(define-key ros-keymap "\C-u" 'set-ros-topic-update-interval)
(define-key ros-keymap "u" 'ros-update-topic-list)
(define-key ros-keymap "\C-c" 'ros-core)
(define-key ros-keymap "\C-t" 'display-ros-topic-info)
(define-key ros-keymap "t" 'echo-ros-topic)
(define-key ros-keymap "h" 'add-hz-update)
(define-key ros-keymap "H" 'remove-hz-update)
(define-key ros-keymap "T" 'ros-topic-info)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Invoking the mode
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun set-rosemacs-shell-hooks ()
  (add-hook 'comint-input-filter-functions 'ros-directory-tracker nil t)
  (add-hook 'comint-dynamic-complete-functions 'comint-dynamic-complete-ros-package nil t)
  (add-hook 'comint-dynamic-complete-functions 'comint-dynamic-complete-ros-topic nil t))


(defun invoke-rosemacs ()
  (interactive)
  (add-hook 'shell-mode-hook 'set-rosemacs-shell-hooks)
  (set-ros-topic-update-interval 0)
  (run-at-time t ros-topic-display-update-interval 'update-ros-topic-buffer))




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

    
(provide 'rosemacs)