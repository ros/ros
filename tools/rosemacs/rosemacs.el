;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Installation instructions
;; 1. Put this file somewhere (if it's not already in rostools/rosemacs in your
;;    ros tree)
;; 2. (Optional) From emacs do M-x byte-compile followed by this file's full path 
;; 3. Add the following lines to ~/.emacs:
;;    (require 'rosemacs)
;;    (invoke-rosemacs)
;; 4. (Optional) Add the following line or equivalent to ~/.emacs to have
;;    quick access to added commands:
;;    (global-set-key "\C-x\C-r" ros-keymap)
;; 5. Start emacs from a shell in which the standard ROS variables are set, and
;;    the variable EMACSLOADPATH is set to include the location of this file
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Usage
;; 1. Directory tracking and tab completion for rosbash commands should now work 
;;    correctly in shell mode
;; 2. The commands find-ros-file, find-ros-message, and find-ros-service
;;    for navigating the ros libraries are available.  Tab completion should 
;;    work for all of them.
;; 3. Also, if the ros-keymap was bound to some prefix in step 4 above, the 
;;    above commands are accessible with key sequences starting with that 
;;    prefix (press control-h after the prefix to see the list).  For example,
;;    \C-x\C-r\C-f would call find-ros-file
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(require 'shell)
(require 'cl)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; State
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar ros-packages nil "Vector of ros packages")
(defvar ros-package-locations nil "Vector of directories containing the items in ros-packages")
(defvar ros-messages nil "Vector of ros messages")
(defvar ros-message-packages nil "Vector of packages corresponding to each ros message")
(defvar ros-services nil "Vector of ros services")
(defvar ros-service-packages nil "Vector of packages corresponding to each service")



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
	      ros-package-locations (map 'vector #'cdr package-alist)))
      (message "Done loading ROS package info"))))

(defun ros-messages-in-package (dir ext)
  "Return list of files in subdirectory ext/ of dir whose extension is .ext"
  (with-temp-buffer
    (let ((l nil)
	  (done nil)
	  (p nil))
      (call-process "ls" nil t nil (concat dir "/" ext "/"))
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
	(dolist (m (ros-messages-in-package dir ext))
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
  (lookup-vectors package ros-packages ros-package-locations))

(defun ros-message-package (m)
  (unless ros-message-packages
    (cache-ros-message-locations))
  (lookup-vectors m ros-messages ros-message-packages))

(defun ros-service-package (m)
  (unless ros-service-packages
    (cache-ros-service-locations))
  (lookup-vectors m ros-services ros-service-packages))


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

(setq message-completor (dynamic-completion-table (lambda (str) (unless ros-messages (cache-ros-message-locations)) (bsearch-completions str ros-messages))))
(setq service-completor (dynamic-completion-table (lambda (str) (unless ros-services (cache-ros-service-locations)) (bsearch-completions str ros-services))))
(setq package-completor 
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
	     (bsearch-completions package ros-packages))))))


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
	    (comint-dynamic-simple-complete prefix (all-completions prefix package-completor))
	    (skip-syntax-backward " ")))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Navigation commands
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun find-ros-file (package-name &optional dont-reload)
  "Open up the directory corresponding to PACKAGE-NAME in dired mode.  If used interactively, tab completion will work."
  (interactive (list (completing-read "Enter ros path: " package-completor) nil))
  (multiple-value-bind (package dir-prefix dir-suffix) (parse-ros-file-prefix package-name)
    (let* ((package-dir (ros-package-dir package))
	   (path (if dir-prefix (concat package-dir dir-prefix dir-suffix) package-dir)))
      (if path
	  (find-file path)
	(if dont-reload
	    (error "Did not find %s in the ros package list." package-name)
	  (progn
	    (message "Did not find %s.  Reloading ros package list and trying again..." package-name)
	    (ros-load-package-locations)
	    (find-ros-file package-name t)))))))


(defun find-ros-message (message)
  "Open definition of a ros message.  If used interactively, tab completion will work."
  (interactive (list (completing-read "Enter message name: " message-completor)))
  (let ((p (ros-message-package message)))
    (if p
	(let ((dir (ros-package-dir p)))
	  (if dir
	      (find-file (concat dir "/msg/" message ".msg"))
	    (error "Could not find directory corresponding to package %s" p)))
      (error "Could not find packag for message %s" message))))

(defun find-ros-service (service)
  "Open definition of a ros service.  If used interactively, tab completion will work."
  (interactive (list (completing-read "Enter service name: " service-completor)))
  (let ((p (ros-service-package service)))
    (if p
	(let ((dir (ros-package-dir p)))
	  (if dir
	      (find-file (concat dir "/srv/" service ".srv"))
	    (error "Could not find directory corresponding to package %s" p)))
      (error "Could not find package for service %s" service))))


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
      
      (cond ((string-match "^roscd\\([[:space]]\\|$\\)" cmd)
	     (if (string-match "\\([^/]*\\)/\\(.*\\)" arg1)
		 (let ((package (match-string 1 arg1))
		       (subdir (match-string 2 arg1)))
		   (message "Package is %s, subdir is %s" package subdir)
		   (let ((dir (ros-package-dir package)))
		     (if dir
			 (shell-process-cd (concat dir "/" subdir))
		       (message "Unable to find directory of ros package %s." arg1))))
	       (let ((dir (ros-package-dir arg1)))
		 (if dir
		     (shell-process-cd dir)
		   (message "Unable to find directory of ros package %s." arg1))))))
      ;; TODO add pushd

      (setq start (progn (string-match shell-command-separator-regexp str end)
			 (match-end 0))))))



(defun ros-emacs-last-word ()
  (let ((end (point)))
    (skip-syntax-backward "w_.()")
    (buffer-substring-no-properties (point) end)))
  
(defvar *ros-commands-starting-with-package* '("roscd" "rosmake" "rosrun" "rospushd"))

(defun comint-get-ros-package-prefix ()
  (save-excursion
    (block match-block
      (let ((arg (ros-emacs-last-word)))
	(skip-syntax-backward " ")
	(dolist (cmd *ros-commands-starting-with-package* nil)
	  (when (string-equal cmd (buffer-substring-no-properties (- (point) (length cmd)) (point)))
	    (return-from match-block arg)))))))

      
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Keymap
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar ros-keymap (make-sparse-keymap))
(define-key ros-keymap "\C-f" 'find-ros-file)
(define-key ros-keymap "f" 'find-ros-file)
(define-key ros-keymap "\C-m" 'find-ros-message)
(define-key ros-keymap "m" 'find-ros-message)
(define-key ros-keymap "\C-s" 'find-ros-service)
(define-key ros-keymap "s" 'find-ros-service)
(define-key ros-keymap "\C-l" 'ros-load-package-locations)
(define-key ros-keymap "l" 'ros-load-package-locations)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Invoking the mode
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun set-rosemacs-shell-hooks ()
  (add-hook 'comint-input-filter-functions 'ros-directory-tracker nil t)
  (add-hook 'comint-dynamic-complete-functions 'comint-dynamic-complete-ros-package nil t))


(defun invoke-rosemacs ()
  (interactive)
  (add-hook 'shell-mode-hook 'set-rosemacs-shell-hooks))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun bsearch-completions (str completions)
  "str is a string, completions is a sorted vector of strings.  Return list of strings in completions that str is a prefix of."
  (unless (string< (aref completions (1- (length completions))) str)
    (let ((i 0)
	  (j (1- (length completions))))
      (while (< (1+ i) j)
	(let ((k (floor (+ i j) 2)))
	  (if (string< str (aref completions k))
	      (setq j k)
	    (setq i k))))

      (when (not (is-prefix str (aref completions i)))
	(incf i))
      ;; Postcondition: completions of str, if they exist, begin at i
      
      (let ((returned-completions nil))
	(while (and (< i (length completions)) (is-prefix str (aref completions i)))
	  (push (aref completions i) returned-completions)
	  (incf i))
	returned-completions))))

(defun is-prefix (str1 str2)
  (let ((m (length str1)))
    (eq t (compare-strings str1 0 m str2 0 m))))

(defun lookup-vectors (str v1 v2)
  (let ((i (position str v1 :test #'string-equal)))
    (when i
      (aref v2 i)))) 
    
(provide 'rosemacs)