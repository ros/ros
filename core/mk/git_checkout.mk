# This file can be used to automate cloning and patching of 3rd 
# party software.
#
# Before including this file, e.g.:
#	include $(shell rospack find mk)/git_checkout.mk
# define the following make variables:
#	GIT_DIR: name of directory into which you want to clone to 
# 	GIT_URL: full URL to download
#	GIT_PATCH: your (list of) patch file(s) to patch the downloaded software
#	GIT_REVISION: -
$(GIT_DIR):
	git clone $(GIT_URL) $(GIT_DIR)
	cd $(GIT_DIR) && git checkout $(GIT_REVISION)
	touch rospack_nosubdirs

patched: $(GIT_DIR) $(GIT_PATCH) Makefile
	cd $(GIT_DIR) && git reset --hard
ifneq ($(strip $(GIT_PATCH)),)
	$(foreach PATCH, $(GIT_PATCH), patch -d $(GIT_DIR) -p1 < $(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched
endif   

download: $(GIT_DIR) patched
