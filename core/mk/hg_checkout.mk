# This file can be used to automate cloning and patching of 3rd
# party software.
#
# Before including this file, e.g.:
#	include $(shell rospack find mk)/hg_checkout.mk
# define the following make variables:
#	HG_DIR: name of directory into which you want to clone to
# 	HG_URL: full URL to download
#	HG_PATCH: your (list of) patch file(s) to patch the downloaded software
#       HG_BRANCH: the branch of the repository to update
#	HG_REVISION: the revision to update

$(HG_DIR):
	mkdir -p $(HG_DIR)
	hg clone $(HG_URL) $(HG_DIR)
	cd $(HG_DIR) && hg update $(HG_BRANCH) && hg update $(HG_REVISION)
	touch rospack_nosubdirs

patched: $(HG_PATCH) Makefile
ifneq ($(strip $(HG_PATCH)),)
	cd $(HG_DIR) && hg revert --all
	$(foreach PATCH, $(HG_PATCH), patch -d $(HG_DIR) -p1 < $(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched
endif

download: $(HG_DIR) patched
