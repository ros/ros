# This file can be used to automate cloning and patching of 3rd 
# party software.
#
# Before including this file, e.g.:
#	include $(shell rospack find mk)/bzr_checkout.mk
# define the following make variables:
#	BZR_DIR: name of directory into which you want to clone to 
# 	BZR_URL: full URL to download
#	BZR_PATCH: your (list of) patch file(s) to patch the downloaded software
#	BZR_REVISION: a bzr revisionspec string

ifneq ($(BZR_REVISION),)
  BZR_REV=-r $(BZR_REVISION)
else
  BZR_REV=
endif

$(BZR_DIR):
	bzr branch $(BZR_URL) $(BZR_REV) $(BZR_DIR)
	touch rospack_nosubdirs

patched: $(BZR_DIR) $(BZR_PATCH) Makefile
	cd $(BZR_DIR) && bzr revert $(BZR_REV)
ifneq ($(strip $(BZR_PATCH)),)
	$(foreach PATCH, $(BZR_PATCH), patch -d $(BZR_DIR) -p1 < $(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched
endif   

download: $(BZR_DIR) patched
