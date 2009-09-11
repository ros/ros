$(GIT_DIR):
	git clone $(GIT_URL) $(GIT_DIR)
	cd $(GIT_DIR) && git checkout $(GIT_REVISION)
	touch rospack_nosubdirs

patched: $(GIT_PATCH) Makefile
	cd $(GIT_DIR) && git reset --hard
ifneq ($(strip $(GIT_PATCH)),)
	$(foreach PATCH, $(GIT_PATCH), patch -d $(GIT_DIR) -p0 < $(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched
endif   

SVN_UP_REVERT_PATCH: $(SVN_DIR) patched

download: $(GIT_DIR) patched
