
ifeq ($(strip $(SVN_CMDLINE)),)
SVN_CMDLINE = svn
endif

$(SVN_DIR):
	$(SVN_CMDLINE) co $(SVN_REVISION) $(SVN_URL) $(SVN_DIR)
ifneq ($(strip $(SVN_PATCH)),)
	$(foreach patch,$(SVN_PATCH), patch -d $(SVN_DIR) -p0 < $(patch);)
endif
	-if test -z "$(SVN_REVISION)" -o "x$(SVN_REVISION)" != "x-r `svn info $(SVN_DIR) | grep Revision | cut -d " " -f 2,2`"; then \
	  cd $(SVN_DIR) && $(SVN_CMDLINE) up $(SVN_REVISION); \
        fi
	touch rospack_nosubdirs
	touch patched

SVN_UP: $(SVN_DIR)

# Note that 'svn revert' can't use the --non-interactive option, so we
# invoke 'svn' directly, instead of calling $(SVN_CMDLINE)
patched: $(SVN_PATCH) Makefile
ifneq ($(strip $(SVN_PATCH)),)
	svn revert -R $(SVN_DIR)
endif
	-cd $(SVN_DIR) && $(SVN_CMDLINE) up $(SVN_REVISION)
	$(foreach PATCH, $(SVN_PATCH), patch -d $(SVN_DIR) -p0 < $(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched

SVN_UP_REVERT_PATCH: $(SVN_DIR) patched
	
download: SVN_UP
