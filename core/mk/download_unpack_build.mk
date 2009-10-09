# This file can be used to automate downloading and unpacking of source
# distributions
#
# Before including this file, define the following make variables:
#
#  TARBALL: name of source distribution to be downloaded
#  TARBALL_URL: full URL (including the file itself) to download
#  SOURCE_DIR: name of directory into which TARBALL will unpack
#  UNPACK_CMD: command to use when unpacking (e.g., tar xzf)
#	 TARBALL_PATCH: patch files to apply to SOURCE_DIR
#  INITIAL_DIR: set this if the tarball unpacks to a different dir than SOURCE_DIR,
#								and you want the directory moved
#
# Optional variables:
#  MD5SUM_FILE: name of md5sum file to check before unpacking
#
# Because this file declares targets, you almost certainly want to declare
# your own 'all' target before including this file.  Otherwise, the first
# target declared here will become the default.

$(TARBALL):
	-mkdir build
	wget $(TARBALL_URL) -O $(TARBALL)
	touch -c $(TARBALL)

download: $(TARBALL)

$(SOURCE_DIR)/unpacked: $(TARBALL) $(TARBALL_PATCH)
ifneq ($(strip $(MD5SUM_FILE)),)
	mkdir -p build
	cd build; `rospack find mk`/rosmd5check ../$(MD5SUM_FILE)
endif
	rm -rf $(SOURCE_DIR) $(INITIAL_DIR)
ifneq ($(strip $(UNPACK_CMD)),)
	cd build; $(UNPACK_CMD) ../$(TARBALL)
else
	cd build; tar xzf ../$(TARBALL)
endif
ifneq ($(strip $(INITIAL_DIR)),)
	mv $(INITIAL_DIR) $(SOURCE_DIR)
endif
ifneq ($(strip $(TARBALL_PATCH)),)
	cd $(SOURCE_DIR) && $(foreach patch,$(TARBALL_PATCH), patch -p0 < ../../$(patch);)
endif
	touch $(SOURCE_DIR)/unpacked

