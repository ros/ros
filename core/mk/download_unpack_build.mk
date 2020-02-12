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

# This target is pretty much vestigial, and is only here to support the
# download target.  The intended use it to depend on the unpacked file,
# below, which repeats the download and check logic.
$(TARBALL):
	-mkdir -p build
ifneq ($(strip $(MD5SUM_FILE)),)
	if [ ! -f $(MD5SUM_FILE) ]; then echo "Error: Couldn't find md5sum file $(MD5SUM_FILE)" && false; fi
	$(ROS_ROOT)/core/rosbuild/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL) `awk {'print $$1'} $(MD5SUM_FILE)`
else
	$(ROS_ROOT)/core/rosbuild/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL)
endif
	touch -c $(TARBALL)

download: $(TARBALL)

$(SOURCE_DIR)/unpacked: $(TARBALL_PATCH)
	-mkdir -p build
ifneq ($(strip $(MD5SUM_FILE)),)
	if [ ! -f $(MD5SUM_FILE) ]; then echo "Error: Couldn't find md5sum file $(MD5SUM_FILE)" && false; fi
	$(ROS_ROOT)/core/rosbuild/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL) `awk {'print $$1'} $(MD5SUM_FILE)`
else
	$(ROS_ROOT)/core/rosbuild/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL)
endif
	touch -c $(TARBALL)
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
	$(foreach patch,$(TARBALL_PATCH), patch -d $(SOURCE_DIR) -p0 < $(patch);)
endif
	touch $(SOURCE_DIR)/unpacked
