all: gtest

TARBALL = build/gtest-1.0.1.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/gtest-1.0.1.tar.gz
SOURCE_DIR = build/gtest-1.0.1
MD5SUM_FILE = gtest-1.0.1.tar.gz.md5sum
include $(shell rospack find mk)/download_unpack_build.mk

# gtest's death test appears to hang when gtest is compiled with Bullseye's
# gcc wrapper.  So, if COVFILE is set (which indicates that we're doing a
# coverage build), then we heuristically modify the PATH to get at the
# real gcc.
ifneq ($(strip $(COVFILE)),)
NEWPATH = /usr/bin:$(PATH)
else
NEWPATH = $(PATH)
endif

gtest: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && PATH=$(NEWPATH) ./configure --prefix=$(CURDIR)/gtest
	cd $(SOURCE_DIR) && PATH=$(NEWPATH) make install
	touch gtest

clean:
	rm -rf gtest $(SOURCE_DIR)

wipe: clean
	rm -rf build
