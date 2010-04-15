all: wxswig

PKGVERSION = SWIG-1.3.29
TARBALL = build/$(PKGVERSION)-wx.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/$(PKGVERSION)-wx.tar.gz
SOURCE_DIR = build/SWIG-1.3.29
UNPACK_CMD = tar xzf
MD5SUM_FILE = md5.checksum
include $(shell rospack find mk)/download_unpack_build.mk

$(VERSION)/Makefile: $(VERSION)/Makefile.in $(VERSION)/configure $(VERSION)/configure.in
	cd $(VERSION) && \
		./configure --prefix=$(WXSWIG_ROOT)

wxswig: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --prefix=$(shell rospack find wxswig) && make $(ROS_PARALLEL_JOBS) && make install
	touch wxswig

clean:
	rm -rf bin share wxswig $(SOURCE_DIR)

wipe: clean
	rm -rf build
