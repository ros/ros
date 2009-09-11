all: gtest

TARBALL = build/gtest-1.0.1.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/gtest-1.0.1.tar.gz
SOURCE_DIR = build/gtest-1.0.1
#MD5SUM_FILE = gtest-1.0.1.tar.gz.md5sum
include $(shell rospack find mk)/download_unpack_build.mk

gtest: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --prefix=$(CURDIR)/gtest
	cd $(SOURCE_DIR) && make install
	touch gtest

clean:
	rm -rf gtest $(SOURCE_DIR)

wipe: clean
	rm -rf build
