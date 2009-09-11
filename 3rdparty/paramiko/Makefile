all: paramiko

TARBALL = build/paramiko-1.7.4.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/paramiko-1.7.4.tar.gz
SOURCE_DIR = build/paramiko-1.7.4
MD5SUM_FILE = paramiko-1.7.4.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

paramiko: $(SOURCE_DIR)/unpacked
	mkdir -p src
	cd $(SOURCE_DIR) && python setup.py build 
	rm -rf src/paramiko
	mv $(SOURCE_DIR)/build/lib/paramiko src/paramiko
	touch paramiko
clean:
	-rm -rf src $(SOURCE_DIR) paramiko
wipe: clean
	-rm -rf build
