all: paramiko

TARBALL = build/paramiko-1.7.5.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/paramiko-1.7.5.tar.gz
SOURCE_DIR = build/paramiko-1.7.5
MD5SUM_FILE = paramiko-1.7.5.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

paramiko: $(SOURCE_DIR)/unpacked
	mkdir -p src
	cd $(SOURCE_DIR) && python setup.py build 
	rm -rf src
	mv `python find_pylib.py paramiko $(SOURCE_DIR)/build/` src
	touch paramiko
clean:
	-rm -rf src $(SOURCE_DIR) paramiko
wipe: clean
	-rm -rf build
