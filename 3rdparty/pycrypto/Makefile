all: pycrypto

TARBALL = build/pycrypto-2.0.1.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/pycrypto-2.0.1.tar.gz
SOURCE_DIR = build/pycrypto-2.0.1
MD5SUM_FILE = pycrypto-2.0.1.tar.gz.md5sum
include $(shell rospack find mk)/download_unpack_build.mk

pycrypto: $(SOURCE_DIR)/unpacked
	-rm -rf src
	mkdir -p src
	cd $(SOURCE_DIR) && python setup.py build 
	mv $(SOURCE_DIR)/build/lib.*/Crypto src/Crypto
	touch pycrypto
clean:
	rm -rf src pycrypto $(SOURCE_DIR)
wipe: clean
	-rm -rf build
