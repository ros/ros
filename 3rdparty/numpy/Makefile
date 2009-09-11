all: numpy

TARBALL = build/numpy-1.1.0.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/numpy-1.1.0.tar.gz
SOURCE_DIR = build/numpy-1.1.0
MD5SUM_FILE = numpy-1.1.0.tar.gz.md5sum
include $(shell rospack find mk)/download_unpack_build.mk

numpy: $(SOURCE_DIR)/unpacked
	mkdir -p src
	cd $(SOURCE_DIR) && python setup.py build --fcompiler=gnu95 && python setup.py install --prefix ../../src
	-rm -rf src/numpy
	mv src/lib/python2.*/site-packages/* src/
	rm -rf src/bin src/lib
	touch numpy
clean:
	rm -rf src numpy $(SOURCE_DIR)
wipe: clean
	-rm -rf build
