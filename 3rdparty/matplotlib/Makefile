all: matplotlib

TARBALL = build/matplotlib-0.98.2.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/matplotlib-0.98.2.tar.gz
SOURCE_DIR = build/matplotlib-0.98.2
MD5SUM_FILE = matplotlib-0.98.2.tar.gz.md5sum
include $(shell rospack find mk)/download_unpack_build.mk

matplotlib: $(SOURCE_DIR)/unpacked
	-rm -rf src
	mkdir -p src
	PYTHONPATH=`rospack find numpy`/src ; \
			   cd $(SOURCE_DIR) && \
			   python setup.py build && \
			   python setup.py install --prefix ../../src 
	mv src/lib/python2.*/site-packages/* src/ 
	rm -rf src/bin src/lib
	touch matplotlib

clean:
	rm -rf src matplotlib $(SOURCE_DIR)

wipe: clean
	-rm -rf build
