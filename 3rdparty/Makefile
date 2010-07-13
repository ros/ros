# Quick and dirty Makefile to clean out 3rdparty packages prior to distro.
# Will be replaced by proper CMake infrastructure.

SUBDIRS = gtest \
          matplotlib \
          numpy \
          paramiko \
          pycrypto \
          pyyaml \
          svnmerge \
          system \
          tinyxml \
          wxswig \
          xmlrpc++

clean:
	-$(foreach d, $(SUBDIRS), make -C $(d) clean;)
	-$(foreach d, $(SUBDIRS), make -C $(d) wipe;)
