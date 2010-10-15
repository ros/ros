# This was modified from the standard cmake.mk to perform and install step, and to do relinking as necessary on OS X

# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

# The all target does the heavy lifting, creating the build directory and
# invoking CMake
all:
	@mkdir -p build
	-mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE) $(ROS_PARALLEL_JOBS)
	cd build && $(MAKE) install
else
	cd build && make $(ROS_PARALLEL_JOBS)
	cd build && make install
endif
	@if test `uname` = Darwin ; then \
	echo "Changing installed rosrecord and rosplay shared library location." && \
	install_name_tool $(ROS_ROOT)/bin/rosrecord -change librosrecorder.dylib `rospack find rosrecord`/lib/librosrecorder.dylib && \
	install_name_tool $(ROS_ROOT)/bin/rosplay -change librosrecorder.dylib `rospack find rosrecord`/lib/librosrecorder.dylib; \
	fi

PACKAGE_NAME=$(shell basename $(PWD))

# The clean target blows everything away
# It also removes auto-generated message/service code directories, 
# to handle the case where the original .msg/.srv file has been removed,
# and thus CMake no longer knows about it.
clean:
	-cd build && make clean
	rm -rf msg/cpp msg/lisp msg/oct msg/java srv/cpp srv/lisp srv/oct srv/java src/$(PACKAGE_NAME)/msg src/$(PACKAGE_NAME)/srv
	rm -rf build
	rm -f .build-version

# All other targets are just passed through
test: all
	if cd build && make -k $@; then make test-results; else make test-results && exit 1; fi
tests: all
	cd build && make $@
test-future: all
	cd build && make -k $@
gcoverage: all
	cd build && make $@

include $(shell rospack find mk)/buildtest.mk
