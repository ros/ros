# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=$(ROS_ROOT)/core/rosbuild/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

# The all target does the heavy lifting, creating the build directory and
# invoking CMake
all:
	@mkdir -p build
	cd build && cmake $(CMAKE_FLAGS) ..
	#cd build && make $(ROS_PARALLEL_JOBS)

# The clean target blows everything away
# It also removes auto-generated message/service code directories,
# to handle the case where the original .msg/.srv file has been removed,
# and thus CMake no longer knows about it.
clean:
	-cd build && make clean
	#rm -rf msg/cpp msg/lisp msg/oct msg/java srv/cpp srv/lisp srv/oct srv/java src/$(PACKAGE_NAME)/msg src/$(PACKAGE_NAME)/srv
	rm -rf build

# Build a source package.  Assumes that you're in a clean tree.
package_source: all
	$(shell rospack find rosbuild)/bin/package_source.py $(CURDIR)

# All other targets are just passed through
#test: all
#	if cd build && make -k $@; then make test-results; else make test-results && exit 1; fi
#tests: all
#	cd build && make $@
#test-future: all
#	cd build && make -k $@
#gcoverage: all
#	cd build && make $@

#include $(shell rospack find mk)/buildtest.mk
