# The all target does the heavy lifting, creating the build directory and
# invoking CMake
all:
	@mkdir -p build
	-mkdir -p bin
	cd build && cmake -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake ..
	cd build && make $(ROS_PARALLEL_JOBS)

# The clean target blows everything away
clean:
	-cd build && make clean
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
