
minimal: core_tools
	rosmake gtest pycrypto paramiko roslaunch rosout rostest
	@echo "You have built the minimal set of ROS tools."
	@echo "If you want to make all ROS tools type 'rosmake ros'."
	@echo "Or you can rosmake any other package in your ROS_PACKAGE_PATH."

# enough for rosmake
core_tools:
	@if [ ! $(ROS_ROOT) ]; then echo "Please set ROS_ROOT first"; false; fi	
	make -C $(ROS_ROOT)/tools/rospack
	@if test -z `which rospack`; then echo "Please add ROS_ROOT/bin to PATH"; false; fi
	make -C $(ROS_ROOT)/tools/rosdep


clean:
	@if test -z `which rospack`; then echo "It appears that you have already done a 'make clean' because rospack is gone."; false; fi
	rosmake -r --target=clean ros
	make -C $(ROS_ROOT)/tools/rospack clean

## include $(shell rospack find mk)/cmake_stack.mk
### copied below since it can't be found before rospack is built 

# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
#CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=../core/rosbuild/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

# The all target does the heavy lifting, creating the build directory and
# invoking CMake
all_dist: core_tools
	@mkdir -p build
	-mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) ..

# The clean target blows everything away
# It also removes auto-generated message/service code directories, 
# to handle the case where the original .msg/.srv file has been removed,
# and thus CMake no longer knows about it.
clean_dist:
	-cd build && make clean
	rm -rf build

# Run the script that does the build, then do a fairly hacky cleanup, #1598
package_source: all_dist
	`rospack find rosbuild`/bin/makestackdist $(CURDIR)
	find build -mindepth 1 -not -name "*.bz2" | xargs rm -rf
	rm -rf bin
