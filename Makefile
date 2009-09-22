all:
	@if [ ! $(ROS_ROOT) ]; then echo "Please set ROS_ROOT first"; false; fi
	cd core/rosbuild && make
	cd tools/rospack && make
	@if test -z `which rospack`; then echo "Please add ROS_ROOT/bin to PATH"; false; fi
	cd tools/rosdep && make
	cd 3rdparty/gtest && make
	cd core/genmsg_cpp && make
	cd core/roslib && make
	cd core/rospy && make
	cd 3rdparty/pycrypto && make
	cd 3rdparty/paramiko && make
	cd 3rdparty/xmlrpc++ && make
	cd tools/roslaunch && make
	cd test/rostest && make
	cd core/rosconsole && make
	cd core/roscpp && make
	cd core/rosout && make
	@echo "\nHOORAY! The ROS tools are now built. Now, you can use 'rosmake' for\nrecursive builds.  For example, try\n    rosmake roscpp_tutorials\n"

clean:
	@if test -z `which rospack`; then echo "It appears that you have already done a 'make clean' because rospack is gone."; false; fi
	make -C core/genmsg_cpp clean
	make -C core/roslib clean
	make -C core/rospy clean
	make -C 3rdparty/pycrypto clean
	make -C 3rdparty/paramiko clean
	make -C 3rdparty/xmlrpc++ clean
	make -C core/rosconsole clean
	make -C core/roscpp clean
	make -C core/rosout clean
	make -C 3rdparty/gtest clean
	make -C tools/rosdep clean
	make -C test/rostest clean
	make -C tools/roslaunch clean
	make -C core/rosbuild clean
	rm -f `find . -name *.pyc`

clean-everything:
	cd core/rosbuild && make
	cd tools/rospack && make
	@for i in `rospack list-names` ; do  if [ $$i = rospack ] ; then continue; fi; echo "cleaning $$i"; cd `rospack find $$i` && make clean; done
	cd tools/rospack && make clean

wipe-everything:
	cd core/rosbuild && make
	cd tools/rospack && make
	@for i in `rospack list-names` ; do  if [ $$i = rospack ] ; then continue; fi; echo "wiping $$i"; cd `rospack find $$i` && make wipe; done
	rm -f rosmakeall-* stderr.txt stdout.txt rosmakeall-profile build-failure test-failure
	cd tools/rospack && make clean
	cd core/rosbuild && make clean

minimal:
	cd core/rosbuild && make
	cd tools/rospack && make
	cd tools/rosdep && make
	cd 3rdparty/gtest && make
	rosmake -v genmsg_cpp
