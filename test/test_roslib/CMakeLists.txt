project(test_roslib)
find_package(catkin)
add_pyunit(test/test_roslib.py)
add_pyunit(test/test_roslib_exceptions.py)
add_pyunit(test/test_roslib_manifest.py)
add_pyunit(test/test_roslib_manifestlib.py)
add_pyunit(test/test_roslib_names.py)
add_pyunit(test/test_roslib_packages.py)
add_pyunit(test/test_roslib_params.py)
add_pyunit(test/test_roslib_rosenv.py)
add_pyunit(test/test_roslib_rospack.py)
#add_pyunit(test/test_roslib_scriptutil.py)
add_pyunit(test/test_roslib_stacks.py)
add_pyunit(test/test_roslib_stack_manifest.py)
 
#integration tests

#rosbuild_add_boost_directories()
#rosbuild_add_gtest(test/utest test/utest.cpp)
#rosbuild_link_boost(test/utest thread)
#
#
#rosbuild_add_gtest(test_package test/package.cpp)
#target_link_libraries(test_package roslib)
