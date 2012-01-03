project(test_roslib)
find_package(catkin)
find_package(Boost COMPONENTS thread)
find_package(ROS COMPONENTS roslib)

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

include_directories(${Boost_INCLUDE_DIRS} ${ROS_INCLUDE_DIRS})
add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${Boost_LIBRARIES} ${ROS_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_package test/package.cpp)
target_link_libraries(${PROJECT_NAME}-test_package ${ROS_LIBRARIES})
