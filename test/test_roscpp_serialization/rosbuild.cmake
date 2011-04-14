include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
#set the default path for built executables to the "bin" directory
#set the default path for built libraries to the "lib" directory
include_directories(include)

rosbuild_add_gtest(serialization src/serialization.cpp)
rosbuild_add_gtest(generated_messages src/generated_messages.cpp)
rosbuild_add_executable(builtin_types src/builtin_types.cpp)
rosbuild_declare_test(builtin_types)
rosbuild_add_gtest_build_flags(builtin_types)
rosbuild_add_rostest(test/builtin_types.test)
rosbuild_add_executable(pre_deserialize src/pre_deserialize.cpp)
rosbuild_declare_test(pre_deserialize)
rosbuild_add_gtest_build_flags(pre_deserialize)
rosbuild_add_rostest(test/pre_deserialize.test)

