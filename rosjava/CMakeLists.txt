cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

# Build java code
# We have to use this to bootstrap ourselves, since 
# messages cannot build until rosjava is built.
add_deps_classpath()
set(_targetname _java_compile_${JAVA_OUTPUT_DIR})
string(REPLACE "/" "_" _targetname ${_targetname})  
add_custom_target(${_targetname} ALL)
add_java_source_dir_internal(${_targetname} ${PROJECT_SOURCE_DIR}/src )


# Build JNI wrapper for roscpp
#include(FindJNI) doesn't actually find Java in most cases

SET(JNI_INCLUDE_DIRECTORIES
  $ENV{JAVA_HOME}/include
  )

FIND_PATH(JAVA_INCLUDE_PATH jni.h 
  ${JNI_INCLUDE_DIRECTORIES}
)

FIND_PATH(JAVA_INCLUDE_PATH2 jni_md.h 
  ${JNI_INCLUDE_DIRECTORIES}
  ${JAVA_INCLUDE_PATH}/win32
  ${JAVA_INCLUDE_PATH}/linux
  ${JAVA_INCLUDE_PATH}/freebsd
)


rosbuild_add_library(rosjava rosjava.cpp)
rosbuild_add_compile_flags(rosjava "-I${JAVA_INCLUDE_PATH}/" "-I${JAVA_INCLUDE_PATH2}/" "-fpermissive")
#rosbuild_add_compile_flags(rosjava "-I/usr/lib/jvm/java-6-sun/include/" "-I/usr/lib/jvm/java-6-sun/include/linux/")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)



rosbuild_gensrv()
rosbuild_genmsg()



#add_subdirectory(test EXCLUDE_FROM_ALL)

