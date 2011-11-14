project(rosconsole)

find_package(catkin)

find_package(cpp_common)
include_directories(${cpp_common_INCLUDE_DIRS})

find_package(rostime)
include_directories(${rostime_INCLUDE_DIRS})

find_package(Boost COMPONENTS regex)

include_directories(include)

add_library(rosconsole SHARED src/rosconsole/rosconsole.cpp)

#todo use cmake stuff to add liblog4cxx 
target_link_libraries(rosconsole /usr/lib/liblog4cxx.so ${Boost_LIBRARIES})

install_cmake_infrastructure(rosconsole
  VERSION 0.0.1
  LIBRARIES rosconsole
  INCLUDE_DIRS include
  )
