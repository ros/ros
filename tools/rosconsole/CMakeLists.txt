project(rosconsole)
catkin_project(rosconsole
  LIBRARIES rosconsole
  INCLUDE_DIRS include
  )

# See ticket: https://code.ros.org/trac/ros/ticket/3626
# On mac use g++-4.2
IF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")
  IF(EXISTS "/usr/bin/g++-4.2")
    set(CMAKE_CXX_COMPILER /usr/bin/g++-4.2)
  ELSE(EXISTS "/usr/bin/g++-4.2")
    # If there is no g++-4.2 use clang++
    set(CMAKE_CXX_COMPILER /usr/bin/clang++)
  ENDIF(EXISTS "/usr/bin/g++-4.2")
ENDIF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")

find_package(ROS COMPONENTS catkin cpp_common rostime rosunit)

include_directories(${ROS_INCLUDE_DIRS})

find_package(Boost COMPONENTS regex thread)

include_directories(include)

add_library(rosconsole src/rosconsole/rosconsole.cpp)

find_library(LOG4CXX_LIBRARY log4cxx)
if(NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
else()
  if(MINGW)
    # Assuming mingw_cross here: it is static and we need to add all of it's dependencies. 
    # Todo: we should use pkg-config to find these (write a better FindLog4cxx to do this jazz). 
    set(LOG4CXX_LIBRARIES "log4cxx;aprutil-1;iconv;expat;apr-1;rpcrt4;shell32;ws2_32;advapi32;kernel32;msvcrt") 
  else()  
    set(LOG4CXX_LIBRARIES ${LOG4CXX_LIBRARY}) 
  endif() 
endif()
target_link_libraries(rosconsole ${ROS_LIBRARIES} ${LOG4CXX_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS rosconsole
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})

if(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
  add_gtest(${PROJECT_NAME}-assertion_test test/assertion_test.cpp)
  target_link_libraries(${PROJECT_NAME}-assertion_test ${PROJECT_NAME})
endif()

add_gtest(${PROJECT_NAME}-thread_test test/thread_test.cpp)
target_link_libraries(${PROJECT_NAME}-thread_test ${PROJECT_NAME})
