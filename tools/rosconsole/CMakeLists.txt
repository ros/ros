project(rosconsole)
find_package(catkin REQUIRED COMPONENTS cpp_common rostime rosunit)
catkin_project(rosconsole
  LIBRARIES rosconsole
  INCLUDE_DIRS include
  CFG_EXTRAS rosconsole-extras.cmake
  )

include(cmake/rosconsole-extras.cmake.in)

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

include_directories(${catkin_INCLUDE_DIRS})

find_package(Boost COMPONENTS regex thread)

include_directories(include)

add_library(rosconsole src/rosconsole/rosconsole.cpp)

find_package(Log4cxx QUIET)
if(NOT LOG4CXX_LIBRARIES)
  # backup plan, hope it is in the system path
  find_library(LOG4CXX_LIBRARIES log4cxx)
endif()
if(NOT LOG4CXX_LIBRARIES)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif()

target_link_libraries(rosconsole ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS rosconsole
  ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PROJECT_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
if(TARGET ${PROJECT_NAME}-utest)
  target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
  catkin_add_gtest(${PROJECT_NAME}-assertion_test test/assertion_test.cpp)
  if(TARGET ${PROJECT_NAME}-assertion_test)
    target_link_libraries(${PROJECT_NAME}-assertion_test ${PROJECT_NAME})
  endif()
endif()

catkin_add_gtest(${PROJECT_NAME}-thread_test test/thread_test.cpp)
if(TARGET ${PROJECT_NAME}-thread_test)
  target_link_libraries(${PROJECT_NAME}-thread_test ${PROJECT_NAME})
endif()
