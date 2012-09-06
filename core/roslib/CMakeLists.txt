project(roslib)
find_package(catkin REQUIRED COMPONENTS rospack rosunit)
catkin_project(roslib
  LIBRARIES roslib
  INCLUDE_DIRS include
  )

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Avoid a boost warning that pops up when using msvc compiler
if(MSVC)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS)
endif()

add_library(roslib src/package.cpp)
target_link_libraries(roslib ${Boost_LIBRARIES} ${catkin_LIBRARIES})

if(NOT (APPLE OR WIN32 OR MINGW))
  target_link_libraries(roslib rt)
endif()


#explicitly install library and includes
install(TARGETS roslib
  ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PROJECT_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
install(PROGRAMS scripts/gendeps
  DESTINATION share/roslib/bin)

#integration tests

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIBRARY_DIRS})

catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
if(TARGET ${PROJECT_NAME}-utest)
  target_link_libraries(${PROJECT_NAME}-utest roslib ${Boost_LIBRARIES} ${catkin_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_package test/package.cpp)
if(TARGET ${PROJECT_NAME}-test_package)
  target_link_libraries(${PROJECT_NAME}-test_package roslib ${catkin_LIBRARIES})
endif()
