project(test_roscpp_serialization)
catkin_project(test_roscpp_serialization)

find_package(ROS COMPONENTS roscpp xmlrpcpp rosconsole rosunit rostest)
find_package(GTest REQUIRED)
include_directories(include ${ROS_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg
  FILES
  ArrayOfFixedLength.msg
  ArrayOfVariableLength.msg
  Constants.msg
  CustomHeader.msg
  EmbeddedExternal.msg
  EmbeddedFixedLength.msg
  EmbeddedVariableLength.msg
  FixedLengthArrayOfExternal.msg
  FixedLength.msg
  FixedLengthStringArray.msg
  HeaderNotFirstMember.msg
  VariableLengthArrayOfExternal.msg
  VariableLength.msg
  VariableLengthStringArray.msg
  WithDuration.msg
  WithHeader.msg
  WithMemberNamedHeaderThatIsNotAHeader.msg
  WithTime.msg
)
generate_messages(DEPENDENCIES std_msgs rosgraph_msgs)

add_gtest(${PROJECT_NAME}-serialization src/serialization.cpp)
target_link_libraries(${PROJECT_NAME}-serialization ${ROS_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(${PROJECT_NAME}-serialization test_roscpp_serialization_gencpp)
add_gtest(${PROJECT_NAME}-generated_messages src/generated_messages.cpp)
target_link_libraries(${PROJECT_NAME}-generated_messages ${ROS_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(${PROJECT_NAME}-generated_messages test_roscpp_serialization_gencpp)

add_executable(${PROJECT_NAME}-builtin_types EXCLUDE_FROM_ALL src/builtin_types.cpp)
add_dependencies(tests ${PROJECT_NAME}-builtin_types) 
target_link_libraries(${PROJECT_NAME}-builtin_types ${ROS_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(${PROJECT_NAME}-builtin_types test_roscpp_serialization_gencpp)
add_rostest(test/builtin_types.test)

add_executable(${PROJECT_NAME}-pre_deserialize EXCLUDE_FROM_ALL src/pre_deserialize.cpp)
add_dependencies(tests ${PROJECT_NAME}-pre_deserialize) 
target_link_libraries(${PROJECT_NAME}-pre_deserialize ${ROS_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(${PROJECT_NAME}-pre_deserialize test_roscpp_serialization_gencpp)
add_rostest(test/pre_deserialize.test)
