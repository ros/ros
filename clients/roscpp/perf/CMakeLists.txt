find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Can't call add_message_files() here because CMake context is lost when we leave this subdir.
# add_message_files(
#   DIRECTORY msg
#   FILES
#   LatencyMessage.msg
#   ThroughputMessage.msg
# )
# Can't call generate_messages() here, because currently can only have one instance of it per project.
# This means this file must be included before generate_messages() in the parent.

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}_perf EXCLUDE_FROM_ALL src/intra.cpp src/inter.cpp)
add_dependencies(${PROJECT_NAME}_perf ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_perf ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# These performance tests will not be built or executed automatically.
# TODO: automate them in some useful way.
add_executable(${PROJECT_NAME}-intra_suite EXCLUDE_FROM_ALL src/intra_suite.cpp)
target_link_libraries(${PROJECT_NAME}-intra_suite ${PROJECT_NAME}_perf roscpp)
