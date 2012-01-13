project(message_filters)
catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include
  LIBRARIES message_filters
)
find_package(Boost COMPONENTS signals thread)
find_package(ROS COMPONENTS roscpp)

include_directories(include ${ROS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/connection.cpp)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

add_subdirectory(test)
