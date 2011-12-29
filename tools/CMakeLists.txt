foreach(subdir
    topic_tools
    rosbag
    rosgraph
    )
  add_subdirectory(${subdir})
endforeach()
