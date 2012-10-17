# Figure out which languages we're building for.  "rospack langs" will
# return a list of packages that:
#   - depend directly on roslang
#   - are not in the env var ROS_LANG_DISABLE
rosbuild_invoke_rospack("" _roslang LANGS langs)
separate_arguments(_roslang_LANGS)

# Iterate over the languages, retrieving any exported cmake fragment from
# each one.
set(_cmake_fragments)
foreach(_l ${_roslang_LANGS})
  # Get the roslang attributes from this package.

  # cmake
  rosbuild_invoke_rospack(${_l} ${_l} CMAKE export --lang=roslang --attrib=cmake)
  if(${_l}_CMAKE)
    foreach(_f ${${_l}_CMAKE})
      list(APPEND _cmake_fragments ${_f})
    endforeach(_f)
  endif(${_l}_CMAKE)
endforeach(_l)

# Hack to resolve languages from wet which rospack does not support yet
find_package(catkin QUIET COMPONENTS genmsg)
if(genmsg_FOUND)
  foreach(_l ${CATKIN_MESSAGE_GENERATORS})
    string(REPLACE "gen" "ros" _l ${_l})
    rosbuild_find_ros_package(${_l})
    if(${_l}_PACKAGE_PATH)
      set(_cmake_fragment ${${_l}_PACKAGE_PATH}/rosbuild/${_l}.cmake)
      if(EXISTS ${_cmake_fragment})
        list(APPEND _cmake_fragments ${_cmake_fragment})
      endif()
    endif(${_l}_PACKAGE_PATH)
  endforeach(_l)
endif()

# Now include them all
foreach(_f ${_cmake_fragments})
  if(NOT EXISTS ${_f})
    message(FATAL_ERROR "Cannot include non-existent exported cmake file ${_f}")
  endif(NOT EXISTS ${_f})
  # Include this cmake fragment; presumably it will do /
  # provide something useful.  Only include each file once (a file
  # might be multiply referenced because of package dependencies
  # dependencies).
  if(NOT ${_f}_INCLUDED)
    message("[rosbuild] Including ${_f}")
    include(${_f})
    set(${_f}_INCLUDED Y)
  endif(NOT ${_f}_INCLUDED)
endforeach(_f)
