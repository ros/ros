###############################################################################
# To-be-deprecated macros below

macro(rospack)
  #_rosbuild_warn("rospack(foo) is deprecated. Use rosbuild_init() instead")
  rosbuild_init()
endmacro(rospack)

macro(find_ros_package pkgname) 
  #_rosbuild_warn_deprecate_no_prefix(find_ros_package)
  rosbuild_find_ros_package(${ARGV})
endmacro(find_ros_package pkgname) 

macro(rospack_add_compile_flags)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_compile_flags)
  rosbuild_add_compile_flags(${ARGV})
endmacro(rospack_add_compile_flags)

macro(rospack_remove_compile_flags)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_remove_compile_flags)
  rosbuild_remove_compile_flags(${ARGV})
endmacro(rospack_remove_compile_flags)

macro(rospack_add_link_flags)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_link_flags)
  rosbuild_add_link_flags(${ARGV})
endmacro(rospack_add_link_flags)

macro(rospack_remove_link_flags)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_remove_link_flags)
  rosbuild_remove_link_flags(${ARGV})
endmacro(rospack_remove_link_flags)

macro(rospack_add_executable)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_executable)
  rosbuild_add_executable(${ARGV})
endmacro(rospack_add_executable)

macro(rospack_add_library)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_library)
  rosbuild_add_library(${ARGV})
endmacro(rospack_add_library)

macro(rospack_add_gtest_build_flags)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_gtest_build_flags)
  rosbuild_add_gtest_build_flags(${ARGV})
endmacro(rospack_add_gtest_build_flags)

macro(rospack_declare_test)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_declare_test)
  rosbuild_declare_test(${ARGV})
endmacro(rospack_declare_test)

macro(rospack_add_gtest)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_gtest)
  rosbuild_add_gtest(${ARGV})
endmacro(rospack_add_gtest)

macro(rospack_add_gtest_future)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_gtest_future)
  rosbuild_add_gtest_future(${ARGV})
endmacro(rospack_add_gtest_future)

macro(rospack_add_rostest)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_rostest)
  rosbuild_add_rostest(${ARGV})
endmacro(rospack_add_rostest)

macro(rospack_add_rostest_future)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_rostest_future)
  rosbuild_add_rostest_future(${ARGV})
endmacro(rospack_add_rostest_future)

macro(rospack_add_rostest_graphical)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_rostest_graphical)
  rosbuild_add_rostest_graphical(${ARGV})
endmacro(rospack_add_rostest_graphical)

macro(rospack_add_pyunit)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_pyunit)
  rosbuild_add_pyunit(${ARGV})
endmacro(rospack_add_pyunit)

macro(rospack_add_pyunit_future)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_pyunit_future)
  rosbuild_add_pyunit_future(${ARGV})
endmacro(rospack_add_pyunit_future)

macro(rospack_add_pyunit_graphical)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_pyunit_graphical)
  rosbuild_add_pyunit_graphical(${ARGV})
endmacro(rospack_add_pyunit_graphical)

macro(get_msgs)
  #_rosbuild_warn_deprecate_no_prefix(get_msgs)
  rosbuild_get_msgs(${ARGV})
endmacro(get_msgs)

macro(get_srvs)
  #_rosbuild_warn_deprecate_no_prefix(get_srvs)
  rosbuild_get_srvs(${ARGV})
endmacro(get_srvs)

macro(gendeps)
  #_rosbuild_warn_deprecate_no_prefix(gendeps)
  rosbuild_gendeps(${ARGV})
endmacro(gendeps)

macro(gensrv)
  #_rosbuild_warn_deprecate_no_prefix(gensrv)
  rosbuild_gensrv(${ARGV})
endmacro(gensrv)

macro(genmsg)
  #_rosbuild_warn_deprecate_no_prefix(genmsg)
  rosbuild_genmsg(${ARGV})
endmacro(genmsg)

macro(rospack_add_boost_directories)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_boost_directories)
  rosbuild_add_boost_directories(${ARGV})
endmacro(rospack_add_boost_directories)

macro(rospack_link_boost)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_link_boost)
  rosbuild_link_boost(${ARGV})
endmacro(rospack_link_boost)

macro(rospack_download_test_data)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_download_test_data)
  rosbuild_download_test_data(${ARGV})
endmacro(rospack_download_test_data)

macro(rospack_add_openmp_flags)
  #_rosbuild_warn_deprecate_rospack_prefix(rospack_add_openmp_flags)
  rosbuild_add_openmp_flags(${ARGV})
endmacro(rospack_add_openmp_flags)

macro(_rospack_invoke)
  #_rosbuild_warn("_rospack_invoke() is deprecated; use rosbuild_invoke_rospack() instead")
  rosbuild_invoke_rospack(${ARGV})
endmacro(_rospack_invoke)

macro(rosbuild _project)
  _rosbuild_warn("rosbuild() is deprecated; use rosbuild_make_distribution() instead")
  rosbuild_make_distribution(${ARGN})
endmacro(rosbuild)

# To-be-deprecated macros above
###############################################################################

###############################################################################
# Deprecated macros below

# Deprecated macros above
###############################################################################

