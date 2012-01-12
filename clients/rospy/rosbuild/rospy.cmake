rosbuild_find_ros_package(rospy)

# Message-generation support.
macro(genmsg_py)
  rosbuild_get_msgs(_msglist)
  set(_inlist "")
  set(_autogen "")
  set(genmsg_py_exe ${rospy_PACKAGE_PATH}/rosbuild/scripts/genmsg_py.py)

  foreach(_msg ${_msglist})
    # Construct the path to the .msg file
    set(_input ${PROJECT_SOURCE_DIR}/msg/${_msg})
    # Append it to a list, which we'll pass back to gensrv below
    list(APPEND _inlist ${_input})
  
    rosbuild_gendeps(${PROJECT_NAME} ${_msg})
  

    set(_output_py ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}/msg/_${_msg})
    string(REPLACE ".msg" ".py" _output_py ${_output_py})
  
    # Add the rule to build the .py from the .msg.
    add_custom_command(OUTPUT ${_output_py} 
                       COMMAND ${genmsg_py_exe} --noinitpy ${_input}
                       DEPENDS ${_input} ${genmsg_py_exe} ${gendeps_exe} ${${PROJECT_NAME}_${_msg}_GENDEPS} ${ROS_MANIFEST_LIST})
    list(APPEND _autogen ${_output_py})
  endforeach(_msg)

  if(_autogen)
    # Set up to create the __init__.py file that will import the .py
    # files created by the above loop.  It can't run until those files are
    # generated, so it depends on them.
    set(_output_py ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}/msg/__init__.py)
    add_custom_command(OUTPUT ${_output_py}
                       COMMAND ${genmsg_py_exe} --initpy ${_inlist}
                       DEPENDS ${_autogen})

    # A target that depends on generation of the __init__.py
    add_custom_target(ROSBUILD_genmsg_py DEPENDS ${_output_py})
    # Make our target depend on rosbuild_premsgsrvgen, to allow any
    # pre-msg/srv generation steps to be done first.
    add_dependencies(ROSBUILD_genmsg_py rosbuild_premsgsrvgen)
    # Add our target to the top-level genmsg target, which will be fired if
    # the user calls genmsg()
    add_dependencies(rospack_genmsg ROSBUILD_genmsg_py)

    # Also set up to clean the src/<project>/msg directory
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}/msg)
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)
endmacro(genmsg_py)

# Call the macro we just defined.
genmsg_py()

# Service-generation support.
macro(gensrv_py)
  rosbuild_get_srvs(_srvlist)
  set(_inlist "")
  set(_autogen "")
  set(gensrv_py_exe ${rospy_PACKAGE_PATH}/rosbuild/scripts/gensrv_py.py)

  foreach(_srv ${_srvlist})
    # Construct the path to the .srv file
    set(_input ${PROJECT_SOURCE_DIR}/srv/${_srv})
    # Append it to a list, which we'll pass back to gensrv below
    list(APPEND _inlist ${_input})
  
    rosbuild_gendeps(${PROJECT_NAME} ${_srv})
  

    set(_output_py ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}/srv/_${_srv})
    string(REPLACE ".srv" ".py" _output_py ${_output_py})
  
    # Add the rule to build the .py from the .srv
    add_custom_command(OUTPUT ${_output_py} 
                       COMMAND ${gensrv_py_exe} --noinitpy ${_input}
                       DEPENDS ${_input} ${gensrv_py_exe} ${gendeps_exe} ${${PROJECT_NAME}_${_srv}_GENDEPS} ${ROS_MANIFEST_LIST})
    list(APPEND _autogen ${_output_py})
  endforeach(_srv)

  if(_autogen)
    # Set up to create the __init__.py file that will import the .py
    # files created by the above loop.  It can't run until those files are
    # generated, so it depends on them.
    set(_output_py ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}/srv/__init__.py)
    add_custom_command(OUTPUT ${_output_py}
                       COMMAND ${gensrv_py_exe} --initpy ${_inlist}
                       DEPENDS ${_autogen})
  
    # A target that depends on generation of the __init__.py
    add_custom_target(ROSBUILD_gensrv_py DEPENDS ${_output_py})
    # Make our target depend on rosbuild_premsgsrvgen, to allow any
    # pre-msg/srv generation steps to be done first.
    add_dependencies(ROSBUILD_gensrv_py rosbuild_premsgsrvgen)
    # Add our target to the top-level gensrv target, which will be fired if
    # the user calls gensrv()
    add_dependencies(rospack_gensrv ROSBUILD_gensrv_py)

    # Also set up to clean the src/<project>/srv directory
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}/srv)
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)
endmacro(gensrv_py)

# Call the macro we just defined.
gensrv_py()

