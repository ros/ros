set(genmsg_py_exe ${rospy_SOURCE_DIR}/scripts/rosbuild2/genmsg_py.py)

find_package(PythonInterp)
if (NOT PYTHONINTERP_FOUND)
  message(FATAL_ERROR "could not find python interpreter")
endif()

# Message-generation support.
macro(genmsg_py TYPE)
  set(_inlist "") # accumulator for __init__.py generation step
  set(_outdir ${ROSBUILD_GEN_DIR}/py/${PROJECT_NAME}/msg)
  set(_autogen "")
  install(DIRECTORY ${_outdir} DESTINATION python/${PROJECT_NAME} OPTIONAL COMPONENT ${PROJECT_NAME})

  foreach(_msg ${ARGN})
    # Construct the path to the .msg file
    if (${TYPE} STREQUAL "STATIC")
      set(_input ${PROJECT_SOURCE_DIR}/${_msg})
      rosbuild_assert_file_exists(${_input})
    elseif(${TYPE} STREQUAL "GENERATED")
      set(_input ${_msg})
    else()
      message(FATAL_ERROR "Unknown message type \"${TYPE}\" (must be either STATIC or GENERATED)")
    endif()

    get_filename_component(_fname ${_msg} NAME)

    string(REPLACE ".msg" ".py" _output_py_base ${_fname})

    set(_output_py ${_outdir}/_${_output_py_base})

    file(MAKE_DIRECTORY ${_outdir})

    set(_incflags "")
    foreach(dir ${DEPENDED_PACKAGE_PATHS})
      list(APPEND _incflags -I${dir})
    endforeach()

    # Add the rule to build the .py from the .msg.
    add_custom_command(OUTPUT ${_output_py} 
      COMMAND ${ROSBUILD_SUBSHELL}
      ${PYTHON_EXECUTABLE} ${genmsg_py_exe} 
      ${_input}
      -o ${_outdir}
      -p ${PROJECT_NAME}
      -I ${CMAKE_CURRENT_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR}
      ${_incflags} 
      DEPENDS ${_input} ${genmsg_py_exe} ${gendeps_exe} 
      ${${PROJECT_NAME}_${_msg}_GENDEPS} ${ROS_MANIFEST_LIST}
      COMMENT "${PROJECT_NAME}: generating msg/_${_output_py_base}")

    list(APPEND ${PROJECT_NAME}_generated_py ${_output_py})
    list(APPEND _autogen ${_output_py})
    list(APPEND _inlist ${_input})
  endforeach(_msg)

  if(_autogen)
    # Set up to create the __init__.py file that will import the .py
    # files created by the above loop.  It can't run until those files are
    # generated, so it depends on them.
    set(_output_py ${_outdir}/__init__.py)
    add_custom_command(OUTPUT ${_output_py}
      COMMAND 
      ${ROSBUILD_SUBSHELL} 
      python ${genmsg_py_exe} 
      --initpy 
      -p ${PROJECT_NAME}
      -s ${CMAKE_CURRENT_SOURCE_DIR}/src
      -o ${_outdir}
      ${_inlist}
      DEPENDS ${_inlist} ${_autogen}
      COMMENT "${PROJECT_NAME}: generating msg/__init__.py")
    list(APPEND ${PROJECT_NAME}_generated_py ${_output_py})
  endif()
endmacro()

set(gensrv_py_exe ${rospy_SOURCE_DIR}/scripts/rosbuild2/gensrv_py.py)
# Service-generation support.
macro(gensrv_py TYPE)
  set(_inlist "")
  set(_autogen "")
  set(_outdir ${ROSBUILD_GEN_DIR}/py/${PROJECT_NAME}/srv)
  install(DIRECTORY ${_outdir} DESTINATION python/${PROJECT_NAME} OPTIONAL COMPONENT ${PROJECT_NAME})

  foreach(_srv ${ARGN})
    if (${TYPE} STREQUAL "STATIC")
      set(_input ${PROJECT_SOURCE_DIR}/${_srv})
    elseif(${TYPE} STREQUAL "GENERATED")
      set(_input ${CMAKE_CURRENT_BINARY_DIR}/${_srv})
    else()
      message(FATAL_ERROR "Unknown message type \"${TYPE}\" (must be either STATIC or GENERATED)")
    endif()

    get_filename_component(_fname ${_srv} NAME)

    string(REPLACE ".srv" ".py" _output_py_base ${_fname})

    set(_output_py ${_outdir}/_${_output_py_base})
    list(APPEND ${PROJECT_NAME}_generated_py ${_output_py})
    
    set(_incflags "")
    foreach(dir ${DEPENDED_PACKAGE_PATHS})
      list(APPEND _incflags -I${dir})
    endforeach()

    # Add the rule to build the .py from the .srv
    add_custom_command(OUTPUT ${_output_py} 
      COMMAND ${ROSBUILD_SUBSHELL} ${PYTHON_EXECUTABLE} ${gensrv_py_exe} 
      ${_input}
      -p ${PROJECT_NAME}
      -s ${CMAKE_CURRENT_SOURCE_DIR}/src
      -o ${_outdir}
      -I ${CMAKE_CURRENT_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR}
      ${_incflags}
      DEPENDS ${_input} ${gensrv_py_exe} ${gendeps_exe} ${${PROJECT_NAME}_${_srv}_GENDEPS} ${ROS_MANIFEST_LIST}
      COMMENT "${PROJECT_NAME}: generating srv/_${_output_py_base}")
    list(APPEND _autogen ${_output_py})
    list(APPEND _inlist ${_input})
  endforeach(_srv)

  if(_autogen)
    # Set up to create the __init__.py file that will import the .py
    # files created by the above loop.  It can't run until those files are
    # generated, so it depends on them.
    set(_output_py ${_outdir}/__init__.py)
    add_custom_command(OUTPUT ${_output_py}
      COMMAND ${ROSBUILD_SUBSHELL} ${PYTHON_EXECUTABLE} ${gensrv_py_exe} --initpy 
      -p ${PROJECT_NAME}
      -o ${_outdir}
      ${_inlist}
      DEPENDS ${_inlist} ${${PROJECT_NAME}_generated_py}
      COMMENT "${PROJECT_NAME}: generating srv/__init__.py"
      )
    list(APPEND ${PROJECT_NAME}_generated_py ${_output_py})
  endif()
endmacro()

macro(gentargets_py)
  add_custom_target(${PROJECT_NAME}_codegen_py ALL
    DEPENDS ${${PROJECT_NAME}_generated_py})

  add_dependencies(codegen_py ${PROJECT_NAME}_codegen_py)
  add_dependencies(${PROJECT_NAME}_codegen ${PROJECT_NAME}_codegen_py)
endmacro()
