rosbuild_find_ros_package(rosclj)
rosbuild_find_ros_package(rosjava)

set( _clj_classpath "" )
list(APPEND _clj_classpath ${rosclj_PACKAGE_PATH}/src)

macro(add_clojure_jars)
  # add all jars in CLJ_HOME env var to classpath.
  file(GLOB _clojure_jars "$ENV{CLJ_HOME}/*.jar")
  foreach(_jar ${_clojure_jars})
    list(APPEND _clj_classpath ${_jar})
  endforeach(_jar)
#   message("clojure jars: ${_clj_classpath}")
endmacro(add_clojure_jars)

add_clojure_jars()    

macro(add_clj_source_dir _srcdir)
  add_deps_classpath()
  set(_targetname _java_compile_${JAVA_OUTPUT_DIR})
  string(REPLACE "/" "_" _targetname ${_targetname})  
  add_custom_target(${_targetname} ALL)
#  message("Classpath ${_java_classpath}")
  foreach(_cp ${_java_classpath})
    add_java_source_dir_internal(${_targetname} ${_cp})
  endforeach(_cp)
  list(APPEND _clj_classpath "${_srcdir}")
endmacro(add_clj_source_dir)

macro(rospack_add_clj_executable _exe_name _command)
  string(REPLACE ";" ":" _clj_classpath_param "${_clj_classpath}")
  add_custom_command(
    OUTPUT ${EXECUTABLE_OUTPUT_PATH}/${_exe_name}
    COMMAND ${rosjava_PACKAGE_PATH}/scripts/rosjava_gen_exe ${rosjava_PACKAGE_PATH}/bin:${JAVA_OUTPUT_DIR}:${_clj_classpath_param} "clojure.main" ${EXECUTABLE_OUTPUT_PATH}/${_exe_name} "${_command}")
  set(_targetname ${EXECUTABLE_OUTPUT_PATH}/${_exe_name})
  string(REPLACE "/" "_" _targetname ${_targetname})
  add_custom_target(${_targetname} ALL DEPENDS ${EXECUTABLE_OUTPUT_PATH}/${_exe_name})
endmacro(rospack_add_clj_executable)

macro(rospack_add_clj_repl _exe_name)
  string(REPLACE ";" ":" _clj_classpath_param "${_clj_classpath}")

  add_custom_command(
    OUTPUT ${EXECUTABLE_OUTPUT_PATH}/${_exe_name}
    COMMAND ${rosjava_PACKAGE_PATH}/scripts/rosjava_gen_exe ${rosjava_PACKAGE_PATH}/bin:${JAVA_OUTPUT_DIR}:${_clj_classpath_param} "clojure.main" ${EXECUTABLE_OUTPUT_PATH}/${_exe_name})
  set(_targetname ${EXECUTABLE_OUTPUT_PATH}/${_exe_name})
  string(REPLACE "/" "_" _targetname ${_targetname})
  add_custom_target(${_targetname} ALL DEPENDS ${EXECUTABLE_OUTPUT_PATH}/${_exe_name})
endmacro(rospack_add_clj_repl)