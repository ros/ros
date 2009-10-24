cmake_minimum_required(VERSION 2.6)
# this makefile slightly hacked up to fit inside the rosdep project
# see http://code.google.com/p/yaml-cpp for the original distribution

project (YAML_CPP)

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "-O2 -Wall -pedantic -Wextra ${CMAKE_CXX_FLAGS}")
endif(CMAKE_COMPILER_IS_GNUCC)

set(YAML_CPP_VERSION_MAJOR "0")
set(YAML_CPP_VERSION_MINOR "2")
set(YAML_CPP_VERSION_PATCH "3")
set(YAML_CPP_VERSION "${YAML_CPP_VERSION_MAJOR}.${YAML_CPP_VERSION_MINOR}.${YAML_CPP_VERSION_PATCH}")

#enable_testing()

option(YAML_CPP_BUILD_TOOLS "Enables or disables yaml-reader and parse tools" true)

set(INCLUDE_INSTALL_DIR include/yaml-cpp)
set(LIB_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../bin)

file(GLOB public_headers include/*.h)
file(GLOB private_headers src/*.h)
file(GLOB sources src/*.cpp)

include_directories(${YAML_CPP_SOURCE_DIR}/include)

add_library(yaml-cpp
	${LIB_TYPE}
	${public_headers}
	${private_headers}
	${sources}
)
set_target_properties(yaml-cpp PROPERTIES
	VERSION "${YAML_CPP_VERSION}"
	SOVERSION "${YAML_CPP_VERSION_MAJOR}.${YAML_CPP_VERSION_MINOR}"
)

