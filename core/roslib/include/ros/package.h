/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSLIB_PACKAGE_H
#define ROSLIB_PACKAGE_H

#include <string>
#include <utility>
#include <vector>
#include <map>

#if defined(__GNUC__)
#  define ROS_DEPRECATED __attribute__((deprecated))
#  define ROS_FORCE_INLINE __attribute__((always_inline))
#elif defined(_MSC_VER)
#  define ROS_DEPRECATED
#  define ROS_FORCE_INLINE __forceinline
#else
#  define ROS_DEPRECATED
#  define ROS_FORCE_INLINE inline
#endif

/*
  Windows import/export and gnu http://gcc.gnu.org/wiki/Visibility
  macros.
 */

#if defined(_MSC_VER)
#  define ROS_HELPER_IMPORT __declspec(dllimport)
#  define ROS_HELPER_EXPORT __declspec(dllexport)
#elif __GNUC__ >= 4
#  define ROS_HELPER_IMPORT __attribute__ ((visibility("default")))
#  define ROS_HELPER_EXPORT __attribute__ ((visibility("default")))
#else
#  define ROS_HELPER_IMPORT
#  define ROS_HELPER_EXPORT
#endif

// Ignore warnings about import/exports when deriving from std classes.
#ifdef _MSC_VER
#  pragma warning(disable: 4251)
#  pragma warning(disable: 4275)
#endif

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
#  ifdef roslib_EXPORTS // we are building a shared lib/dll
#    define ROSLIB_DECL ROS_HELPER_EXPORT
#  else // we are using shared lib/dll
#    define ROSLIB_DECL ROS_HELPER_IMPORT
#  endif
#else // ros is being built around static libraries
#  define ROSLIB_DECL
#endif

namespace ros
{
namespace package
{

typedef std::vector<std::string> V_string;
typedef std::map<std::string, std::string> M_string;

/**
 * \brief Runs a rospack command of the form 'rospack <cmd>', returning the output as a single string
 */
ROSLIB_DECL std::string command(const std::string& cmd);

/**
 * \brief Runs a rospack command of the form 'rospack <cmd>', returning the output as a vector of strings, split by newlines
 */
ROSLIB_DECL void command(const std::string& cmd, V_string& output);
/**
 * \brief Returns the fully-qualified path to a package, or an empty string if the package is not found
 */
ROSLIB_DECL std::string getPath(const std::string& package_name);
/**
 * \brief Gets a list of all packages.  Returns false if it could not run the command.
 */
ROSLIB_DECL bool getAll(V_string& packages);

/**
 * \brief Call the "rospack plugins" command, eg. "rospack plugins --attrib=<attribute> <package>".  Returns a vector of strings which
 * are export values
 */
ROSLIB_DECL void getPlugins(const std::string& package, const std::string& attribute, V_string& plugins, bool force_recrawl=false);

/**
 * \brief Call the "rospack plugins" command, eg. "rospack plugins --attrib=<attribute> <name>".
 * Return a vector of string pairs which are package names and exported values respectively.
 * Note that there can be multiple values for any single package.
 *
 * Note that while this uses the original rospack 'plugin' terminology,
 * this effectively works for any exported tag with attributes in the
 * catkin package.xml export list. Typical examples include:
 *
\code{.xml}
<export>
  <nav_core plugin="${prefix}/blp_plugin.xml" />  <!-- name="nav_core", attribute="plugin" -->
  <rosdoc config="rosdoc.yaml" />                 <!-- name="rosdoc",   attribute="config" -->
</export>
\endcode
 *
 * \param name : name of the package export tag (has to be a package name) [in]
 * \param attribute : name of the attribute inside the export tag with which to filter results [in]
 * \param exports : package/value export pairs resulting from the search [out]
 * \param force_recrawl : force rospack to rediscover everything on the system before running the search [in]
 */
ROSLIB_DECL void getPlugins(
  const std::string& name,
  const std::string& attribute,
  std::vector<std::pair<std::string, std::string> >& exports,
  bool force_recrawl=false
);

/**
 * \brief Call the "rospack plugins" command, eg. "rospack plugins --attrib=<attribute> <package>".
 * Return a map of package name to export value.
 *
 * \warning If there are multiple export values, only the last one is saved in the map.
 *
 * \deprecated Prefer the ::getPlugins(const std::string&, const std::string&, std::vector<std::pair<std::string, std::string>>&, bool) api instead.
 */
ROS_DEPRECATED ROSLIB_DECL void getPlugins(const std::string& package, const std::string& attribute, M_string& plugins, bool force_recrawl=false);

} // namespace package
} // namespace ros

#endif
