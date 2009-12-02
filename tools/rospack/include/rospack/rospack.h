/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#ifndef ROSPACK_ROSPACK_H
#define ROSPACK_ROSPACK_H

/* Author: Morgan Quigley, Brian Gerkey */

/**
\mainpage
\htmlinclude manifest.html

\section overview Overview

\b %rospack is the ROS package management tool.  The %rospack package contains
a single binary, called \b %rospack.

 - Jump to \ref Usage "command-line usage".

%rospack is part dpkg, part pkg-config.  The main function of %rospack is
to crawl through the packages in ROS_ROOT and ROS_PACKAGE_PATH, read and
parse the \b manifest.xml for each package, and assemble a complete
dependency tree for all packages.

Using this tree, %rospack can answer a number of queries about packages and
their dependencies.  Common queries include:
 - find : return the absolute path to a package
 - depends : return a list of all of a package's dependencies
 - depends-on : return a list of packages that depend on the given package
 - export : return flags necessary for building and linking against a package

%rospack is intended to be cross-platform.

\subsection crawling Crawling algorithm
%rospack crawls in the following order: the directory ROS_ROOT, followed by
the colon-separated list of directories ROS_PACKAGE_PATH, in the order they
are listed.

During the crawl, %rospack examines the contents of each directory, looking
for a file called @b manifest.xml.  If such a file is found, the directory
containing it is considered to be a ROS package, with the package name
equal to the directory name.  The crawl does not descend further once a
manifest is found (i.e., packages cannot be nested inside one another).  

If a manifest.xml file is not found in a given directory, each subdirectory
is searched.  This subdirectory search is prevented if a file called @b
rospack_nosubdirs is found.  The directory itself is still searched for a
manifest, but its subdirectories are not crawled.

If multiple packages by the same name exist within the search path, the
first one found wins.  It is strongly recommended that you keep packages by
the same name in separate trees, each having its own element within
ROS_PACKAGE_PATH.  That way, you can deterministically control the search
order by the way that you specify ROS_PACKAGE_PATH.  The search order
within a given element of ROS_PACKAGE_PATH can be unpredictably affected by
the details of how files are laid out on disk.

\subsection efficiency Efficiency considerations
%rospack re-parses the manifest.xml files and rebuilds the dependency tree
on each execution.  However, it maintains a cache of package directories in
ROS_ROOT/.rospack_cache.  This cache is updated whenever there is a cache
miss, or when the cache is 60 seconds old.  You can change this timeout by
setting the environment variable ROS_CACHE_TIMEOUT, in seconds.  Set it to
0.0 to force a cache rebuild on every invocation of %rospack.

%rospack's performance can be adversely affected by the presence of very
broad and/or deep directory structures that don't contain manifest files.
If such directories are in %rospack's search path, it can spend a lot of
time crawling them only to discover that there are no packages to be found.
You can prevent this latency by creating a @b rospack_nosubdirs file in
such directories. If rospack seems to be running annoyingly slowly, you
can use the profile command, which will print out the 20 slowest trees
to crawl (or use profile --length=N to print the slowest N trees).

\subsection dependencies No dependencies
Because %rospack is the tool that determines dependencies, it cannot depend
on anything else.  Thus %rospack contains a copy of the TinyXML library,
instead of using the copy available in 3rdparty.  For the same reason, unit
tests for %rospack, which require gtest, are in a separate package, called
rospack_test.

\section codeapi Code API

%rospack is used entirely as a command-line tool.  While the main
functionality within %rospack is built as a library for testing purposes,
it is not intended for use in writing other applications.  Should this
change, the %rospack library API should be cleaned up and better
documented.

For now, the user-visible API is:
 - rospack::ROSPack::ROSPack()
 - rospack::ROSPack::run()

See main.cpp for example usage

\section rosapi ROS API
%rospack does not expose a ROS API.

\section commandline Command-line tools

\subsection rospack rospack

%rospack is the command-line tool that provides package management services.

%rospack crawls the directory ROS_ROOT and the colon-separated directories
in ROS_PACKAGE_PATH, determining a directory to be package if it contains a
file called @b manifest.xml.

\subsubsection Usage
\verbatim
$ rospack [options] <command> [package]
   Allowed commands:
     help
     find [package]
     list
     list-names
     langs
     depends [package] (alias: deps)
     depends1 [package] (alias: deps1)
     depends-manifests [package] (alias: deps-manifests)
     depends-indent [package] (alias: deps-indent)
     depends-why --target=<target> [package] (alias: deps-why)
     vcs [package]
     vcs0 [package]
     depends-on [package]
     depends-on1 [package]
     export [--deps-only] --lang=<lang> --attrib=<attrib> [package]
     cflags-only-I [--deps-only] [package]
     cflags-only-other [--deps-only] [package]
     libs-only-L [--deps-only] [package]
     libs-only-l [--deps-only] [package]
     libs-only-other [--deps-only] [package]
     profile [--length=<length>] [--zombie-only]
     plugins --attrib=<attrib> [--top=<toppkg>] [package]

If [package] is omitted, the current working directory
is used (if it contains a manifest.xml). 
\endverbatim

Subcommand detail.  All subcommands "return" their results by printing to
stdout.  Any errors or warnings go to stderr.  This separation ensures that
error output does not confuse programs that execute %rospack as a subprocess
to, for example, recover build flags for a package.

 - @b help : help message.

 - <b>find [package]</b> : absolute path to the package, empty string if package is not found
   - Used within many tools, including rosrun and roslaunch.

 - <b>list</b>  : newline-separated list "[package-name] [package-dir]" for all packages

 - <b>list-names</b>  : newline-separated list of "[package-name]" for all packages

 - <b>langs</b>  : space-separated list of available language-specific client libraries (e.g., "rospy roscpp")
   - Returns the list of packages that depend directly on the placeholder package @b roslang, minus any packages that are listed in the environment variable @b ROS_LANG_DISABLE.

 - <b>depends [package]</b>  : newline-separated ordered list of all dependencies of the package
   - Used within rosmake

 - <b>depends-manifests [package]</b>  : space-separated ordered list of manifest.xml files for all dependencies of the package
   - Used within rosbuild.cmake to create explicit dependencies from source files to other packages' manifests.

 - <b>depends-indent [package]</b>  : newline-separated presentation of the entire dependency chain for the package, indented to indicate where in the chain each dependency arises.  May contain duplicates.

 - <b>depends-why --target=TARGET [package]</b> : newline-separated presentation of all dependency chains from the package to TARGET.

 - <b>depends1 [package]</b>  : newline-separated ordered list of immediate dependencies of the package

 - <b>vcs [package]</b>  : newline-separated list of all [versioncontrol] tags from the manifest.xml of the package and all of its dependencies
   - Useful for pre-caching SSL certificates for https SVN servers that
     will be contacted during the build, to avoid having to manually accept the
     certificates during the build.

 - <b>vcs0 [package]</b>  : newline-separated list of all [versioncontrol] tags from the manifest.xml of just the package itself
   - Useful for pre-caching SSL certificates for https SVN servers that
     will be contacted during the build, to avoid having to manually accept the
     certificates during the build.

 - <b>depends-on [package]</b> : newline-separated list of all packages that depend on the package

 - <b>depends-on1 [package]</b> : newline-separated list of all packages that directly depend on the package

 - <b>export [--deps-only] --lang=LANGUAGE --attrib=ATTRIBUTE [package]</b> : Space-separated list of [export][LANGUAGE ATTRIBUTE=""/][/export] values from the manifest.xml of the package and its dependencies.  If --deps-only is provided, then the package itself is excluded.
   - Useful for getting language-specific build flags, e.g,. export/cpp/cflags.

 - <b>cflags-only-I [--deps-only] [package]</b> : Special case of export.  Space-separated list of export/cpp/cflags that start with @b -I.  If --deps-only is provided, then the package itself is excluded.
  - Used within rosbuild.cmake to assemble include paths for compiling.

 - <b>cflags-only-other [--deps-only] [package]</b> : Special case of export.  Space-separated list of export/cpp/cflags that don't start with @b -I.  If --deps-only is provided, then the package itself is excluded.
  - Used within rosbuild.cmake to assemble non-include compile flags.

 - <b>libs-only-L [--deps-only] [package]</b> : Special case of export.  Space-separated list of export/cpp/libs that start with @b -L.  If --deps-only is provided, then the package itself is excluded.
  - Used within rosbuild.cmake to assemble library search paths for linking.

 - <b>libs-only-l [--deps-only] [package]</b> : Special case of export.  Space-separated list of export/cpp/libs that start with @b -l.  If --deps-only is provided, then the package itself is excluded.
  - Used within rosbuild.cmake to assemble libraries for linking.

 - <b>libs-only-other [--deps-only] [package]</b> : Special case of export.  Space-separated list of export/cpp/libs that don't start with @b -l or @b -L.  If --deps-only is provided, then the package itself is excluded.
  - Used within rosbuild.cmake to assemble non-library link flags.

 - <b>profile [--length=N] [--zombie-only] </b> : Force a full crawl of package directories (i.e., don't use cache, and report to console on the N (default 20) directories that took the longest time to crawl.  Useful for finding stray directories that are adversely affecting rospack's performance.  If --zombie-only is given, then only the directories without any manifests in them are printed.  In this case, the output can be fed directly into rm to clean up your tree, e.g.: rospack profile --zombie-only | xargs rm -rf.  @b Note: be sure to check the output before deleting any files!

 - <b>plugins --attrib=<attrib> [--top=<toppkg>] [package] </b> : Examine packages that depend directly on the given package, extracting from each the value of an exported attribute with the name <attrib>.  All matching exports are returned, newline-separated.  E.g., if the manifest for a package "foo," located at /tmp/foo, contains:
\verbatim
<export>
  <rosbuild cmake="${prefix}/cmake/foo.cmake/>
</export>
\endverbatim
then "rospack plugins --attrib=cmake rosbuild" will return (at least):
\verbatim
/tmp/foo/cmake/foo.cmake
\endverbatim
If --top=<toppkg> is given, then in addition to depending directly on the given package, to be scanned for exports, a package must also be a dependency of <toppkg>, or be <toppkg> itself.

\par Examples

\verbatim
$ rospack find roscpp
/Users/gerkey/code/ros/core/roscpp
\endverbatim

\verbatim
$ rospack deps map_server
gtest
genmsg_cpp
roslib
xmlrpc++
rosthread
roscpp
std_msgs
std_srvs
sdl
ijg_libjpeg
sdl_image
\endverbatim

\verbatim
$ rospack depends-on1 libTF
pr2_controllers
grasp_learner
grasp_planner
object_detector
planning_models
gazebo_robot_description
rosTF
scan_utils
gazebo_plugin
robot_kinematics
tilting_laser
\endverbatim
*/

#include <string>
#include <vector>
#include <list>

#include "tinyxml-2.5.3/tinyxml.h"

namespace rospack
{

class Package;
// global helper functions
void string_split(const std::string &s, std::vector<std::string> &t, const std::string &d);
bool file_exists(const std::string &fname);
extern const char *fs_delim;
Package *g_get_pkg(const std::string &name);

typedef std::vector<Package *> VecPkg;
typedef std::list<Package*> Acc;
typedef std::list<Acc> AccList;

/**
 * The Package class contains information about a single package
 */
class Package
{
public:
  enum traversal_order_t { POSTORDER, PREORDER };
  std::string name, path;
  static std::vector<Package *> pkgs;
  static std::vector<Package *> deleted_pkgs;

  Package(std::string _path);
  static bool is_package(std::string path);
  static bool is_no_subdirs(std::string path);  
  const VecPkg &deps1();
  const VecPkg &deps(traversal_order_t order, int depth=0);
  std::string manifest_path();
  std::string flags(std::string lang, std::string attrib);
  std::string rosdep();
  std::string versioncontrol();
  std::vector<std::pair<std::string, std::string> > plugins();
  VecPkg descendants1();
  const std::vector<Package *> &descendants(int depth=0);
  TiXmlElement *manifest_root();
  void accumulate_deps(AccList& acc_list, Package* to);


private:
  bool deps_calculated, direct_deps_calculated, descendants_calculated;
  std::vector<Package *> _deps, _direct_deps, _descendants;
  TiXmlDocument manifest;
  bool manifest_loaded;

  Package(const Package &p) { } // just override the default public one
  
  bool has_parent(std::string pkg);
  const std::vector<Package *> &direct_deps(bool missing_pkg_as_warning=false);
  std::string direct_flags(std::string lang, std::string attrib);
  void load_manifest();
};

/**
 * The ROSPack class contains information the entire package dependency
 * tree.
 */
class ROSPack
{
public:
  static const char* usage();

  char *ros_root;
  
  ROSPack();

  ~ROSPack();

  Package *get_pkg(std::string pkgname);
  
  int cmd_depends_on(bool include_indirect);

  int cmd_depends_why();

  int cmd_find();

  int cmd_deps();

  int cmd_depsindent(Package* pkg, int indent);

  int cmd_deps_manifests();

  int cmd_deps1();

  /*
  int cmd_predeps(char **args, int args_len);
  */

  std::string snarf_libs(std::string flags, bool invert=false);
  std::string snarf_flags(std::string flags, std::string token, bool invert=false);

  int cmd_libs_only(std::string token);

  int cmd_cflags_only(std::string token);

  int cmd_make(char **args, int args_len);

  void export_flags(std::string pkg, std::string lang, std::string attrib);

  int cmd_versioncontrol(int depth);

  int cmd_rosdep(int depth);

  int cmd_export();

  int cmd_plugins();

  /** @brief The method that does the work.
   *
   * Call the run() method with argc and argv to crawl for packages, build
   * the tree, and answer the query in the command-line arguments. 
   *
   * @throws std::runtime_error
   */
  int run(int argc, char **argv);

  // Another form of run, which takes the arguments as a single string.
  // WARNING: this method does naive string-splitting on spaces.
  int run(const std::string& cmd);
  
  // Get the accumulated output
  std::string getOutput() { return output_acc; }

  int cmd_print_package_list(bool print_path);
  
  int cmd_print_langs_list();
  
  void crawl_for_packages(bool force_crawl = false);
  VecPkg partial_crawl(const std::string &path);

  // Exposed for testing purposes only
  std::string deduplicate_tokens(const std::string& s);

  // Storage for --foo options
  // --deps-only
  bool opt_deps_only;
  // --lang=
  std::string opt_lang;
  // --attrib=
  std::string opt_attrib;
  // --length=
  std::string opt_length;
  // --top=
  std::string opt_top;
  // The package name
  std::string opt_package;
  // --target=
  std::string opt_target;
  // the number of entries to list in the profile table
  int opt_profile_length;
  // only display zombie directories in profile?
  bool opt_profile_zombie_only;

private:
  bool cache_lock_failed;
  bool crawled;
  std::string getCachePath();
  /** tests if the cache exists, is new enough, and is valid */
  bool cache_is_good();
  /** returns a double representing the seconds since the Epoch */
  static double time_since_epoch();
  /** returns env[ROS_BINDEPS_PATH] if specified, otherwise /opt/ros */
  std::string getBinDepPath();
  /** returns true if env[ROS_BINDEPS_PATH] exists or /opt/ros exists */
  bool useBinDepPath();
  /** remove trailing slashes */
  void sanitize_rppvec(std::vector<std::string> &rppvec);
  // Output accumulates here
  std::string output_acc;
  // A place to store heap-allocated argv, in case we were passed a
  // std::string in run().  It'll be freed on destruction.
  int my_argc;
  char** my_argv;
  void freeArgv();
};

}

#endif
