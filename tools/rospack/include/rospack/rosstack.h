/*
 * Copyright (C) 2009, Morgan Quigley, Brian Gerkey
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University nor the names of its
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

#ifndef ROSSTACK_ROSSTACK_H
#define ROSSTACK_ROSSTACK_H

/* Author: Morgan Quigley, Brian Gerkey */

/**
\mainpage
\htmlinclude manifest.html

\section overview Overview

\b %rosstack is a ROS dependency and distribution tool.  The %rosstack package contains
a single binary, called \b %rosstack.
*/

#include <string>
#include <vector>

#include "tinyxml-2.5.3/tinyxml.h"
#include "rospack/rospack.h"

namespace rosstack
{

class Stack;
// global helper functions
void string_split(const std::string &s, std::vector<std::string> &t, const std::string &d);
bool file_exists(const std::string &fname);
extern const char *fs_delim;
Stack *g_get_stack(const std::string &name);
typedef std::vector<Stack *> VecStack;

/**
 * The Stack class contains information about a single stack
 */
class Stack
{
public:
  enum traversal_order_t { POSTORDER, PREORDER };
  std::string name, path;
  static std::vector<Stack *> stacks;

  Stack(std::string _path);
  static bool is_stack(const std::string &path);
  static bool is_package(const std::string &path);
  static bool is_no_subdirs(const std::string &path);  
  const VecStack &deps1();
  const VecStack &deps(traversal_order_t order, int depth=0);
  std::string manifest_path();
  VecStack descendants1();
  const VecStack &descendants(int depth=0);
  TiXmlElement *manifest_root();

private:
  bool deps_calculated, direct_deps_calculated, descendants_calculated;
  VecStack _deps, _direct_deps, _descendants;
  TiXmlDocument manifest;
  bool manifest_loaded;

  Stack(const Stack &p) { } // just override the default public one
  
  bool has_parent(std::string stk);
  const VecStack &direct_deps(bool missing_pkg_as_warning=false);
  void load_manifest();
};

/**
 * The ROSStack class contains information the entire stack dependency
 * tree.
 */
class ROSStack
{
public:
  static const char* usage();
  char *ros_root;
  rospack::ROSPack rp;

  ROSStack();
  ~ROSStack();
  Stack *get_stack(const std::string &name);
  int cmd_depends_on(bool include_indirect);
  int cmd_find();
  int cmd_contains();
  int cmd_contains_path();
  int cmd_deps();
  int cmd_depsindent(Stack* stk, int indent);
  int cmd_deps_manifests();
  int cmd_deps1();

  /** @brief The method that does the work.
   *
   * Call the run() method with argc and argv to crawl for packages, build
   * the tree, and answer the query in the command-line arguments. 
   *
   * @throws std::runtime_error
   */
  int run(int argc, char **argv);

  int cmd_print_stack_list(bool print_path);
  int cmd_print_packages();
  
  void crawl_for_stacks(bool force_crawl = false);
  std::string lookup_owner(std::string pkg_name, bool just_owner_name);
  void deleteCache();
private:
  bool crawled;
  /** tests if the cache exists, is new enough, and is valid */
  void createROSHomeDirectory();
  std::string getCachePath();
  bool cache_is_good();
  /** returns a double representing the seconds since the Epoch */
  static double time_since_epoch();
};

}

#endif
