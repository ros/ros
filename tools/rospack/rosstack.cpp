/*
 * Copyright (C) 2009, Morgan Quigley and Brian Gerkey
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

/* Author: Morgan Quigley, Brian Gerkey */

#include <cstdlib>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <string>
#include <vector>
#include <stack>
#include <queue>
#include <cassert>
#include <unistd.h>
#include <dirent.h>
#include <stdexcept>
#include <sys/time.h>
#include <sys/file.h>
#include <time.h>
#include <sstream>
#include <iterator>
#include <stdint.h>

#include <libgen.h>

#include "tinyxml-2.5.3/tinyxml.h"
#include "rosstack.h"
#include "rospack/rospack.h"
using namespace std;

//#define VERBOSE_DEBUG
const double DEFAULT_MAX_CACHE_AGE = 60.0; // rebuild cache every minute

#include <sys/stat.h>
#ifndef S_ISDIR 
#define S_ISDIR(x) (((x) & S_IFMT) == S_IFDIR) 
#endif

using namespace rosstack;

#ifdef __APPLE__
const string g_ros_os("osx");
#else
const string g_ros_os("linux");
#endif

//////////////////////////////////////////////////////////////////////////////
// Global storage for --foo options
// --deps-only
bool g_deps_only;
// --length=
string g_length;
// The stack name
string g_stack, g_version;
// the number of entries to list in the profile table
uint32_t g_profile_length = 0;

//////////////////////////////////////////////////////////////////////////////

const char *rosstack::fs_delim = "/"; // ifdef this for windows someday

Stack::Stack(string _path) : path(_path), 
        deps_calculated(false), direct_deps_calculated(false),
        descendants_calculated(false), manifest_loaded(false)
{
  load_manifest();
}
bool Stack::is_stack(const string &path)
{
  return file_exists(path + string(fs_delim) + "stack.xml");
}
bool Stack::is_package(const string &path)
{
  return file_exists(path + string(fs_delim) + "manifest.xml");
}
bool Stack::is_no_subdirs(const string &path)
{
  return file_exists(path + string(fs_delim) + "rosstack_nosubdirs");
}
bool Stack::version_eq(const std::string &a, const std::string &b)
{
  if (a == b)
    return true;
  if (!a.length() || !b.length())
    return true;
  return false;
}
void Stack::split_name_ver(const string &combined, 
                           string &name, string &version)
{
  vector<string> tokens;
  string_split(combined, tokens, "-");
  if (tokens.size() == 1)
  {
    // no version given
    name = combined;
    version = "";
    return;
  }
  version = tokens.back();
  name = string(combined.begin(), combined.end() - version.length() - 1);
}
const VecStack &Stack::deps1()
{
  return direct_deps();
}
const VecStack &Stack::deps(traversal_order_t order, int depth)
{
  if (depth > 1000)
  {
    fprintf(stderr,"[rospack] woah! expanding the dependency tree made it blow "
                   "up.\n There must be a circular dependency somewhere.\n");
    throw runtime_error(string("circular dependency"));
  }
  if (deps_calculated)
    return _deps;
  // postorder traversal of the dependency tree
  VecStack my_dd = direct_deps();
  for (VecStack::iterator i = my_dd.begin(); i != my_dd.end(); ++i)
  {
    VecStack d = (*i)->deps(order, depth+1); // recurse on direct dependencies
    if (order == PREORDER)
      _deps.push_back(*i);
    for (VecStack::iterator j = d.begin(); j != d.end(); ++j)
    {
      // don't add things twice, but if you have something already
      // and we're doing a quasi-preorder traversal, bump it to the back
      bool have = false;
      VecStack::iterator prior_loc;
      for (VecStack::iterator k = _deps.begin(); k != _deps.end() && !have; ++k)
        if ((*k) == (*j))
        {
          prior_loc = k;
          have = true;
        }
      if (have && order == PREORDER)
      {
        _deps.erase(prior_loc);
        _deps.push_back(*j);
      }
      else if (!have)
        _deps.push_back(*j);
    }
    if (order == POSTORDER)
    {
      // only stuff it to the end if it isn't there already
      bool have = false;
      for (VecStack::iterator k = _deps.begin(); k != _deps.end() && !have; ++k)
        if ((*k) == (*i))
          have = true;
      if (!have)
        _deps.push_back(*i);
    }
  }
  deps_calculated = true;
  return _deps;
}
string Stack::manifest_path()
{
  return path + string(fs_delim) + "stack.xml";
}

VecStack Stack::descendants1()
{
  VecStack children;
  for (VecStack::iterator p = stacks.begin(); p != stacks.end(); ++p)
  {
    // We catch exceptions here, because we don't care if some 
    // unrelated packages in the system have invalid manifests
    try
    {
      if ((*p)->has_parent(name))
        children.push_back(*p);
    }
    catch (runtime_error &e)
    {
    }
  }
  return children;
}

const VecStack &Stack::descendants(int depth)
{
  if (depth > 100)
  {
    fprintf(stderr, "[rosstack] woah! circular dependency! aaaaaa!\n");
    throw runtime_error(string("circular dependency"));
  }
  if (descendants_calculated)
    return _descendants;
  VecStack desc_with_dups;
  for (VecStack::iterator p = stacks.begin(); p != stacks.end(); ++p)
  {
    // We catch exceptions here, because we don't care if some 
    // unrelated packages in the system have invalid manifests
    try
    {
      if ((*p)->has_parent(name))
      {
        desc_with_dups.push_back(*p);
        const VecStack &p_desc = (*p)->descendants(depth+1);
        for (VecStack::const_iterator q = p_desc.begin();
             q != p_desc.end(); ++q)
          desc_with_dups.push_back(*q);
      }
    }
    catch (runtime_error &e)
    {
    }
  }
  assert(_descendants.size() == 0);
  for (VecStack::iterator p = desc_with_dups.begin();
       p != desc_with_dups.end(); ++p)
  {
    bool found = false;
    for (VecStack::iterator q = _descendants.begin();
         q != _descendants.end() && !found; ++q)
      if ((*q)->name == (*p)->name)
        found = true;
    if (!found)
      _descendants.push_back(*p);
  }
  descendants_calculated = true;
  return _descendants;
}


bool Stack::has_parent(string pkg)
{
  VecStack parents = direct_deps(true);
  for (VecStack::iterator i = parents.begin(); i != parents.end(); ++i)
    if ((*i)->name == pkg)
      return true;
  return false;
}

const VecStack &Stack::direct_deps(bool missing_stack_as_warning)
{
  if (direct_deps_calculated)
    return _direct_deps;
#ifdef VERBOSE_DEBUG
  printf("calculating direct deps for package [%s]\n", name.c_str());
#endif
  TiXmlElement *mroot = manifest_root();
  TiXmlNode *dep_node = 0;
  while ((dep_node = mroot->IterateChildren(string("depend"), dep_node)))
  {
    TiXmlElement *dep_ele = dep_node->ToElement();
    assert(dep_ele);
    const char *dep_stackname = dep_ele->Attribute("stack");
    if (!dep_stackname)
    {
      fprintf(stderr,"[rosstack] bad depend syntax (no 'stack' attribute) in "
              "[%s]\n", manifest_path().c_str());
      throw runtime_error(string("invalid manifest"));
    }
    const char *dep_version = dep_ele->Attribute("version");
    if (!dep_version)
    {
      fprintf(stderr,"[rosstack] bad depend syntax (no 'version' attribute) in "
              "[%s]\n", manifest_path().c_str());
      throw runtime_error(string("invalid manifest"));
    }
    // Must make a copy here, because the call to g_get_stack() below might
    // cause a recrawl, which blows aways the accumulated data structure.
    char* dep_stackname_copy = strdup(dep_stackname);
#ifdef VERBOSE_DEBUG
    printf("direct_deps: stk %s has dep %s %s\n",
           name.c_str(), dep_stackname_copy, dep_version);
#endif 
    try
    {
      _direct_deps.push_back(g_get_stack(dep_stackname_copy, dep_version));
    }
    catch (runtime_error &e)
    {
      if (missing_stack_as_warning)
        fprintf(stderr, "[rosstack] warning: couldn't find dependency [%s] of [%s]\n",
                dep_stackname_copy, name.c_str());
      else
      {
        fprintf(stderr, "[rosstack] couldn't find dependency [%s] of [%s]\n",
                dep_stackname_copy, name.c_str());
        free(dep_stackname_copy);
        throw runtime_error(string("missing dependency"));
      }
    }
    free(dep_stackname_copy);
  }
  direct_deps_calculated = true;
  return _direct_deps;
}

void Stack::load_manifest()
{
  if (manifest_loaded)
    return;
  if (!manifest.LoadFile(manifest_path()))
  {
    string errmsg = string("error parsing manifest file at [") + manifest_path().c_str() + string("]");
    fprintf(stderr, "[rosstack] warning: error parsing manifest file at [%s]\n",
            manifest_path().c_str());
    // Only want this warning printed once.
    manifest_loaded = true;
    throw runtime_error(errmsg);
  }
  // find the version attribute, and enforce that it doesn't conflict with the
  // version encoded in the stack path
  TiXmlElement *mroot = manifest.RootElement();
  const char *ver_attr = mroot->Attribute("version");
  if (!ver_attr)
    throw runtime_error(string("no version given in the ") + name + 
                        string(" stack manifest"));
  //printf("%s ver: %s\n", path.c_str(), ver_attr);
  const char *name_attr = mroot->Attribute("name");
  if (!name_attr)
  {
    //mroot->Print(stdout, 2);//(//FILE *)0);
    throw runtime_error(string("no name attribute in the manifest at ") + path);
  }
  name = string(name_attr);
  version = string(ver_attr);
}

TiXmlElement *Stack::manifest_root()
{
  load_manifest();
  TiXmlElement *ele = manifest.RootElement();
  if (!ele)
  {
    string errmsg = string("error parsing manifest file at [") + manifest_path().c_str() + string("]");
    throw runtime_error(errmsg);
  }
  return ele;
}

VecStack Stack::stacks;

//////////////////////////////////////////////////////////////////////////////

ROSStack *g_rosstack = NULL; // singleton

ROSStack::ROSStack() : ros_root(NULL), crawled(false)
{
  g_rosstack = this;
  Stack::stacks.reserve(500); // get some space to avoid early recopying...
  ros_root = getenv("ROS_ROOT");
  if (!ros_root)
  {
    fprintf(stderr,"[rosstack] ROS_ROOT is not defined in the environment.\n");
    throw runtime_error(string("no ROS_ROOT"));
  }
  if (!file_exists(ros_root))
  {
    fprintf(stderr,"[rosstack] the path specified as ROS_ROOT is not " 
                   "accessible. Please ensure that this environment variable "
                   "is set and is writeable by your user account.\n");
    throw runtime_error(string("no ROS_ROOT"));
  }

  createROSHomeDirectory();

  crawl_for_stacks();
}

ROSStack::~ROSStack() 
{ 
  for (VecStack::iterator p = Stack::stacks.begin(); 
       p != Stack::stacks.end(); ++p)
    delete (*p);
  Stack::stacks.clear();
}

const char* ROSStack::usage()
{
  return "USAGE: rosstack [options] <command> [stack]\n"
          "  Allowed commands:\n"
          "    help\n"
          "    find [stack]\n"
          "    contents [stack]\n"
          "    env [stack]\n"
          "    list\n"
          "    list-names\n"
          "    depends [stack] (alias: deps)\n"
          "    depends-manifests [stack] (alias: deps-manifests)\n"
          "    depends1 [stack] (alias: deps1)\n"
          "    depends-indent [stack] (alias: deps-indent)\n"
          "    depends-on [stack]\n"
          "    depends-on1 [stack]\n"
          "    profile [--length=<length>] \n\n"
          " If [stack] is omitted, the current working directory\n"
          " is used (if it contains a stack.xml).\n\n";
}

Stack *ROSStack::get_stack(const string &stack_name, const string &version)
{
  for (VecStack::iterator p = Stack::stacks.begin(); 
       p != Stack::stacks.end(); ++p)
    if ((*p)->name == stack_name && Stack::version_eq(version, (*p)->version))
      return (*p);
  if (!crawled) // maybe it's a brand-new stack. force a crawl.
  {
    crawl_for_stacks(true); // will set the crawled flag; recursion is safe
    return get_stack(stack_name, version);
  }
  throw runtime_error(string("couldn't find stack [") + stack_name + 
        string("-") + version + string("]"));
  return NULL; // or not
}
  
int ROSStack::cmd_depends_on(bool include_indirect)
{
  // Explicitly crawl for stacks, to ensure that we get newly added
  // dependent stacks.  We also avoid the possibility of a recrawl
  // happening within the loop below, which could invalidate the stacks
  // vector as we loop over it.
  crawl_for_stacks(true);

  Stack* s;
  try
  {
    s = get_stack(g_stack, g_version);
  }
  catch(runtime_error)
  {
    fprintf(stderr, "[rosstack] warning: stack %s doesn't exist\n", 
            g_stack.c_str());
    s = new Stack(g_stack);
    Stack::stacks.push_back(s);
  }
  assert(s);
  const VecStack descendants = include_indirect ? s->descendants() 
          : s->descendants1();
  for (VecStack::const_iterator sit = descendants.begin(); 
       sit != descendants.end(); ++sit)
    printf("%s\n", (*sit)->name.c_str());
  return 0;
}

int ROSStack::cmd_find()
{
  // todo: obey the search order
  Stack *p = get_stack(g_stack, g_version);
  printf("%s\n", p->path.c_str());
  return 0;
}

int ROSStack::cmd_env()
{
  Stack *s = get_stack(g_stack, g_version);
  VecStack deps = s->deps(Stack::POSTORDER);
  deps.push_back(s);
  // find the version of ROS, and be sure there is only one
  Stack *ros = NULL;
  for (VecStack::iterator i = deps.begin(); i != deps.end() && !ros; ++i)
    if ((*i)->name == "ros")
      ros = *i;
  // check for duplicates. could implement this more efficiently if we cared.
  for (VecStack::iterator i = deps.begin(); i != deps.end() - 1; ++i)
    for (VecStack::iterator j = i + 1; j != deps.end(); j++)
      if ((*i)->name == (*j)->name && (*i)->version != (*j)->version)
        throw runtime_error(string("stack ") + (*i)->name + string(" with "
            "versions ") + (*i)->version + string(" and ") + (*j)->version +
            string("were both depended upon (possibly indirectly) by stack ") +
            g_stack);
  printf("export ROS_ROOT=%s\n", ros->path.c_str());
  printf("export PATH=$ROS_ROOT/bin:$PATH\n");
  printf("export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH\n");
  printf("export ROS_PACKAGE_PATH=");
  for (VecStack::iterator i = deps.begin(); i != deps.end(); ++i)
    printf("%s:", (*i)->path.c_str());
  printf("$ROS_PACKAGE_PATH\n");

  return 0;
}

int ROSStack::cmd_deps()
{
  VecStack d = get_stack(g_stack, g_version)->deps(Stack::POSTORDER);
  for (VecStack::iterator i = d.begin(); i != d.end(); ++i)
    printf("%s-%s\n", (*i)->name.c_str(), (*i)->version.c_str());
  return 0;
}

int ROSStack::cmd_deps_manifests()
{
  VecStack d = get_stack(g_stack, g_version)->deps(Stack::POSTORDER);
  for (VecStack::iterator i = d.begin(); i != d.end(); ++i)
    printf("%s/stack.xml ", (*i)->path.c_str());
  puts("");
  return 0;
}

int ROSStack::cmd_deps1()
{
  VecStack d = get_stack(g_stack, g_version)->deps1();
  for (VecStack::iterator i = d.begin(); i != d.end(); ++i)
    printf("%s-%s\n", (*i)->name.c_str(), (*i)->version.c_str());
  return 0;
}

int ROSStack::cmd_depsindent(Stack *stack, int indent)
{
  VecStack d = stack->deps1();
  for (VecStack::iterator i = d.begin(); i != d.end(); ++i)
  {
    for(int s=0; s<indent; s++)
      printf(" ");
    printf("%s-%s\n", (*i)->name.c_str(), (*i)->version.c_str());
    cmd_depsindent(*i, indent+2);
  }
  return 0;
}

static bool space(char c) { return isspace(c); }
static bool not_space(char c) { return !isspace(c); }
static vector<string> split_space(const string& str)
{
  typedef string::const_iterator iter;
  vector<string> ret;
  iter i = str.begin();
  while (i != str.end())
  {
    i = find_if(i, str.end(), not_space);
    iter j = find_if(i, str.end(), space);
    if (i != str.end())
      ret.push_back(string(i, j));
    i = j;
  }
  return ret;
}

int ROSStack::run(int argc, char **argv)
{
  assert(argc >= 2);
  int i;
  const char* opt_length  = "--length=";

  string errmsg = string(usage());

  i=1;
  const char* cmd = argv[i++];

  for(;i<argc;i++)
  {
    if(!strncmp(argv[i], opt_length, strlen(opt_length)))
    {
      if(strlen(argv[i]) > strlen(opt_length))
        g_length = string(argv[i]+strlen(opt_length));
      else
        throw runtime_error(errmsg);
    }
    else
      break;
  }
  
  if(strcmp(cmd, "profile") && g_length.size())
    throw runtime_error(errmsg);

  if(i < argc)
  {
    if(!strcmp(cmd, "help") ||
       !strcmp(cmd, "list") ||
       !strcmp(cmd, "list-names") ||
       !strcmp(cmd, "profile"))
      throw runtime_error(errmsg);

    Stack::split_name_ver(argv[i++], g_stack, g_version);
  }
  // Are we sitting in a stack?
  else if(Stack::is_stack("."))
  {
    char buf[1024];
    if(!getcwd(buf,sizeof(buf)))
      throw runtime_error(errmsg);
    g_stack = string(basename(buf));
  }

  if (i != argc)
    throw runtime_error(errmsg);

  if (!strcmp(cmd, "profile"))
  {
    if (g_length.size())
      g_profile_length = atoi(g_length.c_str());
    else
      g_profile_length = 20; // default is about a screenful or so
#ifdef VERBOSE_DEBUG
    printf("profile_length = %d\n", g_profile_length);
#endif
    // re-crawl with profiling enabled
    crawl_for_stacks(true);
    return 0;
  }
  else if (!strcmp(cmd, "find"))
    return cmd_find();
  else if (!strcmp(cmd, "list"))
    return cmd_print_stack_list(true);
  else if (!strcmp(cmd, "list-names"))
    return cmd_print_stack_list(false);
  else if (!strcmp(cmd, "contents"))
    return cmd_print_packages();
  else if (!strcmp(cmd, "env"))
    return cmd_env();
  else if (!strcmp(cmd, "depends") || !strcmp(cmd, "deps"))
    return cmd_deps();
  else if (!strcmp(cmd, "depends-manifests") || !strcmp(cmd, "deps-manifests"))
    return cmd_deps_manifests();
  else if (!strcmp(cmd, "depends1") || !strcmp(cmd, "deps1"))
    return cmd_deps1();
  else if (!strcmp(cmd, "depends-indent") || !strcmp(cmd, "deps-indent"))
    return cmd_depsindent(get_stack(g_stack, g_version), 0);
  else if (!strcmp(cmd, "depends-on"))
    return cmd_depends_on(true);
  else if (!strcmp(cmd, "depends-on1"))
    return cmd_depends_on(false);
  else if (!strcmp(cmd, "help"))
    fputs(usage(), stderr);
  else
    throw runtime_error(errmsg);
  return 0;
}

int ROSStack::cmd_print_stack_list(bool print_path)
{
  for (VecStack::iterator i = Stack::stacks.begin(); 
       i != Stack::stacks.end(); ++i)
    if (print_path)
      printf("%s-%s %s\n", (*i)->name.c_str(),
             (*i)->version.c_str(), (*i)->path.c_str());
    else
      printf("%s-%s\n", (*i)->name.c_str(), (*i)->version.c_str());
  return 0;
}

int ROSStack::cmd_print_packages()
{
  rospack::ROSPack rp;
  string path = get_stack(g_stack, g_version)->path;
  //printf("partial crawl of %s\n", path.c_str());
  rospack::VecPkg pkgs = rp.partial_crawl(path);
  //printf("found %d pkgs\n", pkgs.size());
  for (rospack::VecPkg::iterator i = pkgs.begin(); i != pkgs.end(); ++i)
  {
    printf("%s\n", (*i)->name.c_str());
    delete *i;
  }
}
  
void ROSStack::createROSHomeDirectory()
{
  char *homedir = getenv("HOME");
  if (!homedir) {
    //fprintf(stderr, "[rospack] WARNING: cannot create ~/.ros directory.\n");
  } 
  else 
  {
    string path = string(homedir) + "/.ros";
    if (access(path.c_str(), R_OK) && !mkdir(path.c_str(), 0700))
      fprintf(stderr,"[rosstack] WARNING: cannot create ~/.ros directory.\n");
  }
}

string ROSStack::getCachePath()
{
  string path;
  path = string(ros_root) + "/.rosstack_cache";
  if (access(ros_root, W_OK) == 0)
    return path;
  // if we cannot write into the ros_root, then let's try to
  // write into the user's .ros directory.
  createROSHomeDirectory();
  path = string(getenv("HOME")) + "/.ros/rosstack_cache";
  return path;
}

bool ROSStack::cache_is_good()
{
  string cache_path = getCachePath();
  // first see if it's new enough
  double cache_max_age = DEFAULT_MAX_CACHE_AGE;
  const char *user_cache_time_str = getenv("ROS_CACHE_TIMEOUT");
  if(user_cache_time_str)
    cache_max_age = atof(user_cache_time_str);
  if(cache_max_age == 0.0)
    return false;
  struct stat s;
  if (stat(cache_path.c_str(), &s) == 0)
  {
    double dt = difftime(time(NULL), s.st_mtime);
#ifdef VERBOSE_DEBUG
    printf("cache age: %f\n", dt);
#endif
    // Negative cache_max_age means it's always new enough.  It's dangerous
    // for the user to set this, but rosbash uses it.
    if ((cache_max_age > 0.0) && (dt > cache_max_age))
      return false;
  }
  // try to open it 
  FILE *cache = fopen(cache_path.c_str(), "r");
  if (!cache)
    return false; // it's not readable by us. sad.

  if(flock(fileno(cache), LOCK_EX) != 0)
  {
    fprintf(stderr, "[rospack] Error: failed to lock cache file\n");
    return false;
  }
  // see if ROS_ROOT and ROS_STACK_PATH are identical
  char linebuf[30000];
  bool ros_root_ok = false, ros_stack_path_ok = false;
  const char *ros_stack_path = getenv("ROS_STACK_PATH");
  while (!feof(cache))
  {
    linebuf[0] = 0;
    if (!fgets(linebuf, sizeof(linebuf), cache))
      break;
    if (!linebuf[0])
      continue;
    linebuf[strlen(linebuf)-1] = 0; // get rid of trailing newline
    if (linebuf[0] == '#')
    {
      if (!strncmp("#ROS_ROOT=", linebuf, 10))
      {
        if (!strcmp(linebuf+10, ros_root))
          ros_root_ok = true;
      }
      else if (!strncmp("#ROS_STACK_PATH=", linebuf, 16))
      {
        if (!ros_stack_path)
        {
          if (!strlen(linebuf+16))
            ros_stack_path_ok = true;
        }
        else if (!strcmp(linebuf+16, getenv("ROS_STACK_PATH")))
          ros_stack_path_ok = true;
      }
    }
    else
      break; // we're out of the header. nothing more matters to this check.
  }
  if(flock(fileno(cache), LOCK_UN) != 0)
    fprintf(stderr, "[rosstack] Error: failed to unlock cache file\n");
  fclose(cache);
  return ros_root_ok && ros_stack_path_ok;
}

class CrawlQueueEntry
{
public:
  string path;
  double start_time, elapsed_time;
  CrawlQueueEntry(string _path) 
  : path(_path), start_time(0), elapsed_time(0) { }
  bool operator>(const CrawlQueueEntry &rhs) const
  {
    return elapsed_time > rhs.elapsed_time;
  }
};
  
double ROSStack::time_since_epoch()
{
  struct timeval tod;
  gettimeofday(&tod, NULL);
  return tod.tv_sec + 1e-6 * tod.tv_usec;
}

void ROSStack::crawl_for_stacks(bool force_crawl)
{
  for (VecStack::iterator p = Stack::stacks.begin(); 
       p != Stack::stacks.end(); ++p)
    delete *p;
  Stack::stacks.clear();

  if(!force_crawl && cache_is_good())
  {
    string cache_path = getCachePath();
    FILE *cache = fopen(cache_path.c_str(), "r");
    if (cache) // one last check just in case nutty stuff happened in between
    {
#ifdef VERBOSE_DEBUG
      printf("trying to use cache...\n");
#endif
      char linebuf[30000];
      while (!feof(cache))
      {
        linebuf[0] = 0;
        if (!fgets(linebuf, sizeof(linebuf), cache))
          break; // error in read operation
        if (!linebuf[0] || linebuf[0] == '#')
          continue;
        char *newline_pos = strchr(linebuf, '\n');
        if (newline_pos)
          *newline_pos = 0;
        Stack::stacks.push_back(new Stack(linebuf));
      }
      fclose(cache);
      return; // cache load went OK; we're done here.
    }
  }
  // if we get here, this means the cache either bogus or we've been
  // instructed to rebuild it.
#ifdef VERBOSE_DEBUG
  printf("building cache\n");
#endif
  deque<CrawlQueueEntry> q;
  q.push_back(CrawlQueueEntry(ros_root));
  string rsp = getenv("ROS_STACK_PATH") ? string(getenv("ROS_STACK_PATH")) : "";
  if (!rsp.length())
  {
    // some semblance of a default. not sure what's best.
    //rsp = ros_root + string("/..");
    rsp = string(getenv("HOME")) + string("/ros");
  }
  vector<string> rspvec;
  string_split(rsp, rspvec, ":");
  for (vector<string>::iterator i = rspvec.begin(); i != rspvec.end(); ++i)
  {
    // Check whether this part of ROS_STACK_PATH is itself a package/stack
    if (Stack::is_stack(*i))
    {
      Stack::stacks.push_back(new Stack(*i));
    }
    else if (Stack::is_package(*i))
      continue; // ignore it.
    else if (Stack::is_no_subdirs(*i))
      fprintf(stderr, "[rosstack] WARNING: non-stack directory in "
                      "ROS_STACK_PATH marked rosstack_nosubdirs:\n\t%s\n",
              i->c_str());
   else
      q.push_back(CrawlQueueEntry(*i));
  }
  const double crawl_start_time = time_since_epoch();
  priority_queue<CrawlQueueEntry, vector<CrawlQueueEntry>, 
                 greater<CrawlQueueEntry> > profile;
  while (!q.empty())
  {
    CrawlQueueEntry cqe = q.front();
    q.pop_front();
    //printf("crawling %s\n", cqe.path.c_str());
    if (g_profile_length > 0)
    {
      if (cqe.start_time != 0)
      {
        // this stack symbol means we've already crawled its children, and it's
        // just here for timing purposes. save the traversal time and bail.
        cqe.elapsed_time = time_since_epoch() - cqe.start_time;
        profile.push(cqe);
        if (profile.size() > g_profile_length) // only save the worst guys
          profile.pop();
        continue;
      }
      cqe.start_time = time_since_epoch();
      q.push_front(cqe);
    }
    DIR *d = opendir(cqe.path.c_str());
    if (!d)
    {
      fprintf(stderr, "[rosstack] opendir error [%s] while crawling %s\n", 
              strerror(errno), cqe.path.c_str());
      continue;
    }
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL)
    {
      struct stat s;
      string child_path = cqe.path + fs_delim + string(ent->d_name);
      if (stat(child_path.c_str(), &s) != 0) 
        continue;
      if (!S_ISDIR(s.st_mode)) 
        continue;
      if (ent->d_name[0] == '.')
        continue; // ignore hidden dirs
      else if (Stack::is_package(child_path))
        continue; // ignore this guy, he's a leaf.
      if (Stack::is_stack(child_path))
      {
        // Filter out duplicates; first encountered takes precedence
        Stack *newp = new Stack(child_path);
        //printf("found stack %s\n", child_path.c_str());
        // TODO: make this check more efficient
        bool dup = false;
        for(std::vector<Stack *>::const_iterator it = Stack::stacks.begin();
            it != Stack::stacks.end();
            it++)
        {
          // TODO: check versions too
          if((*it)->name == newp->name)
          {
            dup=true;
            break;
          }
        }
        if(dup)
          delete newp;
        else
          Stack::stacks.push_back(newp);
      }
      //check to make sure we're allowed to descend
      else if (!Stack::is_no_subdirs(child_path)) 
        q.push_front(CrawlQueueEntry(child_path));
    }
    closedir(d);
  }
  crawled = true; // don't try to re-crawl if we can't find something
  const double crawl_elapsed_time = time_since_epoch() - crawl_start_time;
  // write the results of this crawl to the cache file
  string cache_path = getCachePath();
  FILE *cache = fopen(cache_path.c_str(), "w");
  if(flock(fileno(cache), LOCK_EX) != 0)
    fprintf(stderr, "[rosstack] Error: failed to lock cache file\n");
  else
  {
    if (!cache)
    {
      fprintf(stderr, "woah! couldn't create the cache file. Please check "
              "ROS_ROOT to make sure it's a writeable directory.\n");
      throw runtime_error(string("failed to create cache"));
    }
    char *rsp = getenv("ROS_STACK_PATH");
    fprintf(cache, "#ROS_ROOT=%s\n#ROS_STACK_PATH=%s\n", ros_root,
            (rsp ? rsp : ""));
    for (VecStack::iterator s = Stack::stacks.begin();
         s != Stack::stacks.end(); ++s)
      fprintf(cache, "%s\n", (*s)->path.c_str());
    if(flock(fileno(cache), LOCK_UN) != 0)
      fprintf(stderr, "[rosstack] Error: failed to unlock cache file\n");
    fclose(cache);
  }
  if (g_profile_length)
  {
    // dump it into a stack to reverse it (so slowest guys are first)
    stack<CrawlQueueEntry> reverse_profile;
    while (!profile.empty())
    {
      reverse_profile.push(profile.top());
      profile.pop();
    }
    printf("\nFull tree crawl took %.6f seconds.\n", crawl_elapsed_time);
    printf("-------------------------------------------------------------\n");
    while (!reverse_profile.empty())
    {
      CrawlQueueEntry cqe = reverse_profile.top();
      reverse_profile.pop();
      printf("%.6f  %s\n", cqe.elapsed_time, cqe.path.c_str());
    }
    printf("\n");
  }
}

//////////////////////////////////////////////////////////////////////////////

void rosstack::string_split(const string &s, vector<string> &t, const string &d)
{
  t.clear();
  size_t start = 0, end;
  while ((end = s.find_first_of(d, start)) != string::npos)
  {
    t.push_back(s.substr(start, end-start));
    start = end + 1;
  }
  t.push_back(s.substr(start));
}

bool rosstack::file_exists(const string &fname)
{
  return (access(fname.c_str(), F_OK) == 0); // will be different in windows
}

Stack *rosstack::g_get_stack(const string &name, const string &version)
{
  return g_rosstack->get_stack(name, version);
}

