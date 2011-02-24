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
#include <map>
#include <stack>
#include <queue>
#include <cassert>
#if !defined(WIN32)
  #include <unistd.h>
  #include <dirent.h>
  #include <sys/time.h>
  #include <sys/file.h>
  #include <stdint.h>
#endif
#include <stdexcept>
#include <time.h>
#include <sstream>
#include <iterator>

#if defined(_MSC_VER) // msvc only
  #define F_OK 0x00
  #define W_OK 0x02
  #define R_OK 0x04
#else                // non msvc only
  #include <libgen.h>
#endif

#if defined(WIN32) // both msvc and mingw
  #include <direct.h>
  #include <time.h>
  #include <windows.h>
  #include <io.h>
  #include <fcntl.h>
  #define PATH_MAX MAX_PATH
  #define snprintf _snprintf
  #define getcwd _getcwd
  #define fdopen _fdopen
  #define access _access
  #define mkdir(a,b) _mkdir(a)
#endif

#include "tinyxml-2.5.3/tinyxml.h"
#include "rospack/rosstack.h"
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
  #if defined(WIN32)
    const string g_ros_os("win32");
  #else
    const string g_ros_os("linux");
  #endif
#endif

#if defined(_MSVC_VER)
// The MS compiler complains bitterly about undefined symbols due to the
// static members of the TiXmlBase class. They need to exist in every
// compilation unit (i.e. DLL or EXE), but they don't get exported
// properly across DLL boundaries (for reasons I've tried to investigate,
// before deciding it was a waste of my time). So they're defined here as well
// to keep rosstack happy (rospack's lib links directly to tinyxml.cpp,
// rosstack's lib does not).
// I'll fix this later. Thanks, MS, for creating yet another broken system.
const int TiXmlBase::utf8ByteTable[256] =
{
	//	0	1	2	3	4	5	6	7	8	9	a	b	c	d	e	f
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x00
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x10
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x20
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x30
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x40
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x50
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x60
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x70	End of ASCII range
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x80 0x80 to 0xc1 invalid
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x90 
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0xa0 
		1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0xb0 
		1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	// 0xc0 0xc2 to 0xdf 2 byte
		2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	// 0xd0
		3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	// 0xe0 0xe0 to 0xef 3 byte
		4,	4,	4,	4,	4,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1	// 0xf0 0xf0 to 0xf4 4 byte, 0xf5 and higher invalid
};
bool TiXmlBase::condenseWhiteSpace = true;
#endif

//////////////////////////////////////////////////////////////////////////////
// Global storage for --foo options
// --deps-only
bool g_deps_only;
// --length=
string g_length;
// The stack name
string g_stack;
// the number of entries to list in the profile table
unsigned int g_profile_length = 0;
// global singleton rosstack pointer... yeah, I know.
ROSStack *g_rosstack = NULL; 

//////////////////////////////////////////////////////////////////////////////

#if defined(WIN32)
  // This isn't entirely necessary - the Win32 API functions handle / just as
  // well as \ for paths, and CMake chokes if we output paths with \ in them
  // anyway.
  const char *rosstack::fs_delim = "\\";
  const char *rosstack::path_delim = ";";
#else
  const char *rosstack::fs_delim = "/";
  const char *rosstack::path_delim = ":";
#endif


Stack::Stack(string _path) : path(_path), 
        deps_calculated(false), direct_deps_calculated(false),
        descendants_calculated(false), manifest_loaded(false)
{
  vector<string> path_tokens;
  string_split(path, path_tokens, fs_delim);
  name = path_tokens.back();
  // Don't load the manifest here, because it causes spurious errors to be
  // printed if the stack has been moved (#1785).  Presumably the manifest
  // will be loaded later, prior to being needed.
  //load_manifest();
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
const VecStack &Stack::deps1()
{
  return direct_deps();
}
const VecStack &Stack::deps(traversal_order_t order, int depth)
{
  if (depth > 1000)
  {
    fprintf(stderr,"[rosstack] woah! expanding the dependency tree made it blow "
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
    // Must make a copy here, because the call to g_get_stack() below might
    // cause a recrawl, which blows aways the accumulated data structure.
    string dep_stackname_copy = string(dep_stackname);
    string name_copy = name;
#ifdef VERBOSE_DEBUG
    printf("direct_deps: stk %s has dep %s\n",
           name.c_str(), dep_stackname_copy.c_str());
#endif 
    try
    {
      _direct_deps.push_back(g_get_stack(dep_stackname_copy));
    }
    catch (runtime_error &e)
    {
      if (missing_stack_as_warning)
        fprintf(stderr, "[rosstack] warning: couldn't find dependency "
                        "[%s] of [%s]\n",
                dep_stackname_copy.c_str(), name_copy.c_str());
      else
      {
        fprintf(stderr, "[rosstack] couldn't find dependency [%s] of [%s]\n",
                dep_stackname_copy.c_str(), name_copy.c_str());
        throw runtime_error(string("missing dependency"));
      }
    }
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
    fprintf(stderr, "[rosstack] warning: error parsing manifest file at [%s]. Blowing away the cache...\n",
            manifest_path().c_str());
    g_rosstack->deleteCache();
    // Only want this warning printed once.
    manifest_loaded = true;
    throw runtime_error(errmsg);
  }
  TiXmlElement *mroot = manifest.RootElement();
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
          "    list\n"
          "    list-names\n"
          "    depends [stack] (alias: deps)\n"
          "    depends-manifests [stack] (alias: deps-manifests)\n"
          "    depends1 [stack] (alias: deps1)\n"
          "    depends-indent [stack] (alias: deps-indent)\n"
          "    depends-on [stack]\n"
          "    depends-on1 [stack]\n"
          "    contains [package]\n"
          "    contains-path [package]\n"
          "    profile [--length=<length>] \n\n"
          " If [stack] is omitted, the current working directory\n"
          " is used (if it contains a stack.xml).\n\n";
}

Stack *ROSStack::get_stack(const string &stack_name)
{
#ifdef VERBOSE_DEBUG
  printf("searching for stack %s\n", stack_name.c_str());
#endif
  for (VecStack::iterator p = Stack::stacks.begin(); 
       p != Stack::stacks.end(); ++p)
  {
    if ((*p)->name == stack_name)
    {
      if(!crawled)
      {
        // Answer come from the cache; check that the path is valid, and
        // contains a manifest (related to #1115).
        std::string manifest_path = (*p)->path + fs_delim + "stack.xml";
        struct stat s;
        int ret;
        while((ret = stat(manifest_path.c_str(), &s)) != 0 &&
              errno == EINTR);
        if(ret == 0)
        {
          // Answer looks good
          return (*p);
        }
        else
        {
          // Bad cache.  Warn and fall through to the recrawl below.
          fprintf(stderr, "[rosstack] warning: invalid cached location %s for package %s; forcing recrawl\n",
                  (*p)->path.c_str(),
                  (*p)->name.c_str());
          break;
        }
      }
      else
      {
        // Answer came from a fresh crawl; no further checking needed.
        return (*p);
      }
    }
  }
  if (!crawled) // maybe it's a brand-new stack. force a crawl.
  {
    crawl_for_stacks(true); // will set the crawled flag; recursion is safe
    return get_stack(stack_name);
  }
  throw runtime_error(string("couldn't find stack ") + stack_name);
  return NULL; // or not
}
  
int ROSStack::cmd_depends_on(bool include_indirect)
{
  // We can't proceed if the argument-parsing logic wasn't able to provide
  // any package name.  Note that we need to check for an empty opt_package
  // here, but not in other places (e.g., cmd_deps()), because here we're
  // catching the exception that get_pkg() throws when it can't find the
  // package.  Elsewhere, we let that exception propagate up.
  if(g_stack.size() == 0)
  {
    string errmsg = string("no stack name given, and current directory is not a stack root");
    throw runtime_error(errmsg);
  }

  // Explicitly crawl for stacks, to ensure that we get newly added
  // dependent stacks.  We also avoid the possibility of a recrawl
  // happening within the loop below, which could invalidate the stacks
  // vector as we loop over it.
  crawl_for_stacks(true);

  Stack* s;
  try
  {
    s = get_stack(g_stack);
  }
  catch(runtime_error)
  {
    fprintf(stderr, "[rosstack] warning: stack %s doesn't exist\n", 
            g_stack.c_str());
    //s = new Stack(g_stack);
    //Stack::stacks.push_back(s);
    s = add_stack(g_stack);
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
  Stack *p = get_stack(g_stack);
  printf("%s\n", p->path.c_str());
  return 0;
}

string ROSStack::lookup_owner(string pkg_name, bool just_owner_name)
{
  // hack... we'll treat g_stack as the name of the package to look up.
  rospack::Package *pkg = rp.get_pkg(pkg_name);
#ifdef VERBOSE_DEBUG
  printf("package path: [%s]\n", pkg->path.c_str());
#endif
  map<string, string> bases; // all the places the search can bottom out
  for (VecStack::iterator p = Stack::stacks.begin(); // first, add stacks
       p != Stack::stacks.end(); ++p)
    bases[(*p)->path] = (*p)->name;
  /*
  char *rr = getenv("ROS_ROOT"); // add ROS_ROOT
  if (rr)
  {
    bases
  }
  */
  char *rpp = getenv("ROS_PACKAGE_PATH"); // add ROS_PACKAGE_PATH entries
  if (rpp)
  {
    vector<string> rppvec;
    string_split(rpp, rppvec, path_delim);
    sanitize_rppvec(rppvec);
    for (vector<string>::iterator i = rppvec.begin(); i != rppvec.end(); ++i)
      bases[*i] = string("");
  }
#ifdef VERBOSE_DEBUG
  printf("bases:\n");
  for (map<string, string>::iterator i = bases.begin(); i != bases.end(); ++i)
    printf("%s -> %s\n", i->first.c_str(), i->second.c_str());
#endif
  // now, chop the package path until we hit one of the bases
  string pkg_path_fragment = pkg->path;
  while (pkg_path_fragment.length() > 1)
  {
    // chop off everything to the right of the last slash
    size_t last_slash_pos = pkg_path_fragment.find_last_of('/');
    if (last_slash_pos == string::npos)
      break; // shouldn't happen, but might as well catch it
    pkg_path_fragment = pkg_path_fragment.substr(0, last_slash_pos);
#ifdef VERBOSE_DEBUG
    printf("frag = %s\n", pkg_path_fragment.c_str());
#endif
    map<string, string>::iterator i = bases.find(pkg_path_fragment);
    if (i != bases.end())
    {
      if (just_owner_name)
        return bases[pkg_path_fragment];
      else
        return pkg_path_fragment;
      break;
    }
  }
  return string("");
}

int ROSStack::cmd_contains()
{
  printf("%s\n", lookup_owner(g_stack, true).c_str());
  return 0;
}

int ROSStack::cmd_contains_path()
{
  printf("%s\n", lookup_owner(g_stack, false).c_str());
  return 0;
}

int ROSStack::cmd_deps()
{
  VecStack d = get_stack(g_stack)->deps(Stack::POSTORDER);
  for (VecStack::iterator i = d.begin(); i != d.end(); ++i)
    printf("%s\n", (*i)->name.c_str());
  return 0;
}

int ROSStack::cmd_deps_manifests()
{
  VecStack d = get_stack(g_stack)->deps(Stack::POSTORDER);
  for (VecStack::iterator i = d.begin(); i != d.end(); ++i)
    printf("%s/stack.xml ", (*i)->path.c_str());
  puts("");
  return 0;
}

int ROSStack::cmd_deps1()
{
  VecStack d = get_stack(g_stack)->deps1();
  for (VecStack::iterator i = d.begin(); i != d.end(); ++i)
    printf("%s\n", (*i)->name.c_str());
  return 0;
}

int ROSStack::cmd_depsindent(Stack *stack, int indent)
{
  VecStack d = stack->deps1();
  for (VecStack::iterator i = d.begin(); i != d.end(); ++i)
  {
    for(int s=0; s<indent; s++)
      printf(" ");
    printf("%s\n", (*i)->name.c_str());
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
    g_stack = string(argv[i++]);
  }
  // Are we sitting in a stack?
  else if(Stack::is_stack("."))
  {
    char buf[1024];
    if(!getcwd(buf,sizeof(buf)))
      throw runtime_error(errmsg);
#if defined(_MSC_VER)
    // No basename on Windows; use _splitpath_s instead
    char drive[_MAX_DRIVE], dir[_MAX_DIR], fname[_MAX_FNAME], ext[_MAX_EXT];
    _splitpath_s(buf, drive, _MAX_DRIVE, dir, _MAX_DIR, fname, _MAX_FNAME,
                 ext, _MAX_EXT);
    char filename[_MAX_FNAME + _MAX_EXT];
    if (ext[0] != '\0')
    {
      _makepath_s(filename, _MAX_FNAME + _MAX_EXT, NULL, NULL, fname, ext);
      g_stack = string(filename);
    }
    else
      g_stack = string(fname);
#else
    g_stack = string(basename(buf));
#endif
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
  else if (!strcmp(cmd, "contains"))
    return cmd_contains();
  else if (!strcmp(cmd, "contains-path"))
    return cmd_contains_path();
  else if (!strcmp(cmd, "list"))
    return cmd_print_stack_list(true);
  else if (!strcmp(cmd, "list-names"))
    return cmd_print_stack_list(false);
  else if (!strcmp(cmd, "contents"))
    return cmd_print_packages();
  else if (!strcmp(cmd, "depends") || !strcmp(cmd, "deps"))
    return cmd_deps();
  else if (!strcmp(cmd, "depends-manifests") || !strcmp(cmd, "deps-manifests"))
    return cmd_deps_manifests();
  else if (!strcmp(cmd, "depends1") || !strcmp(cmd, "deps1"))
    return cmd_deps1();
  else if (!strcmp(cmd, "depends-indent") || !strcmp(cmd, "deps-indent"))
    return cmd_depsindent(get_stack(g_stack), 0);
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
      printf("%s %s\n", (*i)->name.c_str(), (*i)->path.c_str());
    else
      printf("%s\n", (*i)->name.c_str());
  return 0;
}

int ROSStack::cmd_print_packages()
{
  rospack::ROSPack rp;
  string path = get_stack(g_stack)->path;
  //printf("partial crawl of %s\n", path.c_str());
  rospack::VecPkg pkgs = rp.partial_crawl(path);
  //printf("found %d pkgs\n", pkgs.size());
  for (rospack::VecPkg::iterator i = pkgs.begin(); i != pkgs.end(); ++i)
  {
    printf("%s\n", (*i)->name.c_str());
    delete *i;
  }
  return 0;
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
  path = string(ros_root) + fs_delim + ".rosstack_cache";
  if (access(ros_root, W_OK) == 0)
    return path;
  // if we cannot write into the ros_root, then let's try to
  // write into the user's .ros directory.
  createROSHomeDirectory();
  path = string(getenv("HOME")) + fs_delim + ".ros" + fs_delim + "rosstack_cache";
  return path;
}

void ROSStack::deleteCache()
{
  string cache_path = g_rosstack->getCachePath();
  if (file_exists(cache_path))
    remove(cache_path.c_str());
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

  // see if ROS_ROOT and ROS_PACKAGE_PATH are identical
  char linebuf[30000];
  bool ros_root_ok = false, ros_package_path_ok = false;
  const char *ros_package_path = getenv("ROS_PACKAGE_PATH");
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
      else if (!strncmp("#ROS_PACKAGE_PATH=", linebuf, 18))
      {
        if (!ros_package_path)
        {
          if (!strlen(linebuf+18))
            ros_package_path_ok = true;
        }
        else if (!strcmp(linebuf+18, getenv("ROS_PACKAGE_PATH")))
          ros_package_path_ok = true;
      }
    }
    else
      break; // we're out of the header. nothing more matters to this check.
  }
  fclose(cache);
  return ros_root_ok && ros_package_path_ok;
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
#if defined(WIN32)
  #if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
    #define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
  #else
    #define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
  #endif
  FILETIME ft;
  unsigned __int64 tmpres = 0;

  GetSystemTimeAsFileTime(&ft);
  tmpres |= ft.dwHighDateTime;
  tmpres <<= 32;
  tmpres |= ft.dwLowDateTime;
  tmpres /= 10;
  tmpres -= DELTA_EPOCH_IN_MICROSECS;
  return static_cast<double>(tmpres) / 1e6;
#else
  struct timeval tod;
  gettimeofday(&tod, NULL);
  return tod.tv_sec + 1e-6 * tod.tv_usec;
#endif
}

// Add stack, filtering out duplicates.
Stack* ROSStack::add_stack(string path)
{
  // Filter out duplicates; first encountered takes precedence
  Stack* newp = new Stack(path);
  Stack* return_p = newp;
  // TODO: make this check more efficient
  bool dup = false;
  for(std::vector<Stack *>::const_iterator it = Stack::stacks.begin();
      it != Stack::stacks.end();
      it++)
  {
    if((*it)->name == newp->name)
    {
      dup=true;
      return_p = *it;
      break;
    }
  }
  if(dup)
    delete newp;
  else
    Stack::stacks.push_back(newp);

  return return_p;
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
        //Stack::stacks.push_back(new Stack(linebuf));
        add_stack(linebuf);
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
  vector<string> rspvec;
  // seed the crawler with ROS_ROOT and ROS_PACKAGE_PATH 
  char *rr = getenv("ROS_ROOT");
  if (!rr)
  {
    fprintf(stderr, "[rosstack] ERROR: ROS_ROOT not set.\n");
    exit(1);
  }
  // Add the ROS stack
  //Stack::stacks.push_back(new Stack(string(rr)));
  add_stack(string(rr));
  string rsp;
  char *rpp = getenv("ROS_PACKAGE_PATH");
  if (rpp)
    rsp = string(rpp);
  string_split(rsp, rspvec, path_delim);
  sanitize_rppvec(rspvec);
#ifdef VERBOSE_DEBUG
  printf("seeding crawler with [%s], which has %lu entries\n", rsp.c_str(), rspvec.size());
#endif

  for (vector<string>::iterator i = rspvec.begin(); i != rspvec.end(); ++i)
  {
    if (Stack::is_no_subdirs(*i))
      fprintf(stderr, "[rosstack] WARNING: non-stack directory in "
                      "ROS_PACKAGE_PATH marked "
                      "rosstack_nosubdirs:\n\t%s\n",
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

    // Check whether this part of ROS_PACKAGE_PATH is itself a package/stack
    if (Stack::is_stack(cqe.path))
    {
      //Stack::stacks.push_back(new Stack(*i));
      add_stack(cqe.path);
      continue;
    }
    else if (Stack::is_package(cqe.path))
      continue; // ignore it.

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
#if defined(WIN32)
    // And again...
    WIN32_FIND_DATA find_file_data;
    HANDLE hfind = INVALID_HANDLE_VALUE;

    if ((hfind = FindFirstFile((cqe.path + "\\*").c_str(),
                               &find_file_data)) == INVALID_HANDLE_VALUE)
    {
      fprintf(stderr, "[rosstack] FindFirstFile error %u while crawling %s\n",
              GetLastError(), cqe.path.c_str());
      continue;
    }

    do
    {
      if (!S_ISDIR(find_file_data.dwFileAttributes))
        continue; // Ignore non-directories
      if (find_file_data.cFileName[0] == '.')
        continue; // Ignore hidden directories
      string child_path = cqe.path + fs_delim + string(find_file_data.cFileName);
      if (Stack::is_stack(child_path))
        continue; // Ignore leaves.
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
    while (FindNextFile(hfind, &find_file_data) != 0);
    DWORD last_error = GetLastError();
    FindClose(hfind);
    if (last_error != ERROR_NO_MORE_FILES)
    {
      fprintf(stderr, "[rosstack] FindNextFile error %u while crawling %s\n",
              GetLastError(), cqe.path.c_str());
      continue;
    }
#else
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
        add_stack(child_path);
        /*
        // Filter out duplicates; first encountered takes precedence
        Stack *newp = new Stack(child_path);
        //printf("found stack %s\n", child_path.c_str());
        // TODO: make this check more efficient
        bool dup = false;
        for(std::vector<Stack *>::const_iterator it = Stack::stacks.begin();
            it != Stack::stacks.end();
            it++)
        {
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
          */
      }
      //check to make sure we're allowed to descend
      else if (!Stack::is_no_subdirs(child_path)) 
        q.push_front(CrawlQueueEntry(child_path));
    }
    closedir(d);
#endif
  }
  crawled = true; // don't try to re-crawl if we can't find something
  const double crawl_elapsed_time = time_since_epoch() - crawl_start_time;
  // write the results of this crawl to the cache file
  string cache_path = getCachePath();
  char tmp_cache_dir[PATH_MAX];
  char tmp_cache_path[PATH_MAX];
  strncpy(tmp_cache_dir, cache_path.c_str(), sizeof(tmp_cache_dir));
#if defined(_MSC_VER)
    // No dirname on Windows; use _splitpath_s instead
    char drive[_MAX_DRIVE], dir[_MAX_DIR], fname[_MAX_FNAME], ext[_MAX_EXT];
    _splitpath_s(tmp_cache_dir, drive, _MAX_DRIVE, dir, _MAX_DIR, fname, _MAX_FNAME,
                 ext, _MAX_EXT);
    char full_dir[_MAX_DRIVE + _MAX_DIR];
    _makepath_s(full_dir, _MAX_DRIVE + _MAX_DIR, drive, dir, NULL, NULL);
    snprintf(tmp_cache_path, sizeof(tmp_cache_path), "%s\\.rosstack_cache.XXXXXX", full_dir);
#else
  snprintf(tmp_cache_path, sizeof(tmp_cache_path), "%s/.rosstack_cache.XXXXXX", dirname(tmp_cache_dir));
#endif
#if defined(__MINGW32__)
    // There is no equivalent of mkstemp or _mktemp_s on mingw, so we resort to a slightly less secure
    // method. Could use mktemp, but as we're just redirecting to FILE anyway, tmpfile() works
    // for us.
    FILE *cache = tmpfile();
    if ( cache == NULL ) {
        fprintf(stderr,
                "[rospack] Unable to generate temporary cache file name: %u",
                errno);
    }
#elif defined(WIN32)
  // This one is particularly nasty: on Windows, there is no equivalent of
  // mkstemp, so we're stuck with the security risks of mktemp. Hopefully not a
  // problem in our use cases.
  if (_mktemp_s(tmp_cache_path, PATH_MAX) != 0)
  {
    fprintf(stderr,
            "[rosstack] Unable to generate temporary cache file name: %u",
            GetLastError());
    throw runtime_error(string("Failed to create tmp cache file name"));
  }
  FILE *cache = fopen(tmp_cache_path, "w");
#else
  int fd = mkstemp(tmp_cache_path);
  if (fd < 0)
  {
    fprintf(stderr, "Unable to create temporary cache file: %s\n", tmp_cache_path);
    throw runtime_error(string("failed to create tmp cache file"));
  }

  FILE *cache = fdopen(fd, "w");
#endif
  if (!cache)
  {
    fprintf(stderr, "woah! couldn't create the cache file. Please check "
            "ROS_ROOT to make sure it's a writeable directory.\n");
    throw runtime_error(string("failed to create tmp cache file"));
  }

  fprintf(cache, "#ROS_ROOT=%s\n#ROS_PACKAGE_PATH=%s\n", ros_root, rsp.c_str());
  for (VecStack::iterator s = Stack::stacks.begin();
       s != Stack::stacks.end(); ++s)
    fprintf(cache, "%s\n", (*s)->path.c_str());
  fclose(cache);

  if(file_exists(cache_path.c_str()))
    remove(cache_path.c_str());
  if(rename(tmp_cache_path, cache_path.c_str()) < 0)
  {
    fprintf(stderr,
      "[rospack] Error: failed rename cache file %s to %s\n",
      tmp_cache_path, cache_path.c_str());
    perror("rename");
    throw runtime_error(string("failed to rename cache file"));
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
  if (start != s.length())
    t.push_back(s.substr(start));
}

bool rosstack::file_exists(const string &fname)
{
  return (access(fname.c_str(), F_OK) == 0); // will be different in windows
}

Stack *rosstack::g_get_stack(const string &name)
{
  return g_rosstack->get_stack(name);
}

void ROSStack::sanitize_rppvec(std::vector<std::string> &rppvec)
{
  // drop any trailing slashes
  for (size_t i = 0; i < rppvec.size(); i++)
  {
    size_t last_slash_pos = rppvec[i].find_last_of("/");
    if (last_slash_pos != string::npos &&
        last_slash_pos == rppvec[i].length()-1)
    {
      fprintf(stderr, "[rosstack] warning: trailing slash found in "
                      "ROS_PACKAGE_PATH\n");
      rppvec[i].erase(last_slash_pos);
    }
  }
}

