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

#include <libgen.h>

#include "tinyxml-2.5.3/tinyxml.h"
#include "rospack/rospack.h"
using namespace std;

//#define VERBOSE_DEBUG
const double DEFAULT_MAX_CACHE_AGE = 60.0; // rebuild cache every minute


#include <sys/stat.h>
#ifndef S_ISDIR 
#define S_ISDIR(x) (((x) & S_IFMT) == S_IFDIR) 
#endif

namespace rospack
{

ROSPack *g_rospack = NULL; // singleton

#ifdef __APPLE__
const string g_ros_os("osx");
#else
const string g_ros_os("linux");
#endif

const char *fs_delim = "/"; // ifdef this for windows

Package::Package(string _path) : path(_path), 
        deps_calculated(false), direct_deps_calculated(false),
        descendants_calculated(false), manifest_loaded(false)
{
  vector<string> name_tokens;
  string_split(path, name_tokens, fs_delim);
  name = name_tokens.back();
}
bool Package::is_package(string path)
{
  return file_exists(path + string(fs_delim) + "manifest.xml");
}
bool Package::is_no_subdirs(string path)
{
  return file_exists(path + string(fs_delim) + "rospack_nosubdirs");
}
const VecPkg &Package::deps1()
{
  return direct_deps();
}
const VecPkg &Package::deps(traversal_order_t order, int depth)
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
  VecPkg my_dd = direct_deps();
  for (VecPkg::iterator i = my_dd.begin(); i != my_dd.end(); ++i)
  {
    VecPkg d = (*i)->deps(order, depth+1); // recurse on direct dependencies
    if (order == PREORDER)
      _deps.push_back(*i);
    for (VecPkg::iterator j = d.begin(); j != d.end(); ++j)
    {
      // don't add things twice, but if you have something already
      // and we're doing a quasi-preorder traversal, bump it to the back
      bool have = false;
      VecPkg::iterator prior_loc;
      for (VecPkg::iterator k = _deps.begin(); k != _deps.end() && !have; ++k)
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
      for (VecPkg::iterator k = _deps.begin(); k != _deps.end() && !have; ++k)
        if ((*k) == (*i))
          have = true;
      if (!have)
        _deps.push_back(*i);
    }
  }
  deps_calculated = true;
  return _deps;
}
string Package::manifest_path()
{
  return path + string(fs_delim) + "manifest.xml";
}
string Package::flags(string lang, string attrib)
{
  VecPkg d = deps(PREORDER);
  string s;
  // Conditionally include this package's exported flags, depending on
  // whether --deps-only was given
  if(!g_rospack->opt_deps_only)
    s += this->direct_flags(lang, attrib) + string(" ");
  for (VecPkg::iterator i = d.begin(); i != d.end(); ++i)
  {
    string f = (*i)->direct_flags(lang, attrib);
    if (f.length())
      s += f + string(" ");
  }
  return s;
}

string Package::rosdep()
{
  string sd;
  TiXmlElement *mroot = manifest_root();
  for(TiXmlElement *sd_ele = mroot->FirstChildElement("rosdep");
      sd_ele;
      sd_ele = sd_ele->NextSiblingElement("rosdep"))
  {
    const char *att_str;
    if((att_str = sd_ele->Attribute("name")))
      sd += string("name: ") + string(att_str);
    sd += string("\n");
  }

  return sd;
}

string Package::versioncontrol()
{
  string sd;
  TiXmlElement *mroot = manifest_root();
  for(TiXmlElement *sd_ele = mroot->FirstChildElement("versioncontrol");
      sd_ele;
      sd_ele = sd_ele->NextSiblingElement("versioncontrol"))
  {
    const char *att_str;
    if((att_str = sd_ele->Attribute("type")))
      sd += string("type: ") + string(att_str);
    if((att_str = sd_ele->Attribute("url")))
      sd += string("\turl: ") + string(att_str);
    sd += string("\n");
  }

  return sd;
}

vector<pair<string, string> > Package::plugins()
{
  vector<pair<string, string> > plugins;

  VecPkg deplist;
  // If --top=foo was given, then restrict the search to packages that are
  // dependencies of foo, plus foo itself
  if(g_rospack->opt_top.size())
  {
    Package* gtp = g_get_pkg(g_rospack->opt_top);
    deplist = gtp->deps(Package::POSTORDER);
    deplist.push_back(gtp);
  }

  VecPkg desc1 = descendants1();
  desc1.push_back(this);
  VecPkg::iterator it = desc1.begin();
  VecPkg::iterator end = desc1.end();
  for (; it != end; ++it)
  {
    // If we're restricting the search, make sure this package is in the
    // deplist.  This could be made more efficient.
    if(deplist.size())
    {
      bool found = false;
      for(VecPkg::const_iterator dit = deplist.begin();
          dit != deplist.end();
          dit++)
      {
        if((*dit)->name == (*it)->name)
        {
          found = true;
          break;
        }
      }
      if(!found)
        continue;
    }
    std::string flags = (*it)->direct_flags(name, g_rospack->opt_attrib);
    if (!flags.empty())
    {
      plugins.push_back(make_pair((*it)->name, flags));
    }
  }

  return plugins;
}


VecPkg Package::descendants1()
{
  VecPkg children;
  // Make a copy of the pkgs vector, because a crawl can be caused in
  // has_parent()->direct_deps()->g_get_pkg()->get_pkg().  That crawl 
  // will rebuild pkgs, invalidating our iterator, causing esoteric crashes
  // e.g., #2056.
  VecPkg pkgs_copy(pkgs);
  for (VecPkg::iterator p = pkgs_copy.begin(); p != pkgs_copy.end(); ++p)
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

const vector<Package *> &Package::descendants(int depth)
{
  if (depth > 100)
  {
    fprintf(stderr, "[rospack] woah! circular dependency in the ros tree! aaaaaa!\n");
    throw runtime_error(string("circular dependency"));
  }
  if (descendants_calculated)
    return _descendants;
  VecPkg desc_with_dups;
  for (VecPkg::iterator p = pkgs.begin(); p != pkgs.end(); ++p)
  {
    // We catch exceptions here, because we don't care if some 
    // unrelated packages in the system have invalid manifests
    try
    {
      if ((*p)->has_parent(name))
      {
        desc_with_dups.push_back(*p);
        const VecPkg &p_desc = (*p)->descendants(depth+1);
        for (VecPkg::const_iterator q = p_desc.begin();
             q != p_desc.end(); ++q)
          desc_with_dups.push_back(*q);
      }
    }
    catch (runtime_error &e)
    {
    }
  }
  _descendants.clear();
  for (VecPkg::iterator p = desc_with_dups.begin();
       p != desc_with_dups.end(); ++p)
  {
    bool found = false;
    for (VecPkg::iterator q = _descendants.begin();
         q != _descendants.end() && !found; ++q)
      if ((*q)->name == (*p)->name)
        found = true;
    if (!found)
      _descendants.push_back(*p);
  }
  descendants_calculated = true;
  return _descendants;
}


bool Package::has_parent(string pkg)
{
  vector<Package *> parents = direct_deps(true);
  for (VecPkg::iterator i = parents.begin(); i != parents.end(); ++i)
    if ((*i)->name == pkg)
      return true;
  return false;
}

const vector<Package *> &Package::direct_deps(bool missing_package_as_warning)
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
    const char *dep_pkgname = dep_ele->Attribute("package");
    if (!dep_pkgname)
    {
      fprintf(stderr,"[rospack] bad depend syntax (no 'package' attribute) in [%s]\n", 
              manifest_path().c_str());
      throw runtime_error(string("invalid manifest"));
    }
    // Must make a copy here, because the call to g_get_pkg() below might
    // cause a recrawl, which blows aways the accumulated data structure.
    char* dep_pkgname_copy = strdup(dep_pkgname);
#ifdef VERBOSE_DEBUG
    printf("direct_deps: pkg %s has dep %s\n", name.c_str(), dep_pkgname_copy);
#endif 
    try
    {
      _direct_deps.push_back(g_get_pkg(dep_pkgname_copy));
    }
    catch (runtime_error &e)
    {
      if (missing_package_as_warning)
      {
        // Don't warn if we're in certain modes, #2197
        if(g_rospack->opt_warn_on_missing_deps)
        {
          fprintf(stderr, "[rospack] warning: couldn't find dependency [%s] of [%s]\n",
                  dep_pkgname_copy, name.c_str());
        }
      }
      else
      {
        fprintf(stderr, "[rospack] couldn't find dependency [%s] of [%s]\n",
                dep_pkgname_copy, name.c_str());
        free(dep_pkgname_copy);
        throw runtime_error(string("missing dependency"));
      }
    }
    free(dep_pkgname_copy);
  }
  direct_deps_calculated = true;
  return _direct_deps;
}

string Package::cpp_message_flags(bool cflags, bool lflags)
{
  bool msg_exists = file_exists((path + "/msg_gen/generated").c_str());
  bool srv_exists = file_exists((path + "/srv_gen/generated").c_str());

  string flags;

  if (cflags)
  {
    if (msg_exists)
    {
      flags += string(" -I") + path + "/msg_gen/cpp/include";
    }

    if (srv_exists)
    {
      flags += string(" -I") + path + "/srv_gen/cpp/include";
    }
  }

  // lflags not needed until we have a cpp file, but this implementation works, adding -l<package_name>msgs and -l<package_name>srvs
  // we'll probably need to figure out a better way of testing for msg/srvs than just checking if <package_path>/msg|srv exists though
#if 0
  if (lflags)
  {
    if (msg_exists)
    {
      flags += string(" -L") + path + "/lib";
      flags += string(" -Wl,-rpath,") + path + "/lib";
      flags += " -l" + package->name + "msgs";
    }

    if (srv_exists)
    {
      // if msgs already exist, we'll already have added this to the flags
      if (!msg_exists)
      {
        flags += string(" -L") + path + "/lib";
        flags += string(" -Wl,-rpath,") + path + "/lib";
      }

      flags += " -l" + package->name + "srvs";
    }
  }
#endif

  flags += " ";
  return flags;
}

string Package::direct_flags(string lang, string attrib)
{
  TiXmlElement *mroot = manifest_root();
  TiXmlElement *export_ele = mroot->FirstChildElement("export");
  string str;
  if (export_ele)
  {
    bool os_match = false;
    TiXmlElement *best_usage = NULL;
    for (TiXmlElement *lang_ele = export_ele->FirstChildElement(lang);
         lang_ele; lang_ele = lang_ele->NextSiblingElement(lang))
    {
      const char *os_str;
      if ((os_str = lang_ele->Attribute("os")))
      {
        if(g_ros_os == string(os_str))
        {
          if(os_match)
          {
            fprintf(stderr, "[rospack] warning: ignoring duplicate \"%s\" tag with os=\"%s\" in export block\n",
                    lang.c_str(), os_str);
          }
          else
          {
            best_usage = lang_ele;
            os_match = true;
          }
        }
      }
      if(!os_match)
      {
        if (!best_usage)
          best_usage = lang_ele;
        else if(!os_str)
        {
          fprintf(stderr, "[rospack] warning: ignoring duplicate \"%s\" tag in export block\n",
                  lang.c_str());
        }
      }
    }
    // If we found some exported text, then start parsing it.  Either way,
    // we end up at the logic at the bottom that will conditionally
    // append msg_gen / srv_gen includes.  This structure was changed to
    // fix #3018.
    if (best_usage)
    {
      const char *cstr = best_usage->Attribute(attrib.c_str());
      if (cstr)
      {
        str = cstr;
        while (1) // every decent C program has a while(1) in it
        {
          int i = str.find(string("${prefix}"));
          if (i < 0)
            break; // no more occurrences
          str.replace(i, string("${prefix}").length(), path);
        }

        // Do backquote substitution.  E.g.,  if we find this string:
        //   `pkg-config --cflags gdk-pixbuf-2.0`
        // We replace it with the result of executing the command
        // contained within the backquotes (reading from its stdout), which
        // might be something like:
        //   -I/usr/include/gtk-2.0 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include

        // Construct and execute the string
        // We do the assignment first to ensure that if backquote expansion (or
        // anything else) fails, we'll get a non-zero exit status from pclose().
        string cmd = string("ret=\"") + str + string("\" && echo $ret");

        // Remove embedded newlines
        string token("\n");
        for (string::size_type s = cmd.find(token); s != string::npos;
             s = cmd.find(token, s))
        {
          cmd.replace(s,token.length(),string(" "));
        }

        FILE* p;
        if(!(p = popen(cmd.c_str(), "r")))
        {
          fprintf(stderr, "[rospack] warning: failed to execute backquote "
                  "expression \"%s\" in [%s]\n",
                  cmd.c_str(), manifest_path().c_str());
          string errmsg = string("error in backquote expansion for ") + g_rospack->opt_package;
          throw runtime_error(errmsg);
        }
        else
        {
          char buf[8192];
          memset(buf,0,sizeof(buf));
          // Read the command's output
          do
          {
            clearerr(p);
            while(fgets(buf + strlen(buf),sizeof(buf)-strlen(buf)-1,p));
          } while(ferror(p) && errno == EINTR);
          // Close the subprocess, checking exit status
          if(pclose(p) != 0)
          {
            fprintf(stderr, "[rospack] warning: got non-zero exit status from executing backquote expression \"%s\" in [%s]\n",
                    cmd.c_str(), manifest_path().c_str());
            string errmsg = string("error in backquote expansion for ") + g_rospack->opt_package;
            throw runtime_error(errmsg);
          }
          else
          {
            // Strip trailing newline, which was added by our call to echo
            buf[strlen(buf)-1] = '\0';
            // Replace the backquote expression with the new text
            str = string(buf);
          }
        }
      }
    }
  }

  if (lang == "cpp")
  {
    if (attrib == "cflags")
    {
      // Message flags go last so it's possible to override them
      str += cpp_message_flags(true, false);
    }
    else if (attrib == "lflags")
    {
      // Message flags go last so it's possible to override them
      str += cpp_message_flags(false, true);
    }
  }

  return str;
}

void Package::load_manifest()
{
  if (manifest_loaded)
    return;
  if (!manifest.LoadFile(manifest_path()))
  {
    string errmsg = string("error parsing manifest file at [") + manifest_path().c_str() + string("]");
    fprintf(stderr, "[rospack] warning: error parsing manifest file at [%s]\n",
            manifest_path().c_str());
    // Only want this warning printed once.
    manifest_loaded = true;
    throw runtime_error(errmsg);
  }
}

TiXmlElement *Package::manifest_root()
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

// Naive recursion.  Dynamic programming would be more efficient.
void Package::accumulate_deps(AccList& acc_list, Package* to)
{
  VecPkg dd = direct_deps();
  for(VecPkg::iterator it = dd.begin(); 
      it != dd.end();
      ++it)
  {
    if((*it)->name == to->name)
    {
      Acc acc;
      acc.push_back(this);
      acc.push_back(to);
      acc_list.push_back(acc);
    }
    else
    {
      AccList l;
      (*it)->accumulate_deps(l, to);
      for(AccList::iterator lit = l.begin();
          lit != l.end();
          ++lit)
      {
        lit->push_front(this);
        acc_list.push_back(*lit);
      }
    }
  }
}

VecPkg Package::pkgs;
VecPkg Package::deleted_pkgs;

//////////////////////////////////////////////////////////////////////////////


ROSPack::ROSPack() : ros_root(NULL), opt_quiet(false),
        cache_lock_failed(false), crawled(false), my_argc(0),
        my_argv(NULL), opt_profile_length(0), total_num_pkgs(0),
        duplicate_packages_found(false)
{
  g_rospack = this;
  Package::pkgs.reserve(500); // get some space to avoid early recopying...
  ros_root = getenv("ROS_ROOT");
  if (!ros_root)
  {
    fprintf(stderr,"[rospack] ROS_ROOT is not defined in the environment.\n");
    throw runtime_error(string("no ROS_ROOT"));
  }
  if (!file_exists(ros_root))
  {
    fprintf(stderr,"[rospack] the path specified as ROS_ROOT is not " 
                   "accessible. Please ensure that this environment variable "
                   "is set and is writeable by your user account.\n");
    throw runtime_error(string("no ROS_ROOT"));
  }

  crawl_for_packages();
}

ROSPack::~ROSPack() 
{ 
  for (VecPkg::iterator p = Package::pkgs.begin(); 
       p != Package::pkgs.end(); ++p)
    delete (*p);
  Package::pkgs.clear();
  for (VecPkg::iterator p = Package::deleted_pkgs.begin(); 
       p != Package::deleted_pkgs.end(); ++p)
    delete (*p);
  Package::deleted_pkgs.clear();
  freeArgv();
}

const char* ROSPack::usage()
{
  return "USAGE: rospack <command> [options] [package]\n"
          "  Allowed commands:\n"
          "    help\n"
          "    find [package]\n"
          "    list\n"
          "    list-names\n"
          "    list-duplicates\n"
          "    langs\n"
          "    depends [package] (alias: deps)\n"
          "    depends-manifests [package] (alias: deps-manifests)\n"
          "    depends-msgsrv [package] (alias: deps-msgsrv)\n"
          "    depends1 [package] (alias: deps1)\n"
          "    depends-indent [package] (alias: deps-indent)\n"
          "    depends-why --target=<target> [package] (alias: deps-why)\n"
          "    rosdep [package] (alias: rosdeps)\n"
          "    rosdep0 [package] (alias: rosdeps0)\n"
          "    vcs [package]\n"
          "    vcs0 [package]\n"
          "    depends-on [package]\n"
          "    depends-on1 [package]\n"
          "    export [--deps-only] --lang=<lang> --attrib=<attrib> [package]\n"
          "    plugins --attrib=<attrib> [--top=<toppkg>] [package]\n"
          "    cflags-only-I [--deps-only] [package]\n"
          "    cflags-only-other [--deps-only] [package]\n"
          "    libs-only-L [--deps-only] [package]\n"
          "    libs-only-l [--deps-only] [package]\n"
          "    libs-only-other [--deps-only] [package]\n"
          "    profile [--length=<length>] [--zombie-only]\n"
          "  Extra options:\n"
          "    -q     Quiets error reports.\n\n"
          " If [package] is omitted, the current working directory\n"
          " is used (if it contains a manifest.xml).\n\n";
}

Package *ROSPack::get_pkg(string pkgname)
{
  for (VecPkg::iterator p = Package::pkgs.begin(); 
       p != Package::pkgs.end(); ++p)
  {
    if ((*p)->name == pkgname)
    {
      if(!crawled)
      {
        // Answer come from the cache; check that the path is valid, and
        // contains a manifest (#1115).
        std::string manifest_path = (*p)->path + fs_delim + "manifest.xml";
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
          fprintf(stderr, "[rospack] warning: invalid cached location %s for package %s; forcing recrawl\n",
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
  if (!crawled) // maybe it's a brand-new package. force a crawl.
  {
    crawl_for_packages(true); // will set the crawled flag; recursion is safe
    return get_pkg(pkgname);
  }
  string errmsg = string("couldn't find package [") + pkgname + string("]");
  throw runtime_error(errmsg);
  return NULL; // or not
}
  
int ROSPack::cmd_depends_on(bool include_indirect)
{
  // We can't proceed if the argument-parsing logic wasn't able to provide
  // any package name.  Note that we need to check for an empty opt_package
  // here, but not in other places (e.g., cmd_deps()), because here we're
  // catching the exception that get_pkg() throws when it can't find the
  // package.  Elsewhere, we let that exception propagate up.
  if(opt_package.size() == 0)
  {
    string errmsg = string("no package name given, and current directory is not a package root");
    throw runtime_error(errmsg);
  }

  // Don't warn about missing deps
  opt_warn_on_missing_deps = false;

  // Explicitly crawl for packages, to ensure that we get newly added
  // dependent packages.  We also avoid the possibility of a recrawl
  // happening within the loop below, which could invalidate the pkgs 
  // vector as we loop over it.
  crawl_for_packages(true);

  Package* p;
  try
  {
    p = get_pkg(opt_package);
  }
  catch(runtime_error)
  {
    fprintf(stderr, "[rospack] warning: package %s doesn't exist\n", 
            opt_package.c_str());
    //p = new Package(opt_package);
    //Package::pkgs.push_back(p);
    p = add_package(opt_package);
  }
  assert(p);
  const VecPkg descendants = include_indirect ? p->descendants() 
          : p->descendants1();
  for (VecPkg::const_iterator p = descendants.begin(); 
       p != descendants.end(); ++p)
  {
    //printf("%s\n", (*p)->name.c_str());
    output_acc += (*p)->name + "\n";
  }
  return 0;
}

// Naive recursion.  Dynamic programming would be more efficient.
int ROSPack::cmd_depends_why()
{
  AccList acc_list;
  Package* from = get_pkg(opt_package);
  Package* to = get_pkg(opt_target);
  from->accumulate_deps(acc_list, to);
  printf("Dependency chains from %s to %s:\n", 
         from->name.c_str(), to->name.c_str());
  for(AccList::iterator lit = acc_list.begin();
      lit != acc_list.end();
      ++lit)
  {
    printf("* ");
    for(Acc::iterator ait = lit->begin();
        ait != lit->end();
        ++ait)
    {
      if(ait != lit->begin())
        printf("-> ");
      printf("%s ", (*ait)->name.c_str());
    }
    printf("\n");
  }
  return 0;
}

int ROSPack::cmd_find()
{
  // todo: obey the search order
  Package *p = get_pkg(opt_package);
  //printf("%s\n", p->path.c_str());
  output_acc += p->path + "\n";
  return 0;
}

int ROSPack::cmd_deps()
{
  VecPkg d = get_pkg(opt_package)->deps(Package::POSTORDER);
  for (VecPkg::iterator i = d.begin(); i != d.end(); ++i)
  {
    //printf("%s\n", (*i)->name.c_str());
    output_acc += (*i)->name + "\n";
  }
  return 0;
}

int ROSPack::cmd_deps_manifests()
{
  VecPkg d = get_pkg(opt_package)->deps(Package::POSTORDER);
  for (VecPkg::iterator i = d.begin(); i != d.end(); ++i)
  {
    //printf("%s/manifest.xml ", (*i)->path.c_str());
    output_acc += (*i)->path + "/manifest.xml ";
  }
  //puts("");
  output_acc += "\n";
  return 0;
}

int ROSPack::cmd_deps_msgsrv()
{
  VecPkg d = get_pkg(opt_package)->deps(Package::POSTORDER);
  for (VecPkg::iterator i = d.begin(); i != d.end(); ++i)
  {
    Package* p = *i;
    bool msg_exists = file_exists((p->path + "/msg_gen/generated").c_str());
    bool srv_exists = file_exists((p->path + "/srv_gen/generated").c_str());

    if (msg_exists)
    {
      output_acc += p->path + "/msg_gen/generated ";
    }

    if (srv_exists)
    {
      output_acc += p->path + "/srv_gen/generated ";
    }
  }
  output_acc += "\n";
  return 0;
}

int ROSPack::cmd_deps1()
{
  VecPkg d = get_pkg(opt_package)->deps1();
  for (VecPkg::iterator i = d.begin(); i != d.end(); ++i)
  {
    //printf("%s\n", (*i)->name.c_str());
    output_acc += (*i)->name + "\n";
  }
  return 0;
}

int ROSPack::cmd_depsindent(Package* pkg, int indent)
{
  VecPkg d = pkg->deps1();
  
  for (VecPkg::iterator i = d.begin(); i != d.end(); ++i)
  {
    for(int s=0; s<indent; s++)
    {
      //printf(" ");
      output_acc += " ";
    }
    //printf("%s\n", (*i)->name.c_str());
    output_acc += (*i)->name + "\n";
    cmd_depsindent(*i, indent+2);
  }
  return 0;
}

/*
int ROSPack::cmd_predeps(char **args, int args_len)
{
  if (args_len != 1)
    fprintf(stderr,"[rospack] usage: rospack predeps PACKAGE\n");
  else
  {
    VecPkg d = get_pkg(args[0])->deps(Package::PREORDER);
    for (VecPkg::iterator i = d.begin(); i != d.end(); ++i)
      printf("%s\n", (*i)->name.c_str());
  }
  return 0;
}
*/

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
      while (j != str.end() && *(j-1) == '\\')
        j = find_if(j+1, str.end(), space);
      ret.push_back(string(i, j));
    i = j;
  }
  return ret;
}

string ROSPack::snarf_libs(string flags, bool invert)
{
  vector<string> tokens = split_space(flags);
  string snarfed;
  for (size_t i = 0; i < tokens.size(); ++i)
  {
    //fprintf(stderr, "token = %s, len=%d, f=%c last=%s\n", tokens[i].c_str(), tokens[i].length(), tokens[i][0], tokens[i].substr(tokens[i].length()-2).c_str());
    if (invert)
    {
      if ((tokens[i].substr(0, 2) != "-l") &&
          (tokens[i].length() < 2 || tokens[i][0] != '/' || tokens[i].substr(tokens[i].length()-2) != ".a"))
        snarfed += (snarfed.length() ? " " : "" ) + tokens[i];
    }
    else
    {
      if (tokens[i].substr(0, 2) == "-l")
        snarfed += (snarfed.length() ? " " : "" ) + tokens[i].substr(2);
      else if (tokens[i].length() > 2 && tokens[i][0] == '/' && tokens[i].substr(tokens[i].length()-2) == ".a")
        snarfed += (snarfed.length() ? " " : "" ) + tokens[i];
    }
  }
  return snarfed;
}

string ROSPack::snarf_flags(string flags, string prefix, bool invert)
{
  vector<string> tokens = split_space(flags);
  string snarfed;
  for (size_t i = 0; i < tokens.size(); ++i)
  {
    if ((tokens[i].substr(0, prefix.length()) == prefix) ^ invert)
    {
      snarfed += (snarfed.length() ? " " : "" ) + tokens[i].substr(invert ? 0 : prefix.length());
    }
  }
  return snarfed;
}

int ROSPack::cmd_libs_only(string token)
{
  string lflags = get_pkg(opt_package)->flags("cpp", "lflags");;
  if(!token.compare("-other"))
  {
    lflags = snarf_libs(lflags, true);
    lflags = snarf_flags(lflags, "-L", true);
  }
  else if(!token.compare("-l"))
  {
    lflags = snarf_libs(lflags);
  }
  else
  {
    lflags = snarf_flags(lflags, token);
    lflags = deduplicate_tokens(lflags);
  }
  //printf("%s\n", lflags.c_str());
  output_acc += lflags + "\n";
  return 0;
}

int ROSPack::cmd_cflags_only(string token)
{
  string cflags = get_pkg(opt_package)->flags("cpp", "cflags");
  if(!token.compare("-other"))
    cflags = snarf_flags(cflags, "-I", true);
  else
  {
    cflags = snarf_flags(cflags, token);
    cflags = deduplicate_tokens(cflags);
  }
  //printf("%s\n", cflags.c_str());
  output_acc += cflags + "\n";
  return 0;
}

void ROSPack::export_flags(string pkg, string lang, string attrib)
{
  string flags = get_pkg(pkg)->flags(lang, attrib);
  //printf("%s\n", flags.c_str());
  output_acc += flags + "\n";
}

int ROSPack::cmd_versioncontrol(int depth)
{
  string sds;

  sds += get_pkg(opt_package)->versioncontrol();

  if(depth < 0)
  {
    VecPkg descs = get_pkg(opt_package)->deps(Package::POSTORDER);
    for(VecPkg::iterator dit = descs.begin();
        dit != descs.end();
        dit++)
    {
      sds += (*dit)->versioncontrol();
    }
  }

  //printf("%s", sds.c_str());
  output_acc += sds;
  return 0;
}

int ROSPack::cmd_rosdep(int depth)
{
  string sds;
  sds += get_pkg(opt_package)->rosdep();

  if(depth < 0)
  {
    VecPkg descs = get_pkg(opt_package)->deps(Package::POSTORDER);
    for(VecPkg::iterator dit = descs.begin();
        dit != descs.end();
        dit++)
    {
      sds += (*dit)->rosdep();
    }
  }

  //printf("%s", sds.c_str());
  output_acc += sds;
  return 0;
}

int ROSPack::cmd_export()
{
  export_flags(opt_package, opt_lang, opt_attrib);
  return 0;
}

int ROSPack::cmd_plugins()
{
  // Don't warn about missing deps
  opt_warn_on_missing_deps = false;
  
  Package* p = get_pkg(opt_package);

  vector<pair<string, string> > plugins = p->plugins();
  vector<pair<string, string> >::iterator it = plugins.begin();
  vector<pair<string, string> >::iterator end = plugins.end();
  for (; it != end; ++it)
  {
    //printf("%s %s\n", it->first.c_str(), it->second.c_str());
    output_acc += it->first + " " + it->second + "\n";
  }

  return 0;
}

void ROSPack::freeArgv()
{
  if(my_argc)
  {
    for(int i=0;i<my_argc;i++)
      free(my_argv[i]);
    free(my_argv);
  }
  my_argc = 0;
  my_argv = NULL;
}

int ROSPack::run(const std::string& cmd)
{
  std::vector<std::string> cmd_list;
  
  // TODO: split the input string properly, accounting for quotes, escaped
  // quotes, etc.  This shouldn't really matter, because rospack shouldn't be
  // given arguments with embedded spaces.
  string_split(cmd, cmd_list, " ");

  // In case we're called more than once.
  freeArgv();

  my_argc = (int)cmd_list.size() + 1;
  my_argv = (char**)malloc(sizeof(char*) * my_argc);
  my_argv[0] = strdup("rospack");
  for(int i=1;i<my_argc;i++)
    my_argv[i] = strdup(cmd_list[i-1].c_str());

  return run(my_argc, my_argv);
}

int ROSPack::run(int argc, char **argv)
{
  assert(argc >= 2);
  int i;
  const char* opt_deps_name    = "--deps-only";
  const char* opt_zombie_name  = "--zombie-only";
  const char* opt_lang_name    = "--lang=";
  const char* opt_attrib_name  = "--attrib=";
  const char* opt_length_name  = "--length=";
  const char* opt_top_name     = "--top=";
  const char* opt_target_name  = "--target=";
  const char* opt_quiet_name   = "-q";

  // Reset to defaults.
  opt_deps_only = false;
  // --lang=
  opt_lang = string("");
  // --attrib=
  opt_attrib = string("");
  // --length=
  opt_length = string("");
  // --top=
  opt_top = string("");
  // --target=
  opt_target = string("");
  // The package name
  opt_package = string("");
  // the number of entries to list in the profile table
  opt_profile_length = 0;
  // only display zombie directories in profile?
  opt_profile_zombie_only = false;
  // warn on missing deps
  opt_warn_on_missing_deps = true;
  // don't display duplicate pkgs
  opt_display_duplicate_pkgs = false;

  output_acc = string("");

  string errmsg = string(usage());

  i=1;
  const char* cmd = argv[i++];

  for(;i<argc;i++)
  {
    if(!strcmp(argv[i], opt_deps_name))
      opt_deps_only=true;
    else if(!strcmp(argv[i], opt_zombie_name))
      opt_profile_zombie_only=true;
    else if(!strcmp(argv[i], opt_quiet_name))
      opt_quiet=true;
    else if(!strncmp(argv[i], opt_target_name, strlen(opt_target_name)))
    {
      if(opt_target.size())
        throw runtime_error(errmsg);
      else if(strlen(argv[i]) > strlen(opt_target_name))
        opt_target = string(argv[i]+strlen(opt_target_name));
      else
        throw runtime_error(errmsg);
    }
    else if(!strncmp(argv[i], opt_lang_name, strlen(opt_lang_name)))
    {
      if(opt_lang.size())
        throw runtime_error(errmsg);
      else if(strlen(argv[i]) > strlen(opt_lang_name))
        opt_lang = string(argv[i]+strlen(opt_lang_name));
      else
        throw runtime_error(errmsg);
    }
    else if(!strncmp(argv[i], opt_attrib_name, strlen(opt_attrib_name)))
    {
      if(opt_attrib.size())
        throw runtime_error(errmsg);
      else if(strlen(argv[i]) > strlen(opt_attrib_name))
        opt_attrib = string(argv[i]+strlen(opt_attrib_name));
      else
        throw runtime_error(errmsg);
    }
    else if(!strncmp(argv[i], opt_length_name, strlen(opt_length_name)))
    {
      if(strlen(argv[i]) > strlen(opt_length_name))
        opt_length = string(argv[i]+strlen(opt_length_name));
      else
        throw runtime_error(errmsg);
    }
    else if(!strncmp(argv[i], opt_top_name, strlen(opt_top_name)))
    {
      if(strlen(argv[i]) > strlen(opt_top_name))
        opt_top = string(argv[i]+strlen(opt_top_name));
      else
        throw runtime_error(errmsg);
    }
    else
      break;
  }

  if((strcmp(cmd, "depends-why") && strcmp(cmd, "deps-why")) && opt_target.size())
    throw runtime_error(errmsg);

  if((!strcmp(cmd, "depends-why") || !strcmp(cmd, "deps-why")) && !opt_target.size())
    throw runtime_error(errmsg);
  
  if(strcmp(cmd, "profile") && (opt_length.size() || opt_profile_zombie_only))
    throw runtime_error(errmsg);
  
  // --top= is only valid for plugins
  if(strcmp(cmd, "plugins") && opt_top.size())
    throw runtime_error(errmsg);

  // --attrib= is only valid for export and plugins
  if((strcmp(cmd, "export") && strcmp(cmd, "plugins")) && 
     opt_attrib.size())
    throw runtime_error(errmsg);

  // --lang= is only valid for export
  if((strcmp(cmd, "export") && opt_lang.size()))
    throw runtime_error(errmsg);
  
  // export requires both --lang and --attrib
  if(!strcmp(cmd, "export") && (!opt_lang.size() || !opt_attrib.size()))
    throw runtime_error(errmsg);
    
  // plugins requires --attrib
  if(!strcmp(cmd, "plugins") && !opt_attrib.size())
    throw runtime_error(errmsg);

  if(opt_deps_only && 
     strcmp(cmd, "export") &&
     strcmp(cmd, "cflags-only-I") &&
     strcmp(cmd, "cflags-only-other") &&
     strcmp(cmd, "libs-only-L") &&
     strcmp(cmd, "libs-only-l") &&
     strcmp(cmd, "libs-only-other"))
    throw runtime_error(errmsg);

  if(i < argc)
  {
    if(!strcmp(cmd, "help") ||
       !strcmp(cmd, "list") ||
       !strcmp(cmd, "list-names") ||
       !strcmp(cmd, "list-duplicates") ||
       !strcmp(cmd, "langs") ||
       !strcmp(cmd, "profile"))
      throw runtime_error(errmsg);

    opt_package = string(argv[i++]);
  }
  // Are we sitting in a package?
  else 
  {
    char buf[1024];
    if(getcwd(buf,sizeof(buf)))
    {
      if(Package::is_package("."))
        opt_package = string(basename(buf));
    }
  }

  if (i != argc)
    throw runtime_error(errmsg);

  if (!strcmp(cmd, "profile"))
  {
    if (opt_length.size())
      opt_profile_length = atoi(opt_length.c_str());
    else
    {
      if(opt_profile_zombie_only)
        opt_profile_length = -1; // default is infinite
      else
        opt_profile_length = 20; // default is about a screenful or so
    }
#ifdef VERBOSE_DEBUG
    printf("profile_length = %d\n", opt_profile_length);
#endif
    // re-crawl with profiling enabled
    crawl_for_packages(true);
    return 0;
  }
  else if (!strcmp(cmd, "find"))
    return cmd_find();
  else if (!strcmp(cmd, "list"))
    return cmd_print_package_list(true);
  else if (!strcmp(cmd, "list-names"))
    return cmd_print_package_list(false);
  else if (!strcmp(cmd, "list-duplicates"))
    return cmd_list_duplicates();
  else if (!strcmp(cmd, "langs"))
    return cmd_print_langs_list();
  else if (!strcmp(cmd, "depends") || !strcmp(cmd, "deps"))
    return cmd_deps();
  else if (!strcmp(cmd, "depends-manifests") || !strcmp(cmd, "deps-manifests"))
    return cmd_deps_manifests();
  else if (!strcmp(cmd, "depends-msgsrv") || !strcmp(cmd, "deps-msgsrv"))
      return cmd_deps_msgsrv();
  else if (!strcmp(cmd, "depends1") || !strcmp(cmd, "deps1"))
    return cmd_deps1();
  else if (!strcmp(cmd, "depends-indent") || !strcmp(cmd, "deps-indent"))
    return cmd_depsindent(get_pkg(opt_package), 0);
  else if (!strcmp(cmd, "depends-on"))
    return cmd_depends_on(true);
  else if (!strcmp(cmd, "depends-why") || !strcmp(cmd, "deps-why"))
    return cmd_depends_why();
  else if (!strcmp(cmd, "depends-on1"))
    return cmd_depends_on(false);
  /*
  else if (!strcmp(argv[i], "predeps"))
    return cmd_predeps(argv+i+1, argc-i-1);
    */
  else if (!strcmp(cmd, "export"))
    return cmd_export();
  else if (!strcmp(cmd, "plugins"))
    return cmd_plugins();
  else if (!strcmp(cmd, "rosdep") || !strcmp(cmd, "rosdeps"))
    return cmd_rosdep(-1);
  else if (!strcmp(cmd, "rosdep0") || !strcmp(cmd, "rosdeps0"))
    return cmd_rosdep(0);
  else if (!strcmp(cmd, "vcs"))
    return cmd_versioncontrol(-1);
  else if (!strcmp(cmd, "vcs0"))
    return cmd_versioncontrol(0);
  else if (!strcmp(cmd, "libs-only-l"))
    return cmd_libs_only("-l");
  else if (!strcmp(cmd, "libs-only-L"))
    return cmd_libs_only("-L");
  else if (!strcmp(cmd, "libs-only-other"))
    return cmd_libs_only("-other");
  else if (!strcmp(cmd, "cflags-only-I"))
    return cmd_cflags_only("-I");
  else if (!strcmp(cmd, "cflags-only-other"))
    return cmd_cflags_only("-other");
  else if (!strcmp(cmd, "help"))
    fputs(usage(), stderr);
  else
  {
    throw runtime_error(errmsg);
  }
  return 0;
}

int ROSPack::cmd_print_package_list(bool print_path)
{
  for (VecPkg::iterator i = Package::pkgs.begin(); 
       i != Package::pkgs.end(); ++i)
    if (print_path)
    {
      //printf("%s %s\n", (*i)->name.c_str(), (*i)->path.c_str());
      output_acc += (*i)->name + " " + (*i)->path + "\n";
    }
    else
    {
      //printf("%s\n", (*i)->name.c_str());
      output_acc += (*i)->name + "\n";
    }
  return 0;
}

int ROSPack::cmd_list_duplicates()
{
  // Force crawl, noting duplicates
  opt_display_duplicate_pkgs = true;
  crawl_for_packages(true);
  // If duplicates were found, return non-zero, because in this mode we
  // consider that to be an error.
  if(duplicate_packages_found)
    return 1;
  else
    return 0;
}  

int ROSPack::cmd_print_langs_list()
{
  // Don't warn about missing deps
  opt_warn_on_missing_deps = false;

  // Check for packages that depend directly on roslang
  VecPkg lang_pkgs;
  Package* roslang;
  
  roslang = get_pkg("roslang");
  assert(roslang);

  lang_pkgs = roslang->descendants1();
  
  // Filter out packages mentioned in ROS_LANG_DISABLE
  char *disable = getenv("ROS_LANG_DISABLE");
  vector<string> disable_list;
  if(disable)
    string_split(disable, disable_list, ":");

  for(VecPkg::const_iterator i = lang_pkgs.begin();
      i != lang_pkgs.end();
      ++i)
  {
    vector<string>::const_iterator j;
    for(j = disable_list.begin();
        j != disable_list.end();
        ++j)
    {
      if((*j) == (*i)->name)
        break;
    }
    if(j == disable_list.end())
    {
      //printf("%s ", (*i)->name.c_str());
      output_acc += (*i)->name + " ";
    }
  }
  //printf("\n");
  output_acc += "\n";
  return 0;
}

string ROSPack::getCachePath()
{
  string cache_file_name;
  char* ros_home = getenv("ROS_HOME");

  if (ros_home)
  {
    // Create ROS_HOME if it doesn't exist, #2812.
    //
    // By providing the trailing slash, stat() will only succeed if the
    // path exists AND is a directory.
    std::string ros_home_slash = ros_home + std::string("/");
    struct stat s;
    if(stat(ros_home_slash.c_str(), &s))
    {
      if(mkdir(ros_home_slash.c_str(), 0700) != 0)
      {
	perror("[rospack] WARNING: cannot create rospack cache directory");
      }
    }
    cache_file_name = ros_home_slash + std::string("rospack_cache");
  }
  else
  {
    // Not cross-platform?
    ros_home = getenv("HOME");
    if (ros_home)
    {
      // By providing the trailing slash, stat() will only succeed if the
      // path exists AND is a directory.
      std::string dotros = ros_home + std::string("/.ros/");
      struct stat s;
      if(stat(dotros.c_str(), &s))
      {
        if(mkdir(dotros.c_str(), 0700) != 0)
	  perror("[rospack] WARNING: cannot create rospack cache directory");
      }
      cache_file_name = dotros + "rospack_cache";
    }
  }
  return cache_file_name;
}

bool ROSPack::cache_is_good()
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
  for(;;)
  {
    if (!fgets(linebuf, sizeof(linebuf), cache))
      break;
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
  size_t start_num_pkgs;
  bool has_manifest;
  CrawlQueueEntry(string _path) 
  : path(_path), start_time(0), elapsed_time(0), 
        start_num_pkgs(0), has_manifest(false){ }
  bool operator>(const CrawlQueueEntry &rhs) const
  {
    return elapsed_time > rhs.elapsed_time;
  }
};
  
double ROSPack::time_since_epoch()
{
  struct timeval tod;
  gettimeofday(&tod, NULL);
  return tod.tv_sec + 1e-6 * tod.tv_usec;
}

// Add package, filtering out duplicates.
Package* ROSPack::add_package(string path)
{
  // Filter out duplicates; first encountered takes precedence
  Package* newp = new Package(path);
  Package* return_p = newp;
  // TODO: make this check more efficient
  bool dup = false;
  for(std::vector<Package *>::const_iterator it = Package::pkgs.begin();
      it != Package::pkgs.end();
      it++)
  {
    if((*it)->name == newp->name)
    {
      dup=true;
      return_p = *it;
      // If we're supposed to display info on dups, do it now
      if(opt_display_duplicate_pkgs)
        output_acc += (*it)->path + " " + newp->path + "\n";
      // Note that we've encountered a duplicate
      duplicate_packages_found = true;
      break;
    }
  }
  if(dup)
    delete newp;
  else
    Package::pkgs.push_back(newp);

  return return_p;
}

void ROSPack::crawl_for_packages(bool force_crawl)
{
  for (VecPkg::iterator p = Package::pkgs.begin(); 
       p != Package::pkgs.end(); ++p)
    Package::deleted_pkgs.push_back(*p);
  Package::pkgs.clear();

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
      for(;;)
      {
        if (!fgets(linebuf, sizeof(linebuf), cache))
          break; // error in read operation
        if (linebuf[0] == '#')
          continue;
        char *newline_pos = strchr(linebuf, '\n');
        if (newline_pos)
          *newline_pos = 0;
        //Package::pkgs.push_back(new Package(linebuf));
        add_package(linebuf);
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
  if (char *rpp = getenv("ROS_PACKAGE_PATH"))
  {
    vector<string> rppvec;
    string_split(rpp, rppvec, ":");
    sanitize_rppvec(rppvec);
    for (vector<string>::iterator i = rppvec.begin(); i != rppvec.end(); ++i)
    {
      if(!i->size())
        continue;
      else if (!Package::is_package(*i) && Package::is_no_subdirs(*i))
        fprintf(stderr, "[rospack] WARNING: non-package directory in "
                "ROS_PACKAGE_PATH marked rospack_nosubdirs:\n\t%s\n",
                i->c_str());
      else
        q.push_back(CrawlQueueEntry(*i));
    }
  }
  const double crawl_start_time = time_since_epoch();
  priority_queue<CrawlQueueEntry, vector<CrawlQueueEntry>, 
                 greater<CrawlQueueEntry> > profile;
  while (!q.empty())
  {
    CrawlQueueEntry cqe = q.front();
    q.pop_front();
    
    // Check whether this part of ROS_PACKAGE_PATH is itself a package
    if (Package::is_package(cqe.path))
    {
      //Package::pkgs.push_back(new Package(*i));
      add_package(cqe.path);
      continue;
    }

    //printf("crawling %s\n", cqe.path.c_str());
    if (opt_profile_length != 0)
    {
      if (cqe.start_time != 0)
      {
        // this stack symbol means we've already crawled its children, and it's
        // just here for timing purposes. 

        // save the traversal time 
        cqe.elapsed_time = time_since_epoch() - cqe.start_time;

        // Did the number of packages increase since we started crawling
        // this directory's children?  If not, then this is likely a zombie
        // directory that should probably be deleted.  We'll mark it as
        // such in the profile console output.
        if(cqe.start_num_pkgs < total_num_pkgs)
          cqe.has_manifest = true;
        if(!opt_profile_zombie_only || !cqe.has_manifest)
        {
          profile.push(cqe);
          if ((opt_profile_length > 0) && (profile.size() > opt_profile_length)) // only save the worst guys
            profile.pop();
        }
        continue;
      }
      cqe.start_time = time_since_epoch();
      cqe.start_num_pkgs = total_num_pkgs;
      q.push_front(cqe);
    }
    DIR *d = opendir(cqe.path.c_str());
    if (!d)
    {
      fprintf(stderr, "[rospack] opendir error [%s] while crawling %s\n", 
              strerror(errno), cqe.path.c_str());
      continue;
    }
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL)
    {
      struct stat s;
      string child_path = cqe.path + fs_delim + string(ent->d_name);
      int ret;
      while ((ret = stat(child_path.c_str(), &s)) != 0 &&
             errno == EINTR);
      if (ret != 0)
        continue;
      if (!S_ISDIR(s.st_mode)) 
        continue;
      if (ent->d_name[0] == '.')
        continue; // ignore hidden dirs
      if (Package::is_package(child_path))
      {
        total_num_pkgs++;
        add_package(child_path);
      }
      //check to make sure we're allowed to descend
      else if (!Package::is_no_subdirs(child_path)) 
        q.push_front(CrawlQueueEntry(child_path));
    }
    closedir(d);
  }
  crawled = true; // don't try to re-crawl if we can't find something
  const double crawl_elapsed_time = time_since_epoch() - crawl_start_time;
  // Write the results of this crawl to the cache file.  At each step, give
  // up on error, printing a warning to stderr.
  string cache_path(getCachePath());
  if(!cache_path.size())
  {
    fprintf(stderr, "[rospack] No location available to write cache file.  Try setting ROS_HOME or HOME.\n");
  }
  else
  {
    char tmp_cache_dir[PATH_MAX];
    char tmp_cache_path[PATH_MAX];
    strncpy(tmp_cache_dir, cache_path.c_str(), sizeof(tmp_cache_dir));
    snprintf(tmp_cache_path, sizeof(tmp_cache_path), "%s/.rospack_cache.XXXXXX", dirname(tmp_cache_dir));
    int fd = mkstemp(tmp_cache_path);
    if (fd < 0)
    {
      fprintf(stderr, "[rospack] Unable to create temporary cache file %s: %s\n", 
              tmp_cache_path, strerror(errno));
    }
    else
    {
      FILE *cache = fdopen(fd, "w");
      if (!cache)
      {
        fprintf(stderr, "[rospack] Unable open cache file %s: %s\n", 
                tmp_cache_path, strerror(errno));
      }
      else
      {
        char *rpp = getenv("ROS_PACKAGE_PATH");
        fprintf(cache, "#ROS_ROOT=%s\n#ROS_PACKAGE_PATH=%s\n", ros_root,
                (rpp ? rpp : ""));
        for (VecPkg::iterator pkg = Package::pkgs.begin();
             pkg != Package::pkgs.end(); ++pkg)
          fprintf(cache, "%s\n", (*pkg)->path.c_str());
        fclose(cache);
        if(rename(tmp_cache_path, cache_path.c_str()) < 0)
        {
          fprintf(stderr, "[rospack] Error: failed to rename cache file %s to %s: %s\n", 
                  tmp_cache_path, cache_path.c_str(), strerror(errno));
        }
      }
    }
  }

  if (opt_profile_length)
  {
    // dump it into a stack to reverse it (so slowest guys are first)
    stack<CrawlQueueEntry> reverse_profile;
    // Also build up a separate list of paths that will be used to remove
    // children of zombies.
    vector<string> zombie_dirs;
    zombie_dirs.reserve(profile.size());
    while (!profile.empty())
    {
      reverse_profile.push(profile.top());
      zombie_dirs.push_back(profile.top().path);
      profile.pop();
    }
    if(!opt_profile_zombie_only)
    {
      //printf("\nFull tree crawl took %.6f seconds.\n", crawl_elapsed_time);
      //printf("Directories marked with (*) contain no manifest.  You may\n");
      //printf("want to delete these directories.\n");
      //printf("-------------------------------------------------------------\n");
      char buf[16];
      snprintf(buf, sizeof(buf), "%.6f", crawl_elapsed_time);
      output_acc += "\nFull tree crawl took " + string(buf) + " seconds.\n";
      output_acc += "Directories marked with (*) contain no manifest.  You may\n";
      output_acc += "want to delete these directories.\n";
      output_acc += "To get just of list of directories without manifests,\n";
      output_acc += "re-run the profile with --zombie-only\n.";
      output_acc += "-------------------------------------------------------------\n";
    }
    while (!reverse_profile.empty())
    {
      CrawlQueueEntry cqe = reverse_profile.top();
      reverse_profile.pop();

      if(!opt_profile_zombie_only)
      {
        //printf("%.6f %s %s\n", 
               //cqe.elapsed_time, 
               //cqe.has_manifest ? " " : "*",
               //cqe.path.c_str());
        char buf[16];
        snprintf(buf, sizeof(buf), "%.6f", cqe.elapsed_time);
        output_acc += string(buf) + " ";
        if(cqe.has_manifest)
          output_acc += "  ";
        else
          output_acc += "* ";
        output_acc += cqe.path;
        output_acc += "\n";
      }
      else
      {
        bool dup = false;
        // Does this directory contain a stack.xml or app.xml?  In that
        // case, it's an empty stack or app, and should not be considered a
        // zombie.
        if(file_exists(cqe.path + fs_delim + "stack.xml") ||
           file_exists(cqe.path + fs_delim + "app.xml"))
        {
          continue;
        }
        //
        // Does this entry's parent appear in the list?
        for(vector<string>::const_iterator it = zombie_dirs.begin();
            it != zombie_dirs.end();
            ++it)
        {
          if((cqe.path.size() > it->size()) &&
             (cqe.path.substr(0,it->size()) == (*it)))
          {
            dup = true;
            break;
          }
        }
        if(dup)
          continue;

        //printf("%s\n", cqe.path.c_str());
        output_acc += cqe.path + "\n";
      }
    }
    if(!opt_profile_zombie_only)
    {
      //printf("\n");
      output_acc += "\n";
    }
  }
}

VecPkg ROSPack::partial_crawl(const string &path)
{
  deque<CrawlQueueEntry> q;
  q.push_back(CrawlQueueEntry(path));
  VecPkg partial_pkgs;
  while (!q.empty())
  {
    CrawlQueueEntry cqe = q.front();
    //printf("crawling %s\n", cqe.path.c_str());
    q.pop_front();
    DIR *d = opendir(cqe.path.c_str());
    if (!d)
    {
      fprintf(stderr, "[rospack] opendir error [%s] while crawling %s\n", 
              strerror(errno), cqe.path.c_str());
      continue;
    }
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL)
    {
      struct stat s;
      string child_path = cqe.path + fs_delim + string(ent->d_name);
      int ret;
      while ((ret = stat(child_path.c_str(), &s)) != 0 && 
             errno == EINTR);
      if (ret != 0)
        continue;
      if (!S_ISDIR(s.st_mode)) 
        continue;
      if (ent->d_name[0] == '.')
        continue; // ignore hidden dirs
      if (Package::is_package(child_path))
      {
        // Filter out duplicates; first encountered takes precedence
        Package* newp = new Package(child_path);
        // TODO: make this check more efficient
        bool dup = false;
        for(std::vector<Package *>::const_iterator it = partial_pkgs.begin();
            it != partial_pkgs.end();
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
          partial_pkgs.push_back(newp);
      }
      //check to make sure we're allowed to descend
      else if (!Package::is_no_subdirs(child_path)) 
        q.push_front(CrawlQueueEntry(child_path));
    }
    closedir(d);
  }
  return partial_pkgs; 
}

//////////////////////////////////////////////////////////////////////////////

void string_split(const string &s, vector<string> &t, const string &d)
{
  t.clear();
  size_t start = 0, end;
  while ((end = s.find_first_of(d, start)) != string::npos)
  {
    if((end-start) > 0)
      t.push_back(s.substr(start, end-start));
    start = end + 1;
  }
  if(start < s.size())
    t.push_back(s.substr(start));
}

// Produce a new string by keeping only the first of each repeated token in
// the input string, where tokens are space-separated.  I'm sure that Rob
// could point me at the Boost/STL one-liner that does the same thing.
string ROSPack::deduplicate_tokens(const string& s)
{
  vector<string> in;
  vector<string> out;
  string_split(s, in, " ");
  for(int i=0; i<in.size(); i++)
  {
    bool dup = false;
    for(int j=0; j<out.size(); j++)
    {
      if(!out[j].compare(in[i]))
      {
        dup = true;
        break;
      }
    }
    if(!dup)
      out.push_back(in[i]);
  }

  string res;
  for(int j=0; j<out.size(); j++)
  {
    if(!j)
      res += out[j];
    else
      res += string(" ") + out[j];
  }

  return res;
}

bool file_exists(const string &fname)
{
  // this will be different in windows
  return (access(fname.c_str(), F_OK) == 0);
}

Package *g_get_pkg(const string &name)
{
  // a hack... but I'm lazy and love single-file programs
  return g_rospack->get_pkg(name);
}

void ROSPack::sanitize_rppvec(std::vector<std::string> &rppvec)
{
  // drop any trailing slashes
  for (size_t i = 0; i < rppvec.size(); i++)
  {
    size_t last_slash_pos;
    while((last_slash_pos = rppvec[i].find_last_of("/")) == rppvec[i].length()-1)
    {
      fprintf(stderr, "[rospack] warning: trailing slash found in "
                      "ROS_PACKAGE_PATH\n");
      rppvec[i].erase(last_slash_pos);
    }
  }
}

}
