// copyright morgan quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#include <stdexcept>
#include <string>
#include <list>
#include <set>
#include <map>
#include <vector>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sys/wait.h>
#include "rospack/rospack.h"
#include "rospack/rosstack.h"
#include "tinyxml-2.5.3/tinyxml.h"
#include "yaml.h"

using std::string;
using std::vector;
using std::list;
using std::set;
using std::map;
using std::runtime_error;

using namespace rospack;
//using namespace rosdep;

const char *COLOR_NORMAL = "\033[0m";
const char *COLOR_RED    = "\033[31m";
const char *COLOR_GREEN  = "\033[32m";
const char *COLOR_YELLOW = "\033[33m";

int usage()
{
  printf("\nusage: rosdep COMMAND [ARGS]\n\n"
         "  available commands:\n\n"
         "rosdep generate_bash PACKAGE [PACKAGE2, ...]  \n"
         "rosdep satisfy PACKAGE [PACKAGE2, ...]  \n"
         "  will try to generate a bash script which will satisfy the \n"
         "  dependencies of PACKAGE[S] on your operating system.\n\n"
         "rosdep install PACKAGE [PACKAGE2, ...]\n"
         "  will generate a bash script and then execute it.\n\n"
         "rosdep depdb PACKAGE [PACKAGE2, ...]\n"
         "  will generate the dependency database for PACKAGE[S] and print\n"
         "  it to the console (note that the database will change depending\n"
         "  on which package(s) you query.\n\n"
         "rosdep what_needs ROSDEP_NAME[S]\n"
         "  will print a list of packages that declare a rosdep on (at least\n"
         "  one of) ROSDEP_NAME[S]\n\n"
         "rosdep check PACKAGE[S]\n"
         "  will check if the dependencies of PACKAGE[S] have been met.\n\n");
  return 0;
}

class Sat
{
public:
  string os_name, os_ver, rosdep_name;
  vector<string> pkgs; // packages for native package manager to install
  string bash; // non-zero length if sat is done via bash
};

class Dep
{
public:
  string yaml_path;
  vector<Sat> sats;
  bool from_old_db;
  Dep() : from_old_db(false) { }
};

class ROSDep
{
public:
  typedef map<string, Dep> DepMap;
  DepMap deps;
  bool debug;

  ROSDep() : debug(false) { }

  int cmd_depdb(vector<string> pkg_names)
  {
    safe_load(pkg_names);
    print_depdb();
    return 0;
  }

  int cmd_detect_os()
  {
    ensure_detect_os();
    printf("%s %s\n", os_name.c_str(), os_ver.c_str());
    return 0;
  }

  int cmd_install(vector<string> pkg_names)
  {
    rospack::Package *rp_pkg = rs.rp.get_pkg("rosdep");
    if (!rp_pkg)
    {
      fprintf(stderr, "rosdep: couldn't find my own package directory!\n");
      return 1;
    }
    string cmd = rp_pkg->path + string("/scripts/satisfy ");
    for (vector<string>::iterator i = pkg_names.begin();
         i != pkg_names.end(); ++i)
      cmd += *i + string(" ");
    int ret = system(cmd.c_str());
    int rc = WEXITSTATUS(ret);
    return rc;
  }

  int cmd_what_needs(vector<string> rosdep_names)
  {
    // crawl the entire ros tree and see who needs a given rosdep
    for (vector<Package *>::iterator p = rospack::Package::pkgs.begin();
         p != rospack::Package::pkgs.end(); ++p)
    {
      //printf("searching %s\n", (*pkg_it)->name.c_str());
      for (TiXmlElement *d = (*p)->manifest_root()->FirstChildElement("rosdep");
           d; d = d->NextSiblingElement("rosdep"))
      {
        if (!d->Attribute("name"))
          throw runtime_error(string("rosdep tag in ") + (*p)->path +
                              string("/manifest.xml") +
                              string(" has no 'name' attribute"));
        const char *dep_name = d->Attribute("name");
        for (vector<string>::iterator s = rosdep_names.begin();
             s != rosdep_names.end(); ++s)
          if (string(dep_name) == *s)
            printf("%s\n", (*p)->name.c_str());
      }
    }
    return 0;
  }

  int cmd_check(vector<string> pkg_names)
  {
    if (pkg_names.size() == 0)
      return 0;
    ensure_detect_os();
    safe_load(pkg_names);
    map<string, vector<string> > deps_with_owners;
    set<string> rosdeps_only;
    for (set<Package *>::iterator p = dep_pkgs.begin();
         p != dep_pkgs.end(); ++p)
    {
      for (TiXmlElement *d = (*p)->manifest_root()->FirstChildElement("rosdep");
           d; d = d->NextSiblingElement("rosdep"))
      {
        if (!d->Attribute("name"))
          throw runtime_error(string("rosdep tag in ") + (*p)->path +
                              string("/manifest.xml") +
                              string(" has no 'name' attribute"));
        string dep_name = string(d->Attribute("name"));
        deps_with_owners[dep_name].push_back((*p)->name);
        rosdeps_only.insert(string(dep_name));
      }
    }
    if (debug)
    {
      printf("rosdeps:\n");
      for (map<string, vector<string> >::iterator i = deps_with_owners.begin();
           i != deps_with_owners.end(); ++i)
      {
        printf("  %s:\n", i->first.c_str());
        for (vector<string>::iterator j = i->second.begin();
             j != i->second.end(); ++j)
          printf("    %s\n", j->c_str());
      }
      printf("\n\n");
    }
    list <Sat *> sats;
    try
    {
      sats = rosdeps_to_sats(rosdeps_only);
    }
    catch (...)
    {
      return 1;
    }
    bool anything_missing = false;
    for (list <Sat *>::iterator sat_it = sats.begin(); 
         sat_it != sats.end(); ++sat_it)
    {
      for (vector<string>::iterator pkg_it = (*sat_it)->pkgs.begin();
           pkg_it != (*sat_it)->pkgs.end(); ++pkg_it)
      {
        string cmd = package_check_condition(*pkg_it);
        int ret = system((cmd + string(" 2>/dev/null >/dev/null ")).c_str());
        int rc = WEXITSTATUS(ret);
        if (rc)
        {
          if (!anything_missing)
          {
            anything_missing = true;
            printf("Missing packages:\n");
            printf("%-20s rosdep               ROS package\n",
                   (os_name + string(" package")).c_str());
            printf("=======================================================\n");
          }
          printf("%-20s %-20s %-20s\n", 
                 (*pkg_it).c_str(),
                 (*sat_it)->rosdep_name.c_str(),
                 deps_with_owners[(*sat_it)->rosdep_name][0].c_str());
        }
      }
    }
    return (anything_missing ? 1 : 0);
  }

  list <Sat *> rosdeps_to_sats(set<string> rosdeps)
  {
    list<Sat *> sats;
    for (set<string>::iterator i = rosdeps.begin(); i != rosdeps.end(); ++i)
    {
      //printf("  trying to satisfy %s\n", i->c_str());
      DepMap::iterator dep_it = deps.find(*i);
      if (dep_it == deps.end())
      {
        //throw runtime_error(string("couldn't find a rosdep key for ") + *i);
        fprintf(stderr, 
                "rosdep: %sunable to find a definition for rosdep %s%s\n"
                "        after searching these files:\n", 
                COLOR_RED, i->c_str(), COLOR_NORMAL);
        for (vector<string>::iterator yaml_it = yaml_files_found.begin();
             yaml_it != yaml_files_found.end(); yaml_it++)
          fprintf(stderr, "           %s\n", yaml_it->c_str());
        fprintf(stderr, "        looked for, but did not find, files in\n");
        for (vector<string>::iterator yaml_it = yaml_files_not_found.begin();
             yaml_it != yaml_files_not_found.end(); yaml_it++)
          fprintf(stderr, "           %s\n", yaml_it->c_str());
        throw runtime_error("woah");
      }
      // now, see if we can match for our OS
      bool satisfied = false;
      for (vector<Sat>::iterator sat = dep_it->second.sats.begin();
           sat != dep_it->second.sats.end(); ++sat)
      {
        if (sat->os_name == os_name &&
            (sat->os_ver == os_ver || sat->os_ver.empty() || os_ver.empty()))
        {
          satisfied = true;
          sats.push_back(&(*sat));
          if (dep_it->second.from_old_db)
          {
            fprintf(stderr, "WARNING: using rosdep '%s', which was only found to "
                            "be satisfied in the 'old' "
                            "global database file in the rosdep package. "
                            "The 'new' idea is to place rosdep.yaml "
                            "files in individual stacks. Stacks inherit the "
                            "rosdep rules defined in their dependency tree, " 
                            "so a rosdep rule only needs to be defined once "
                            "in the stack tree -- in fact, multiple definitions "
                            "are inherently ambiguous, so rosdep will crash "
                            "if it finds one. The solution is to define a "
                            "rosdep rule as low in the tree as possible, but "
                            "no lower...\n", i->c_str());
          }
        }
      }      
      if (!satisfied)
      {
        fprintf(stderr, 
                "rosdep: couldn't find a way to satisfy rosdep [%s] on\n"
                "OS/distro [%s] version [%s]. This rosdep was defined in\n"
                "%s   If you are able to edit that file so the rosdep can\n"
                "be satsified on your OS/distro, please send us a patch!\n\n",
                dep_it->first.c_str(),
                os_name.c_str(), os_ver.c_str(),
                dep_it->second.yaml_path.c_str());
        throw runtime_error("woah...");
      }
    }
    return sats;
  }
  
  int cmd_update()
  {
    fprintf(stderr, 
            "rosdep: the 'update' command is deprecated. It will only\n"
            "update the 'global' rosdep.yaml file in the rosdep package,\n"
            "which is only kept around now for backwards-compatibility.\n"
            "The preferred method is now to place rosdep rules in\n"
            "rosdep.yaml files in the root of each package (i.e., next to\n"
            "the stack.xml files). Note that each package 'inherits' all\n"
            "of the rosdep rules in the packages it depends on, so the\n"
            "required\n");
    rospack::Package *rosdep_pkg = rs.rp.get_pkg("rosdep");
    if (!rosdep_pkg)
    {
      fprintf(stderr, "rosdep: couldn't find my own package path.\n");
      return 1;
    }
    string cmd = rosdep_pkg->path + string("/scripts/depdb-update " ) +
                 rosdep_pkg->path + string("/rosdep.yaml");
    fprintf(stderr, "executing command:\n%s\n", cmd.c_str());
    if (system(cmd.c_str()))
    {
      fprintf(stderr, "error updating the global rosdep.yaml file\n");
      return 1;
    }
    return 0;
  }

  int cmd_generate_bash(vector<string> pkg_names)
  {
    ensure_detect_os();
    safe_load(pkg_names);
    set<string> rosdeps;

    for (set<Package *>::iterator p = dep_pkgs.begin();
         p != dep_pkgs.end(); ++p)
    {
      for (TiXmlElement *d = (*p)->manifest_root()->FirstChildElement("rosdep");
           d; d = d->NextSiblingElement("rosdep"))
      {
        if (!d->Attribute("name"))
          throw runtime_error(string("rosdep tag in ") + (*p)->path +
                              string("/manifest.xml") +
                              string(" has no 'name' attribute"));
        const char *dep_name = d->Attribute("name");
        rosdeps.insert(string(dep_name));
      }
    }
    if (debug)
    {
      printf("rosdeps:\n");
      for (set<string>::iterator i = rosdeps.begin(); i != rosdeps.end(); ++i)
        printf("  %s\n", i->c_str());
      printf("\n\n");
    }
    // collect satisfaction rules for these rosdeps
    list <Sat *> sats;
    try
    {
      sats = rosdeps_to_sats(rosdeps);
    }
    catch (...)
    {
      return 1;
    }
    // see what we've got
#if 0
    printf("need to install these packages:\n");
    for (list<Sat *>::iterator sat_it = sats.begin();
         sat_it != sats.end(); ++sat_it)
    {
      for (vector<string>::iterator pkg_it = (*sat_it)->pkgs.begin();
           pkg_it != (*sat_it)->pkgs.end(); ++pkg_it)
        printf("  %s\n", pkg_it->c_str());
    }
    printf("and execute this bash:\n");
    for (list<Sat *>::iterator sat_it = sats.begin();
         sat_it != sats.end(); ++sat_it)
      if (!(*sat_it)->bash.empty())
        printf("----\n%s----\n", (*sat_it)->bash.c_str());
#endif
    set<string> native_pkgs_to_install;
    // check if they're already there
    for (list<Sat *>::iterator sat_it = sats.begin();
         sat_it != sats.end(); ++sat_it)
      for (vector<string>::iterator pkg_it = (*sat_it)->pkgs.begin();
           pkg_it != (*sat_it)->pkgs.end(); ++pkg_it)
      {
        string cmd = package_check_condition(*pkg_it);
        int ret = system((cmd + string(" 2>/dev/null >/dev/null ")).c_str());
        int rc = WEXITSTATUS(ret);
        if (rc)
          native_pkgs_to_install.insert(*pkg_it);
      }

    string bash = "#!/usr/bin/bash\n\nset -o errexit\nset -o verbose\n\n";
    string pkg_install_cmd;
    bool full_auto = false;
    if (getenv("ROSDEP_YES"))
      full_auto = (0 == strcmp(getenv("ROSDEP_YES"),"1"));
    if (os_name == "ubuntu" || os_name == "debian")
    {
      pkg_install_cmd = "sudo apt-get install ";
      if (full_auto)
        pkg_install_cmd += "-y ";
    }
    else if (os_name == "fedora" || os_name == "centos")
      pkg_install_cmd = "sudo yum install ";
    else if (os_name == "arch")
      pkg_install_cmd = "sudo pacman -Sy --needed ";
    else if (os_name == "macports")
      pkg_install_cmd = "sudo port install ";
    else
      throw runtime_error(string("unknown os_name: ") + os_name);

    if (native_pkgs_to_install.size())
    {
      for (set<string>::iterator native_pkg_it = native_pkgs_to_install.begin();
           native_pkg_it != native_pkgs_to_install.end(); ++native_pkg_it)
          pkg_install_cmd += *native_pkg_it + string(" ");
      pkg_install_cmd += string("\n");
      bash += pkg_install_cmd;
    }
    
    for (list<Sat *>::iterator sat_it = sats.begin();
         sat_it != sats.end(); ++sat_it)
      if (!(*sat_it)->bash.empty())
        bash += string("\n") + (*sat_it)->bash + string("\n");

    printf("%s\n", bash.c_str());
    return 0;
  }

private:
  rosstack::ROSStack rs;
  string os_name, os_ver;
  set<rospack::Package *> dep_pkgs;
  vector<string> yaml_files_found, yaml_files_not_found;

  void safe_load(vector<string> pkg_names)
  {
    try
    {
      load(pkg_names);
    } catch (runtime_error &e)
    {
      printf("rosdep: %s\n", e.what());
      exit(1);
    }
  }
  
  void load_one_file(string yaml_path, bool from_old_db = false)
  {
    std::ifstream fin(yaml_path.c_str());
    if (!fin)
    {
      if (debug)
        printf("couldn't open %s\n", yaml_path.c_str());
      yaml_files_not_found.push_back(yaml_path);
      return; // this file doesn't exist or isn't readable by us.
    }
    yaml_files_found.push_back(yaml_path);
    try
    {
      YAML::Parser parser(fin);
      while (parser)
      {
        YAML::Node doc;
        parser.GetNextDocument(doc);
        if (doc.GetType() != YAML::CT_MAP)
          throw runtime_error(yaml_path + "yaml top level isn't a map");
        for (YAML::Iterator i = doc.begin(); i != doc.end(); ++i)
        {
          string dep_name;
          i.first() >> dep_name;
          DepMap::iterator db_dep_it = deps.find(dep_name);
          if (db_dep_it != deps.end()) // todo: pin down where it was defined
          {
            if (from_old_db)
              continue; // ignore duplicates that conflict with the old DB
            else // but crash on duplicates in the Federation. shapeshifters.
              throw runtime_error(yaml_path + 
                                  string(" defined dependency key ") +
                                  dep_name + 
                                  string(" which was already defined in ") +
                                  db_dep_it->second.yaml_path);
          }
          Dep dep;
          if (from_old_db)
            dep.from_old_db = true;
          dep.yaml_path = yaml_path;
          for (YAML::Iterator j = i.second().begin(); j!=i.second().end(); ++j)
          {
            if (j.second().GetType() == YAML::CT_SCALAR)
            {
              Sat sat;
              j.first() >> sat.os_name;
              string native_pkgs;
              j.second() >> native_pkgs;
              if (native_pkgs.find_first_of('\n') == string::npos)
                rospack::string_split(native_pkgs, sat.pkgs, " ");
              else
                sat.bash = native_pkgs;
              sat.rosdep_name = dep_name;
              dep.sats.push_back(sat);
            }
            else if (j.second().GetType() == YAML::CT_MAP)
            {
              for (YAML::Iterator k = j.second().begin();
                  k != j.second().end(); ++k)
              {
                Sat sat;
                j.first() >> sat.os_name;
                k.first() >> sat.os_ver;
                string native_pkgs;
                k.second() >> native_pkgs;
                if (native_pkgs.find_first_of('\n') == string::npos)
                  rospack::string_split(native_pkgs, sat.pkgs, " ");
                else
                  sat.bash = native_pkgs;
                sat.rosdep_name = dep_name;
                dep.sats.push_back(sat);
              }
            }
            else if (j.second().GetType() != YAML::CT_NONE)
              throw runtime_error(yaml_path+": expected a scalar or map");
          }
          deps[dep_name] = dep;
        }
        if (debug)
          printf("parsed %s\n", yaml_path.c_str());
      }
    }
    catch(YAML::ParserException &e)
    {
      printf("YAML parse error in %s: %s\n",
             yaml_path.c_str(), e.what());
      exit(1);
      return;
    }
  }

  void load(vector<string> pkg_names)
  {
    for (vector<string>::iterator i = pkg_names.begin();
         i != pkg_names.end(); ++i)
    {
      rospack::Package *rp_pkg = rs.rp.get_pkg(*i);
      if (!rp_pkg)
        throw runtime_error(string("rospack couldn't find package ") + *i);
      else
        dep_pkgs.insert(rp_pkg);
      const rospack::VecPkg cdeps(rp_pkg->deps(rospack::Package::POSTORDER));
      for (vector<rospack::Package *>::const_iterator dep = cdeps.begin();
           dep != cdeps.end(); ++dep)
        dep_pkgs.insert(*dep);
    }
    if (debug)
      printf("package owners:\n");
    set<string> owner_paths;
    for (set<rospack::Package *>::iterator i = dep_pkgs.begin();
         i != dep_pkgs.end(); ++i)
    {
      string owner_path = rs.lookup_owner((*i)->name, false);
      if (!owner_path.length())
        throw std::runtime_error(string("couldn't determine owner of ") +
                                 (*i)->name +
                                 string(". Is it in ROS_STACK_PATH?"));
      owner_paths.insert(owner_path);
      if (debug)
      {
        printf("%s:\n", (*i)->name.c_str());
        string owner_name = rs.lookup_owner((*i)->name, true);
        printf("  %s: %s\n", owner_name.c_str(), owner_path.c_str());
      }
    }
    if (debug)
    {
      printf("\n\nset of owners:\n");
      for (set<string>::iterator i = owner_paths.begin();
           i != owner_paths.end(); ++i)
        printf("  %s\n", i->c_str());
    }
    for (set<string>::iterator i = owner_paths.begin();
         i != owner_paths.end(); ++i)
    {
      string yaml_path = (*i) + string("/rosdep.yaml");
      load_one_file(yaml_path);
    }
    rospack::Package *rosdep_pkg = rs.rp.get_pkg("rosdep");
    if (!rosdep_pkg)
      throw runtime_error(string("rospack couldn't find rosdep"));
    load_one_file(rosdep_pkg->path + string("/rosdep.yaml"), true);
  }

  void print_depdb()
  {
    for (DepMap::iterator i = deps.begin(); i != deps.end(); ++i)
    {
      for (vector<Sat>::iterator sat = i->second.sats.begin();
          sat != i->second.sats.end(); ++sat)
      {
        printf("%s ( %s %s ) -> [ ",
            i->first.c_str(), sat->os_name.c_str(), sat->os_ver.c_str());
        if (sat->bash.length() == 0)
        {
          for (vector<string>::iterator j = sat->pkgs.begin();
              j != sat->pkgs.end(); ++j)
            printf("%s ", j->c_str());
        }
        else
          printf("(bash code) ");
        printf("]\n");
      }
    }
  }

  bool detect_os()
  {
    // allow manual override of OS detection
    char *override_os_name = getenv("ROSDEP_OS_NAME");
    if (override_os_name)
    {
      os_name = string(override_os_name);
      char *override_os_version = getenv("ROSDEP_OS_VERSION");
      if (override_os_version)
        os_ver = string(override_os_version);
      return true;
    }

    // unfortunately, the order of these checks seems to matter...
    if (rospack::file_exists("/etc/arch-release"))
    {
      if (debug)
        printf("/etc/arch-release file found\n");
      os_name = "arch";
      return true;
    }
    else if (rospack::file_exists("/etc/redhat-release"))
    {
      if (debug)
        printf("/etc/redhat_release file found\n");
      FILE *f = fopen("/etc/redhat-release", "r");
      char issue[100];
      if (!fgets(issue, sizeof(issue), f))
        throw runtime_error("couldn't read from /etc/redhat-release");
      vector<string> os_tokens;
      rospack::string_split(issue, os_tokens, " ");
      if (debug)
      {
        printf("found %lu os tokens:\n", (unsigned long)os_tokens.size());
        for (size_t i = 0; i < os_tokens.size(); i++)
          printf("[%s]\n", os_tokens[i].c_str());
      }
      if (os_tokens.size() >= 3)
      {
        if (os_tokens[0] == string("Fedora") &&
            os_tokens[1] == string("release"))
        {
          os_name = "fedora";
          os_ver = os_tokens[2];
          return true;
        }
      }
      printf("Unknown /etc/redhat-release file found. Evidently your OS is not "
             "supported by this version of rosdep.");
    }
    else if (rospack::file_exists("/etc/issue"))
    {
      if (debug)
        printf("/etc/issue file found\n");
      FILE *f = fopen("/etc/issue", "r");
      char issue[100]; //os_name[50], os_ver[50];
      vector<string> os_tokens;
      if (!fgets(issue, sizeof(issue), f))
        throw std::runtime_error("couldn't read from /etc/issue");
      rospack::string_split(issue, os_tokens, " ");
      if (debug)
      {
        printf("found %lu os tokens:\n", (unsigned long)os_tokens.size());
        for (size_t i = 0; i < os_tokens.size(); i++)
          printf("[%s]\n", os_tokens[i].c_str());
      }
      if (os_tokens.size() < 2)
        throw runtime_error("couldn't parse two tokens from /etc/issue\n");
      else
      {
        if (os_tokens[0] == string("Ubuntu"))
        {
          os_name = "ubuntu";
          vector<string> version_tokens;
          rospack::string_split(os_tokens[1], version_tokens, ".");
          assert(version_tokens.size() >= 2);
          os_ver = version_tokens[0] + "." + version_tokens[1];
          return true;
        }
        else if (os_tokens[0] == string("Debian"))
        {
          os_name = "debian";
          os_ver = os_tokens[2];
          return true;
        }
        else if (os_tokens[0] == string("Linux") &&
                 os_tokens[1] == string("Mint"))
        {
          os_name = "mint";
          os_ver = os_tokens[2];
          return true;
        }
        else
          throw runtime_error("/etc/issue was neither ubuntu nor debian.\n");
      }
      fclose(f);
    }
    else if (rospack::file_exists("/usr/bin/sw_vers"))
    {
      os_name = "macports"; // assume this is the only decent way to get stuff
      FILE *sw_pipe = popen("sw_vers | grep 'ProductVersion' | grep -o '[0-9][0-9]*\\.[0-9]*'", "r");
      if (!sw_pipe)
        throw runtime_error("couldn't get output of sw_vers");
      char sw_pipe_response[100];
      if (fgets(sw_pipe_response, sizeof(sw_pipe_response), sw_pipe))
        os_ver = string(sw_pipe_response, strlen(sw_pipe_response)-1);
      else
        throw runtime_error("no response from sw_vers");
      return true;
    }
    os_name = "unknown";
    return false;
  }

  void ensure_detect_os()
  {
    if (!detect_os())
    {
      printf("rosdep couldn't detect your OS. if you'd like to help \n"
             "improve rosdep, please submit a patch to rosdep.cpp.\n\n");
      exit(1);
    }
  }

  string package_check_condition(string pkg_name)
  {
    if (os_name == "ubuntu" || os_name == "debian")
      return string("dpkg -s ") + pkg_name;
    else if (os_name == "fedora" || os_name == "centos")
      return string("yum list ") + pkg_name;
    else if (os_name == "arch")
      return string("pacman -Q ") + pkg_name;
    else if (os_name == "macports")
      return string("false"); // fix this...
    else
      throw runtime_error(string("unhandled os_name: ") + os_name);
    return string("never gets here. this is just for the compiler warning");
  }
 
};

int main(int argc, char **argv)
{
  if (argc <= 1)
    return usage();

  ROSDep rd;
  if (getenv("DEBUG") && strcmp(getenv("DEBUG"), "0"))
    rd.debug = true;

  string command(argv[1]);
  vector<string> pkg_names;

  for (int i = 2; i < argc; i++)
    pkg_names.push_back(argv[i]);

  if (command == string("depdb"))
    return rd.cmd_depdb(pkg_names);
  else if (command == string("install"))
    return rd.cmd_install(pkg_names);
  else if (command == string("generate_bash") ||
           command == string("satisfy"))
    return rd.cmd_generate_bash(pkg_names);
  else if (command == string("what_needs"))
    return rd.cmd_what_needs(pkg_names);
  else if (command == string("detect_os"))
    return rd.cmd_detect_os();
  else if (command == string("check"))
    return rd.cmd_check(pkg_names);
  else if (command == string("update"))
    return rd.cmd_update();
  else
    return usage();
}

///// GRAVEYARD. SPOOKY. //////

/*
   printf("I want to install these rosdep packages:\n---------------\n");
   for (vector<string>::iterator s = deps.begin(); s != deps.end(); ++s)
   printf("%s\n", s->c_str());
   printf("---------------\n");
   */
/*
   printf("using these native packages:\n---------------\n");
   for (vector<string>::iterator s = native_pkgs.begin();
   s != native_pkgs.end(); ++s)
   printf("%s\n", s->c_str());
   printf("---------------\n");
   */

