// copyright morgan quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#include <fstream>
#include <limits.h>
#include "depdb.h"
#include "yaml.h"
#include <stdexcept>
#include <iostream>
#include "rospack/rospack.h"

using namespace rosdep;
using std::map;
using std::string;
using std::vector;
using std::runtime_error;

DepDB::DepDB()
{
  // query rospack to find our package path
  char *override_db_path = getenv("ROSDEP_DB_PATH");
  if (override_db_path)
    depdb_path = string(override_db_path);
  else
  {
    // use `rospack find rosdep`/rosdep.yaml
    FILE *rospack_pipe = popen("rospack find rosdep", "r");
    if (!rospack_pipe)
      throw runtime_error("couldn't run rospack");
    char rospack_response[PATH_MAX];
    if (fgets(rospack_response, PATH_MAX, rospack_pipe))
    {
      // scrape out newlines
      string ret(rospack_response);
      for (size_t newline = ret.find('\n'); newline != string::npos;
                  newline = ret.find('\n'))
        ret.erase(newline, 1);
      rosdep_pkg_path = ret;
    }
    else
      throw runtime_error("no response from rospack");
    depdb_path = rosdep_pkg_path + string("/rosdep.yaml");
  }
}

void
DepDB::load()
{
  std::ifstream fin(depdb_path.c_str());
  // make sure it's openable
  if (!fin)
    throw runtime_error(string("couldn't open rosdep YAML file ") + depdb_path);
  YAML::Parser parser(fin);
  while (parser)
  {
    YAML::Node doc;
    parser.GetNextDocument(doc);
    if (doc.GetType() != YAML::CT_MAP)
      throw runtime_error("yaml top level isn't a map");
    for (YAML::Iterator i = doc.begin(); i != doc.end(); ++i)
    {
      std::string dep;
      i.first() >> dep;
      //printf(" reading dep: %s\n", dep.c_str());
      for (YAML::Iterator j = i.second().begin(); j != i.second().end(); ++j)
      {
        if (j.second().GetType() == YAML::CT_SCALAR)
        {
          Sat sat;
          j.first() >> sat.os_name;
          std::string native_pkgs;
          j.second() >> native_pkgs;
          if (native_pkgs.find_first_of('\n') == string::npos)
            rospack::string_split(native_pkgs, sat.pkgs, " ");
          else
            sat.bash = native_pkgs;
          deps[dep].push_back(sat);
        }
        else if (j.second().GetType() == YAML::CT_MAP)
        {
          for (YAML::Iterator k = j.second().begin();
               k != j.second().end(); ++k)
          {
            Sat sat;
            j.first() >> sat.os_name;
            k.first() >> sat.os_ver;
            std::string native_pkgs;
            k.second() >> native_pkgs;
            if (native_pkgs.find_first_of('\n') == string::npos)
              rospack::string_split(native_pkgs, sat.pkgs, " ");
            else
              sat.bash = native_pkgs;
            deps[dep].push_back(sat);
          }
        }
        else if (j.second().GetType() != YAML::CT_NONE)
          throw runtime_error("woah. should be either a scalar or a map here");
      }
    }
  }
}

void DepDB::print()
{
  for (DepMap::iterator i = deps.begin(); i != deps.end(); ++i)
  {
    for (vector<Sat>::iterator sat = i->second.begin();
         sat != i->second.end(); ++sat)
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

bool DepDB::satisfy(const vector<string> &depdb_pkgs,
                          vector<string> &native_pkgs,
                          string &bash, const string &os_name, const string &os_ver)
{
  //printf("satisfying with os = %s %s\n", os_name.c_str(), os_ver.c_str());
  for (vector<string>::const_iterator d = depdb_pkgs.begin();
       d != depdb_pkgs.end(); ++d)
  {
    //printf("trying to satisfy rosdep %s\n", d->c_str());
    bool found_rosdep = false, satisfied_rosdep = false;
    for (DepMap::iterator i = deps.begin();
         i != deps.end() && !found_rosdep; ++i)
    {
      //printf("found depdb entry %s\n", i->first.c_str());
      if (i->first == *d)
      {
        found_rosdep = true;
        //printf("found entry in depdb for %s\n", d->c_str());
        for (vector<Sat>::iterator sat = i->second.begin();
             sat != i->second.end(); ++sat)
        {
          //printf("found sat for %s on %s %s\n",
          //       d->c_str(),
          //       sat->os_name.c_str(),
          //       sat->os_ver.c_str());
          if (sat->os_name == os_name &&
              (sat->os_ver == os_ver ||
               sat->os_ver.empty()   ||
               os_ver.empty()))
          {
            satisfied_rosdep = true;
            //printf("hooray, found a match\n");
            if (!sat->bash.length())
              native_pkgs.insert(native_pkgs.end(),
                                 sat->pkgs.begin(), sat->pkgs.end());
            else
              bash += string("# ") + *d + string("\n") + 
                      sat->bash + string("\n");
          }
        }
      }
    }
    if (!found_rosdep || !satisfied_rosdep)
    {
      fprintf(stderr,
              "\nrosdep: couldn't find a way to satisfy dependency [%s] for \n"
              "OS/distro [%s] version [%s]. \n\n"
              "It's possible that this has already been fixed. You can try \n"
              "to run 'rosdep update' to bring down the latest version of \n"
              "rosdep.yaml. This will bring your local rosdep.yaml file up \n"
              "to the latest version in SVN. Then, try rosdep again.\n\n"
              "If rosdep continues to fail, consider editing\n"
              "%s\n"
              "to add the necessary information; we will gladly accept\n"
              "patches.\n\n",
              d->c_str(), os_name.c_str(), os_ver.c_str(), depdb_path.c_str());
      return false;
    }
  }
  return true;
}

