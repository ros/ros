// copyright morgan quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#include <string>
#include <list>
#include <vector>
#include <cstdio>
#include "rospack/rospack.h"
#include "tinyxml-2.5.3/tinyxml.h"
#include "os.h"
#include "depdb.h"

using std::string;
using std::vector;
using std::list;

using namespace rospack;
using namespace rosdep;

void usage()
{
  printf("\nusage: rosdep COMMAND [PACKAGE1 PACKAGE2 ...]\n"
         "  where COMMAND is one of { satisfy, depdb, update }\n\n"
         "'rosdep satisfy PACKAGE'   will try to output a bash script which \n"
         "will satisfy the dependencies of PACKAGE on your operating system.\n"
         "If you want to run the output of rosdep all in one shot, you can \n"
         "use this bit of bash magic:\n\n"
         "bash <(rosdep satisfy PACKAGE)\n\n"
         "'rosdep rosdb' will print out the internal database of \n"
         "rosdep, which it loads from `rospack find rosdep`/rosdep.yaml\n"
         "'rosdep update' will download a new YAML database.\n"
         "Additions and corrections to this file are greatly appreciated.\n\n");
}

int main(int argc, char **argv)
{
  OS os;
  DepDB db;
  if (os.isUnknown())
  {
    printf("woah! couldn't detect your operating system.\n");
    return 0;
  }
  //printf("running on %s %s\n", os.name.c_str(), os.version.c_str());
  if (argc <= 1)
  {
    usage();
    return 0;
  }
  ROSPack rp;
  if (!strcmp(argv[1], "satisfy"))
  {
    list<Package *> pkgs_no_deps;
    for (int i = 2; i < argc; i++)
    {
      Package *p = rp.get_pkg(argv[i]);
      if (!p)
      {
        printf("couldn't find package [%s]\n", argv[i]);
        return 1;
      }
      pkgs_no_deps.push_back(p);
    }
    list<Package *> pkgs;
    for (list<Package *>::iterator p = pkgs_no_deps.begin(); 
         p != pkgs_no_deps.end(); ++p)
    {
      vector<Package *> p2 = (*p)->deps(Package::POSTORDER);
      pkgs.insert(pkgs.end(), p2.begin(), p2.end());
      pkgs.push_back(*p);
    }
    pkgs.sort();
    pkgs.unique();
    vector<string> deps, native_pkgs;
    for (list<Package *>::iterator p = pkgs.begin(); p != pkgs.end(); ++p)
    {
      for (TiXmlElement *dep = (*p)->manifest_root()->FirstChildElement("rosdep");
           dep;
           dep = dep->NextSiblingElement("rosdep"))
      {
        const char *dep_name;
        if ((dep_name = dep->Attribute("name")))
        {
          // don't insert duplicates; ensure this isn't already in there 
          bool found = false;
          for (vector<string>::iterator i = deps.begin();
               i != deps.end() && !found; ++i)
            if (*i == string(dep_name))
              found = true;
          if (!found)
            deps.push_back(string(dep_name));
        }
      }
    }
    string bash;
    if (!db.satisfy(deps, native_pkgs, bash, os.name, os.version))
      return 1;
    printf("\n%s\n", os.generate_bash(native_pkgs, bash).c_str());
  }
  else if (!strcmp(argv[1], "depdb"))
  {
    db.print();
  }
  else if (!strcmp(argv[1], "update"))
  {
    string cmd = string("cd ") + db.rosdep_pkg_path +
                 string(" && scripts/depdb-update ") + db.depdb_path;
    //printf("%s\n", cmd.c_str());
    if (system(cmd.c_str()))
      printf("whoops, error updating the dependency database.\n");
    else
      printf("dependency database updated successfully.\n");
  }
  else
    usage();
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
  return 0;
}

