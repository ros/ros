// copyright morgan quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#ifndef DEPDB_H
#define DEPDB_H

#include <string>
#include <vector>
#include <map>

namespace rosdep
{

class Sat // a way to satisfy a particular dependency on a particular OS
{
public:
  std::string os_name, os_ver;
  std::vector<std::string> pkgs;
  std::string bash; // non-zero length if this sat is done via bash rather than pkg man.
};

class DepDB
{
public:
  typedef std::map< std::string, std::vector<Sat> > DepMap;
  DepMap deps;
  std::string rosdep_pkg_path, depdb_path;

  DepDB();
  ~DepDB() { }

  void print();
  bool satisfy(const std::vector<std::string> &depdb_pkgs,
                     std::vector<std::string> &native_pkgs,
                     std::string &bash,
               const std::string &os_name, const std::string &os_ver);
};

}

#endif

