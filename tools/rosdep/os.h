// copyright morgan quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#ifndef ROSINSTALL_OS_H
#define ROSINSTALL_OS_H

#include <string>
#include <vector>
#include <list>

class TiXmlElement;

namespace rosdep
{

class OS
{
public:
  OS();
  std::string name, version;
  //OS(const OS &copy) : name(copy.name) { }
  //const char *prettyName();
  //const char *manifestName();
  bool isUnknown() { return name == "unknown"; }
  //bool addInstallTag(TiXmlElement *);
  //bool executeTransactions();
  std::string generate_bash(std::vector<std::string> pkgs, const std::string &bash);

private:
  bool detect(std::string &name, std::string &ver);
  
  /*
  class Transaction
  {
  public:
    enum Manager { UNKNOWN, APT, PACMAN } manager;
    std::list<std::string> pkgs;
    Transaction(TiXmlElement *);
    Transaction(std::list<Transaction>);
    void print();
    bool execute();
  private:
    Transaction();
    Manager managerFromText(const char *text);
    const char *textFromManager(Manager);
  };
  std::list<Transaction> transactions;
  */
};

}

#endif

