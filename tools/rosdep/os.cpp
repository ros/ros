// copyright morgan quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#include <stdexcept>
#include <cassert>
#include <string>
#include <cstdio>
#include "os.h"
#include "rospack/rospack.h"

#define RI_VERBOSITY 0

using namespace rosdep;
using std::string;
using std::list;
using std::vector;
using std::runtime_error;

OS::OS() : name(string("unknown"))
{
  if (!detect(name, version))
    throw runtime_error("couldn't detect this OS\n");
}

bool OS::detect(string &name, string &ver)
{
  // allow manual override of OS detection
  char *override_os_name = getenv("ROSDEP_OS_NAME");
  if (override_os_name)
  {
    name = string(override_os_name);
    char *override_os_version = getenv("ROSDEP_OS_VERSION");
    if (override_os_version)
      ver = string(override_os_version);
    return true;
  }

  // unfortunately, the order of these checks matters...
  if (rospack::file_exists("/etc/arch-release"))
  {
#if RI_VERBOSITY > 0
    printf("/etc/arch-release file found\n");
#endif
    name = "arch";
    return true;
  }
  else if (rospack::file_exists("/etc/issue"))
  {
#if RI_VERBOSITY > 0
    printf("/etc/issue file found\n");
#endif
    FILE *f = fopen("/etc/issue", "r");
    char issue[100]; //os_name[50], os_ver[50];
    std::vector<string> os_tokens;
    fgets(issue, sizeof(issue), f);
    rospack::string_split(issue, os_tokens, " ");
#if RI_VERBOSITY > 0
    printf("found %d os tokens:\n", os_tokens.size());
    for (size_t i = 0; i < os_tokens.size(); i++)
      printf("[%s]\n", os_tokens[i].c_str());
#endif
    if (os_tokens.size() < 2)
    {
#if RI_VERBOSITY > 0
      printf("couldn't grab two tokens out of /etc/issue\n");
#endif
      throw runtime_error("couldn't parse two tokens from /etc/issue\n");
    }
    else
    {
      if (os_tokens[0] == string("Ubuntu"))
      {
        name = "ubuntu";
        std::vector<string> version_tokens;
        rospack::string_split(os_tokens[1], version_tokens, ".");
        assert(version_tokens.size() >= 2);
        version = version_tokens[0] + "." + version_tokens[1];
        return true;
      }
      else if (os_tokens[0] == string("Debian"))
      {
        name = "debian";
        version = os_tokens[2];
        return true;
      }
      else
        throw runtime_error("/etc/issue wasn't ubuntu. need to fix rosdep.\n");
    }
    fclose(f);
  }
  else if (rospack::file_exists("/usr/bin/sw_vers"))
  {
    name = "macports"; // assume this is the only decent way to get stuff
    FILE *sw_pipe = popen("sw_vers | grep 'ProductVersion' | grep -o '[0-9][0-9]*\\.[0-9]*'", "r");
    if (!sw_pipe)
      throw runtime_error("couldn't get output of sw_vers");
    char sw_pipe_response[100];
    if (fgets(sw_pipe_response, sizeof(sw_pipe_response), sw_pipe))
      version = string(sw_pipe_response, strlen(sw_pipe_response)-1);
    else
      throw runtime_error("no response from sw_vers");
    return true;
  }
  name = "unknown";
  return false;
}

string OS::generate_bash(vector<string> pkgs, const string &verbatim_bash)
{
  string bash = "#!/bin/bash\nset -o errexit\nset -o verbose\n\n";
  vector<string> pkgs_to_install;
  if (name == "macports")
  {
    // screen-scrape the output of the 'ports installed' command so as to
    // not install a port multiple times. I was hoping there was a built-in
    // function of the 'port' command to do this, but i couldn't find it.
    FILE *port_pipe = popen("port installed | grep -o '^  [a-zA-Z0-9_-]*' ", "r");
    if (!port_pipe)
      throw runtime_error("couldn't get the installed ports");
    char installed[1000];
    while (!feof(port_pipe))
    {
      if (fgets(installed, sizeof(installed), port_pipe))
      {
        if (strlen(installed) > 0)
          installed[strlen(installed)-1] = 0; // blow away newline
        //printf("installed port: [%s]\n", installed+2);
        for (vector<string>::iterator i = pkgs.begin(); i != pkgs.end(); ++i)
        {
          if (*i == string(installed + 2))
          {
            pkgs.erase(i);
            break;
          }
        }
      }
      else if (!feof(port_pipe))
        throw runtime_error("error reading from port command");
    }
  }

  if (pkgs.size() > 0)
  {
    if (name == "arch")
      bash += "sudo pacman --needed -S";
    else if (name == "ubuntu" || name == "debian")
      bash += "sudo apt-get -y install";
    else if (name == "macports")
      bash += "sudo port install";
    for (vector<string>::iterator i = pkgs.begin(); i != pkgs.end(); ++i)
      bash += string(" ") + *i;
    bash += string("\n\n");
  }
  return bash + verbatim_bash;
}

