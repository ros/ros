/*
 * Copyright (C) 2008, Willow Garage, Inc.
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

#include "rp.h"
#include <boost/program_options.hpp>
#include <iostream>
#include <stdlib.h>

namespace po = boost::program_options;

const char* usage()
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

int
parse_args(int argc, char** argv, po::variables_map& vm)
{
  po::options_description desc("Allowed options");
  desc.add_options()
          ("command", po::value<std::string>(), "command")
          ("package", po::value<std::string>(), "package")
          ("target", po::value<std::string>(), "target")
          ("deps-only", "deps-only")
          ("lang", po::value<std::string>(), "lang")
          ("attrib", po::value<std::string>(), "attrib")
          ("top", po::value<std::string>(), "top")
          ("length", po::value<std::string>(), "length")
          ("zombie-only", "zombie-only");

  po::positional_options_description pd;
  pd.add("command", 1).add("package", 1);
  try
  {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pd).run(), vm);
  }
  catch(boost::program_options::error e)
  {
    rospack::log_error("rospack", std::string("failed to parse command-line options: ") + e.what());
    // TODO: print USAGE
    return 1;
  }
  po::notify(vm);

  return 0;
}

int
main(int argc, char** argv)
{
  po::variables_map vm;
  int ret;

  ret = parse_args(argc, argv, vm);
  if(ret)
    return ret;

  rospack::Rospack rp;
  std::vector<std::string> search_path;
  rospack::get_search_path_from_env(search_path);
  rp.crawl(search_path, false);

  std::string command;
  std::string package;
  if(vm.count("command"))
    command = vm["command"].as<std::string>();
  if(vm.count("package"))
    package = vm["package"].as<std::string>();
  else
  {
    // TODO: try to determine package from directory context
  }

  if(!command.size())
  {
    rospack::log_error("rospack", "no command given");
    return 1;
  }

  if(command == "find")
  {
    if(!package.size())
    {
      rospack::log_error("rospack", "no package given for find command");
      return 1;
    }
    std::string path;
    if(!rp.find(package, path))
      return 1;
    else
    {
      printf("%s\n", path.c_str());
      return 0;
    }
  }
  else if(command == "list")
  {
    std::vector<std::pair<std::string, std::string> > list;
    rp.list(list);
    for(std::vector<std::pair<std::string, std::string> >::const_iterator it = list.begin();
        it != list.end();
        ++it)
    {
      printf("%s %s\n", it->first.c_str(), it->second.c_str());
    }
  }
  else if(command == "list-names")
  {
    std::vector<std::pair<std::string, std::string> > list;
    rp.list(list);
    for(std::vector<std::pair<std::string, std::string> >::const_iterator it = list.begin();
        it != list.end();
        ++it)
    {
      printf("%s\n", it->first.c_str());
    }
  }
  else if(command == "list-duplicates")
  {
    rospack::log_error("rospack", 
                       std::string("command ") + command + " not implemented");
    return 1;
  }
  else if(command == "langs")
  {
    rospack::log_error("rospack", 
                       std::string("command ") + command + " not implemented");
    return 1;
  }
  else if(command == "depends" || command == "deps" || 
          command == "depends1" || command == "deps1")
  {
    std::vector<std::string> deps;
    if(!rp.deps(package, (command == "depends1" || command == "deps1"), deps))
      return 1;
    else
    {
      for(std::vector<std::string>::const_iterator it = deps.begin();
          it != deps.end();
          ++it)
        printf("%s\n", it->c_str());
      return 0;
    }
  }
  else if(command == "help")
  {
    printf("%s", usage());
    return 0;
  }
  else
  {
    rospack::log_error("rospack", 
                       std::string("command ") + command + " not implemented");
    return 1;
  }

  return 0;
}
