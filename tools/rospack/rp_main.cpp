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
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <stdlib.h>

using namespace rospack;
namespace po = boost::program_options;

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
    rospack_error("rospack", std::string("failed to parse command-line options: ") + e.what());
    // TODO: print USAGE
    return 1;
  }
  po::notify(vm);

  return 0;
}

void
get_search_path_from_env(std::vector<std::string>& sp)
{
  char* rr = getenv("ROS_ROOT");
  char* rpp = getenv("ROS_PACKAGE_PATH");

  if(rr)
    sp.push_back(rr);
  if(rpp)
  {
    std::vector<std::string> rpp_strings;
    boost::split(rpp_strings, rpp, 
                 boost::is_any_of(":"),
                 boost::token_compress_on);
    for(std::vector<std::string>::const_iterator it = rpp_strings.begin();
        it != rpp_strings.end();
        ++it)
    {
      sp.push_back(*it);
    }
  }
}

int
main(int argc, char** argv)
{
  po::variables_map vm;
  int ret;

  ret = parse_args(argc, argv, vm);
  if(ret)
    return ret;

  Rospack rp;
  std::vector<std::string> search_path;
  get_search_path_from_env(search_path);
  rp.crawl(search_path, false);

  std::string command;
  std::string package;
  if(vm.count("command"))
    command = vm["command"].as<std::string>();
  if(vm.count("package"))
    package = vm["package"].as<std::string>();

  if(!command.size())
  {
    rospack_error("rospack", "no command given");
    return 1;
  }

  if(command == "find")
  {
    if(!package.size())
    {
      rospack_error("rospack", "no package given for find command");
      return 1;
    }
    std::string path = rp.find(package);
    printf("%s\n", path.c_str());
  }
  else
  {
    rospack_error("rospack", 
                  std::string("command ") + command + " not implemented");
    return 1;
  }

  return 0;
}
