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

namespace po = boost::program_options;

int
parse_args(int argc, char** argv)
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
  po::variables_map vm;
  try
  {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pd).run(), vm);
  }
  catch(boost::program_options::error e)
  {
    fprintf(stderr, "Error parsing command-line options: %s\n", e.what());
    // TODO: print USAGE
    return 1;
  }
  po::notify(vm);

  for(po::variables_map::const_iterator it = vm.begin();
      it != vm.end();
      ++it)
  {
    printf("%s:%s:\n", it->first.c_str(), it->second.as<std::string>().c_str());
  }

  return 0;
}

int
main(int argc, char** argv)
{
  int ret;

  ret = parse_args(argc, argv);
  if(ret)
    return ret;

  rospack::Rospack rp;
  std::vector<std::string> search_path;
  search_path.push_back("/Users/gerkey/code/ros/ros");
  search_path.push_back("/Users/gerkey/code/ros/ros/tools/rospack");
  search_path.push_back("/Users/gerkey/code/ros/ros_comm");
  rp.crawl(search_path, false);

  //rp.debug_dump();

  std::string rospack_path = rp.find("rospack");
  //printf("rospack:%s:\n", rospack_path.c_str());

  return 0;
}
