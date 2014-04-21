/*
 * Copyright (C) 2009, Willow Garage, Inc.
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

#include "ros/package.h"
#include "rospack/rospack.h"

#include <cstdio>
#include <iostream>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/thread/mutex.hpp>

namespace ros
{
namespace package
{

// Mutex used to lock calls into librospack, which is not thread-safe.
static boost::mutex librospack_mutex;

std::string command(const std::string& _cmd)
{
  boost::mutex::scoped_lock lock(librospack_mutex);

  rospack::ROSPack rp;
  int ret;
  try
  {
    ret = rp.run(_cmd);
    if(ret == 0)
      return rp.getOutput();
    else {
      if ( !rp.is_quiet() )
	std::cerr << "ROSPack::run returned non-zero." << std::endl;
    }
  }
  catch(std::runtime_error &e)
  {
    if ( !rp.is_quiet() )
      std::cerr << "[rospack] " << e.what() << std::endl;
  }
  return std::string("");
}

void command(const std::string& cmd, V_string& output)
{
  std::string out_string = command(cmd);
  V_string full_list;
  boost::split(full_list, out_string, boost::is_any_of("\r\n"));

  // strip empties
  V_string::iterator it = full_list.begin();
  V_string::iterator end = full_list.end();
  for (; it != end; ++it)
  {
    if (!it->empty())
    {
      output.push_back(*it);
    }
  }
}

std::string getPath(const std::string& package_name)
{
  std::string path = command("find " + package_name);

  // scrape any newlines out of it
  for (size_t newline = path.find('\n'); newline != std::string::npos;
              newline = path.find('\n'))
  {
    path.erase(newline, 1);
  }

  return path;
}

bool getAll(V_string& packages)
{
  command("list-names", packages);

  return true;
}

static void getPlugins(const std::string& package, const std::string& attribute, V_string& packages, V_string& plugins, bool force_recrawl)
{
  if (force_recrawl)
  {
    command("profile");
  }

  V_string lines;
  command("plugins --attrib=" + attribute + " " + package, lines);

  V_string::iterator it = lines.begin();
  V_string::iterator end = lines.end();
  for (; it != end; ++it)
  {
    V_string tokens;
    boost::split(tokens, *it, boost::is_any_of(" "));

    if (tokens.size() >= 2)
    {
      std::string package = tokens[0];
      std::string rest = boost::join(V_string(tokens.begin() + 1, tokens.end()), " ");
      packages.push_back(package);
      plugins.push_back(rest);
    }
  }
}

void getPlugins(const std::string& package, const std::string& attribute, V_string& plugins, bool force_recrawl)
{
  V_string packages;
  getPlugins(package, attribute, packages, plugins, force_recrawl);
}

void getPlugins(const std::string& package, const std::string& attribute, M_string& plugins, bool force_recrawl)
{
  V_string packages, plugins_v;
  getPlugins(package, attribute, packages, plugins_v, force_recrawl);
  for (std::size_t i = 0 ; i < packages.size() ; ++i)
    plugins[packages[i]] = plugins_v[i];
}

} // namespace package
} // namespace ros
