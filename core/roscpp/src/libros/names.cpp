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

#include "ros/names.h"
#include "ros/this_node.h"

#include <ros/console.h>
#include <ros/assert.h>

namespace ros
{

namespace names
{

M_string g_remappings;
M_string g_unresolved_remappings;

const M_string& getRemappings()
{
  return g_remappings;
}

const M_string& getUnresolvedRemappings()
{
  return g_unresolved_remappings;
}

std::string clean(const std::string& name)
{
  std::string clean = name;

  size_t pos = clean.find("//");
  while (pos != std::string::npos)
  {
    clean.erase(pos, 1);
    pos = clean.find("//", pos);
  }

  if (*clean.rbegin() == '/')
  {
    clean.erase(clean.size() - 1, 1);
  }

  return clean;
}

std::string append(const std::string& left, const std::string& right)
{
  return clean(left + "/" + right);
}

std::string remap(const std::string& name)
{
  std::string resolved = resolve(name, false);

  M_string::const_iterator it = g_remappings.find(resolved);
  if (it != g_remappings.end())
  {
    return it->second;
  }

  return name;
}

std::string resolve(const std::string& name, bool _remap)
{
  return resolve(this_node::getNamespace(), name, _remap);
}

std::string resolve(const std::string& ns, const std::string& name, bool _remap)
{
  if (name.empty())
  {
    if (ns.empty())
    {
      return "/";
    }

    if (ns[0] == '/')
    {
      return ns;
    }

    return append("/", ns);
  }

  std::string copy = name;

  if (copy[0] == '~')
  {
    copy = append(this_node::getName(), copy.substr(1));
  }

  if (copy[0] != '/')
  {
    copy = append("/", append(ns, copy));
  }

  copy = clean(copy);

  if (_remap)
  {
    copy = remap(copy);
  }

  return copy;
}

void init(const M_string& remappings)
{
  M_string::const_iterator it = remappings.begin();
  M_string::const_iterator end = remappings.end();
  for (; it != end; ++it)
  {
    const std::string& left = it->first;
    const std::string& right = it->second;
    if (!left.empty() && left[0] != '_')
    {
      std::string resolved_left = resolve(left, false);
      std::string resolved_right = resolve(right, false);
      g_remappings[resolved_left] = resolved_right;
      g_unresolved_remappings[left] = right;
    }
  }
}

} // namespace names

} // namespace ros
