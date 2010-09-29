// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Reusable parser routines

#include "topic_tools/parse.h"

#include <ros/console.h>

namespace topic_tools
{

// Strip any leading namespace qualification from a topic (or other kind
// of) ROS name
bool 
getBaseName(const std::string& full_name, std::string& base_name)
{
  std::string tmp = full_name;
  int i = tmp.rfind('/');
  // Strip off trailing slahes (are those legal anyway?)
  while((tmp.size() > 0) && (i >= (int)(tmp.size() - 1)))
  {
    tmp = tmp.substr(0,tmp.size()-1);
    i = tmp.rfind('/');
  }

  if(tmp.size() == 0)
  {
    ROS_ERROR("Base name extracted from \"%s\" is an empty string", 
              full_name.c_str());
    return false;
  }

  if(i < 0)
    base_name = tmp;
  else
    base_name = tmp.substr(i+1, tmp.size()-i-1);

  return true;
}

}
