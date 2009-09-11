/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/init.h"
#include "ros/node.h"

namespace ros
{

void init(int& _argc, char** _argv)
{
  Node::s_args_.clear();

  int full_argc = _argc;
  // now, move the remapping argv's to the end, and decrement argc as needed
  for (int i = 0; i < _argc; )
  {
    std::string arg = _argv[i];
    size_t pos = arg.find(":=");
    if (pos != std::string::npos)
    {
      std::string local_name = arg.substr(0, pos);
      std::string external_name = arg.substr(pos + 2);

      Node::s_remappings_.push_back(std::make_pair(local_name, external_name));
      Node::s_args_.push_back(_argv[i]);

      // shuffle everybody down and stuff this guy at the end of argv
      char *tmp = _argv[i];
      for (int j = i; j < full_argc - 1; j++)
        _argv[j] = _argv[j+1];
      _argv[_argc-1] = tmp;
      _argc--;
    }
    else
    {
      i++; // move on, since we didn't shuffle anybody here to replace it
    }
  }

  Node::s_initialized_ = true;
}

void init(const VP_string& remappings)
{
  Node::s_remappings_ = remappings;

  Node::s_initialized_ = true;
}

void init(int& argc, char** argv, const std::string& name, uint32_t options)
{
  Node::s_name_ = name;
  Node::s_flags_ = options;

  init(argc, argv);
}

void init(const VP_string& remappings, const std::string& name, uint32_t options)
{
  Node::s_name_ = name;
  Node::s_flags_ = options;

  init(remappings);
}

}
