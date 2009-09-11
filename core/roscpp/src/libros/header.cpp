/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "ros/header.h"
#include "ros/transport/transport.h"
#include <ros/console.h>
#include <ros/assert.h>

#include <sstream>

#include <sys/socket.h>
#include <cerrno>

using namespace std;

namespace ros
{

Header::Header()
: read_map_(new M_string())
{

}

Header::~Header()
{

}

bool Header::parse(boost::shared_array<uint8_t> buffer, uint32_t size, std::string& error_msg)
{
  // skim through the header and pull out the key, value pairs
  std::string tcp_header((char*)buffer.get(), size);

  size_t prev_crpos = 0;
  for (size_t crpos = tcp_header.find_first_of("\n", 0);
       crpos != string::npos;
       crpos = tcp_header.find_first_of("\n", crpos+1))
  {
    string line = tcp_header.substr(prev_crpos, crpos - prev_crpos);
    //printf(":%s:\n", line.c_str());
    size_t eqpos = line.find_first_of("=", 0);
    if (eqpos == string::npos)
    {
      error_msg = "Received an invalid TCPROS header.  Each line must have an equals sign.";
      ROS_ERROR("%s", error_msg.c_str());

      return false;
    }
    string key = line.substr(0, eqpos);
    string value = line.substr(eqpos+1);

    (*read_map_)[key] = value;
    prev_crpos = crpos+1;
  }

  return true;
}

bool Header::getValue(const std::string& key, std::string& value) const
{
  M_string::const_iterator it = read_map_->find(key);
  if (it == read_map_->end())
  {
    return false;
  }

  value = it->second;

  return true;
}

}
