///////////////////////////////////////////////////////////////////////////////
// The mux package provides a generic multiplexer
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
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
/////////////////////////////////////////////////////////////////////////////


#include <cstdio>
#include "ros/console.h"
#include "ros/ros.h"
#include "topic_tools/MuxSelect.h"
#include "topic_tools/parse.h"
using namespace std;
using namespace ros;
using namespace topic_tools;

int main(int argc, char **argv)
{
  ROS_WARN("topic_tools/switch_mux is deprecated; please use topic_tools/mux_select instead");
  if (argc < 3)
  {
    printf("usage: switch MUXED_TOPIC SELECT_TOPIC\n");
    return 1;
  }
  std::string topic_name;
  if(!getBaseName(string(argv[1]), topic_name))
    return 1;
  ros::init(argc, argv, topic_name + string("_switcher"));
  ros::NodeHandle nh;
  string srv_name = string(argv[1]) + "_select";
  ROS_INFO("Waiting for service %s...\n", srv_name.c_str());
  ros::service::waitForService(srv_name, -1);
  ros::ServiceClient client = nh.serviceClient<MuxSelect>(srv_name);
  MuxSelect cmd;
  cmd.request.topic = argv[2];
  if (client.call(cmd))
  {
    ROS_INFO("muxed topic %s successfully switched from %s to %s",
             argv[1], cmd.response.prev_topic.c_str(),
             cmd.request.topic.c_str());
    return 0;
  }
  else
  {
    ROS_ERROR("failed to switch muxed topic %s to %s",
              argv[1], cmd.request.topic.c_str());
    return 1;
  }
}

