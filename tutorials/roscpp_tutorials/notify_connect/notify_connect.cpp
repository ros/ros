/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates how to get a callback when a new subscriber connects
 * to an advertised topic, or a subscriber disconnects.
 */

uint32_t g_count = 0;

void chatterConnect(const ros::SingleSubscriberPublisher& pub)
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "chatter connect #" << g_count++;
  ROS_INFO("%s", ss.str().c_str());
  msg.data = ss.str();

  pub.publish(msg);  // This message will get published only to the subscriber that just connected
}

void chatterDisconnect(const ros::SingleSubscriberPublisher& pub)
{
  ROS_INFO("chatter disconnect");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "notify_connect");
  ros::NodeHandle n;

  /**
   * This version of advertise() optionally takes a connect/disconnect callback
   */
  ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000, chatterConnect, chatterDisconnect);

  ros::spin();

  return 0;
}

