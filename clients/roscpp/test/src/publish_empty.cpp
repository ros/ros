/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/* Author: Brian Gerkey */

/*
 * Publish an empty message N times, back to back
 */

#include <string>
#include <cstdio>
#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <roscpp/TestEmpty.h>

int32_t g_msg_count = 0;
void subscriberCallback(const ros::SingleSubscriberPublisher&, ros::Publisher& pub)
{
  roscpp::TestEmpty msg;
  for(int i = 0; i < g_msg_count; i++)
  {
    pub.publish(msg);
    ROS_INFO("published message %d", i);
  }
}

#define USAGE "USAGE: publish_empty <count>"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_onsub");

  if(argc != 2)
  {
    puts(USAGE);
    exit(-1);
  }

  g_msg_count = atoi(argv[1]);

  ros::NodeHandle nh;
  ros::Publisher pub;
  pub = nh.advertise<roscpp::TestEmpty>("roscpp/pubsub_test", g_msg_count, boost::bind(subscriberCallback, _1, boost::ref(pub)));

  ros::spin();
}
