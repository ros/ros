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
 * Subscribe to a topic, expecting to get a single message.
 */

#include <string>

#include <gtest/gtest.h>

#include <stdlib.h>

#include "ros/ros.h"
#include <roscpp/TestArray.h>

int g_msg_count;
ros::Duration g_dt;
uint32_t g_options = 0;
bool g_success = false;
bool g_failure = false;
int32_t g_msg_i = -1;

void subscriberCallback(const ros::SingleSubscriberPublisher&, const ros::Publisher& pub)
{
  roscpp::TestArray outmsg;
  for(int i=0;i<g_msg_count;i++)
  {
    outmsg.counter = i;
    pub.publish(outmsg);
    ROS_INFO("published %d", i);
  }
}

void messageCallback(const roscpp::TestArrayConstPtr& msg)
{
  ROS_INFO("received message %d", msg->counter);
  if(g_failure || g_success)
    return;

  g_msg_i++;
  if(g_msg_i != msg->counter)
  {
    g_failure = true;
    ROS_INFO("failed");
  }
  else if(g_msg_i == (g_msg_count-1))
  {
    g_success = true;
    ROS_INFO("success");
  }
}

TEST(SelfSubscribe, advSub)
{
  ros::NodeHandle nh;
  ros::Duration d;
  d.fromNSec(10000000);

  g_success = false;
  g_failure = false;
  g_msg_i = -1;

  {
    ros::Publisher pub;
    pub = nh.advertise<roscpp::TestArray>("roscpp/pubsub_test", g_msg_count, boost::bind(subscriberCallback, _1, boost::ref(pub)));
    ASSERT_TRUE(pub);
    ros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", g_msg_count, messageCallback);
    ASSERT_TRUE(sub);
    ros::Time t1(ros::Time::now()+g_dt);
    while(ros::Time::now() < t1 && !g_success && !g_failure)
    {
      d.sleep();
      ros::spinOnce();
    }
  }

  ASSERT_TRUE(g_success);
  ASSERT_FALSE(g_failure);

  // Now try the other order
  g_success = false;
  g_failure = false;
  g_msg_i = -1;

  {
    ros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", g_msg_count, messageCallback);
    ASSERT_TRUE(sub);
    ros::Publisher pub;
    pub = nh.advertise<roscpp::TestArray>("roscpp/pubsub_test", g_msg_count, boost::bind(subscriberCallback, _1, boost::ref(pub)));
    ASSERT_TRUE(pub);

    ros::Time t1(ros::Time::now()+g_dt);
    while(ros::Time::now() < t1 && !g_success && !g_failure)
    {
      d.sleep();
      ros::spinOnce();
    }
  }


  ASSERT_TRUE(g_success);
  ASSERT_FALSE(g_failure);
}

#define USAGE "USAGE: sub_pub <count> <time>"

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscribe_self");

  if(argc != 3)
  {
    puts(USAGE);
    return -1;
  }

  g_msg_count = atoi(argv[1]);
  g_dt.fromSec(atof(argv[2]));

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
