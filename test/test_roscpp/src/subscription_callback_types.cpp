
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

/**
 * Author: Josh Faust
 */

/*
 * Test compilation of all the different subscription callback types
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <vector>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void chatterCallback2(const std_msgs::String& msg)
{
  ROS_INFO("I heard(2): [%s]", msg.data.c_str());
}

void chatterCallback3(std_msgs::String::ConstPtr msg)
{
  ROS_INFO("I heard(3): [%s]", msg->data.c_str());
}

void chatterCallback4(const ros::MessageEvent<std_msgs::String const>& event)
{
  ROS_INFO("I heard(4): [%s, %s]", event.getMessage()->data.c_str(), event.getConnectionHeader()["callerid"].c_str());
}

void chatterCallback5(std_msgs::String msg)
{
  ROS_INFO("I heard(5): [%s]", msg.data.c_str());
}

void chatterCallback6(const std_msgs::String::Ptr& msg)
{
  ROS_INFO("I heard(6): [%s]", msg->data.c_str());
}

void chatterCallback7(std_msgs::String::Ptr msg)
{
  ROS_INFO("I heard(7): [%s]", msg->data.c_str());
}

void chatterCallback8(const ros::MessageEvent<std_msgs::String>& event)
{
  ROS_INFO("I heard(8): [%s, %s]", event.getMessage()->data.c_str(), event.getConnectionHeader()["callerid"].c_str());
}

struct A
{
  void chatterCallback(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("A I heard: [%s]", msg->data.c_str());
  }

  void chatterCallback2(const std_msgs::String& msg)
  {
    ROS_INFO("A I heard(2): [%s]", msg.data.c_str());
  }

  void chatterCallback3(std_msgs::String::ConstPtr msg)
  {
    ROS_INFO("A I heard(3): [%s]", msg->data.c_str());
  }

  void chatterCallback4(const ros::MessageEvent<std_msgs::String const>& event)
  {
    ROS_INFO("A I heard(4): [%s]", event.getMessage()->data.c_str());
  }

  void chatterCallback5(std_msgs::String msg)
  {
    ROS_INFO("A I heard(5): [%s]", msg.data.c_str());
  }

  void chatterCallback6(const std_msgs::String::ConstPtr& msg, const std::string& bound)
  {
    ROS_INFO("A I heard(6): [%s, %s]", msg->data.c_str(), bound.c_str());
  }

  void chatterCallback7(const std_msgs::String& msg, const std::string& bound)
  {
    ROS_INFO("A I heard(7): [%s, %s]", msg.data.c_str(), bound.c_str());
  }

  void chatterCallback8(const std_msgs::String::Ptr& msg)
  {
    ROS_INFO("A I heard(6): [%s]", msg->data.c_str());
  }

  void chatterCallback9(std_msgs::String::Ptr msg)
  {
    ROS_INFO("A I heard(7): [%s]", msg->data.c_str());
  }

  void chatterCallback10(const ros::MessageEvent<std_msgs::String>& event)
  {
    ROS_INFO("A I heard(8): [%s, %s]", event.getMessage()->data.c_str(), event.getConnectionHeader()["callerid"].c_str());
  }

  void constCallback(std_msgs::String::Ptr msg) const
  {
    ROS_INFO("A I heard(7): [%s] and I'm const", msg->data.c_str());
  }

};

TEST(SubscriptionCallbackTypes, compile)
{
  ros::NodeHandle n;

  std::vector<ros::Subscriber> subs;
  subs.push_back(n.subscribe("chatter", 1000, chatterCallback));
  subs.push_back(n.subscribe<std_msgs::String>("chatter", 1000, chatterCallback));
  subs.push_back(n.subscribe("chatter", 1000, chatterCallback2));
  subs.push_back(n.subscribe("chatter", 1000, chatterCallback3));
  subs.push_back(n.subscribe("chatter", 1000, chatterCallback4));
  subs.push_back(n.subscribe("chatter", 1000, chatterCallback5));
  subs.push_back(n.subscribe("chatter", 1000, chatterCallback6));
  subs.push_back(n.subscribe("chatter", 1000, chatterCallback7));
  subs.push_back(n.subscribe("chatter", 1000, chatterCallback8));


  A a;
  subs.push_back(n.subscribe("chatter", 1000, &A::chatterCallback, &a));
  subs.push_back(n.subscribe<std_msgs::String>("chatter", 1000, &A::chatterCallback, &a));
  subs.push_back(n.subscribe("chatter", 1000, &A::chatterCallback2, &a));
  subs.push_back(n.subscribe("chatter", 1000, &A::chatterCallback3, &a));
  subs.push_back(n.subscribe("chatter", 1000, &A::chatterCallback4, &a));
  subs.push_back(n.subscribe("chatter", 1000, &A::chatterCallback5, &a));

  subs.push_back(n.subscribe<std_msgs::String>("chatter", 1000, boost::bind(&A::chatterCallback6, &a, _1, std::string("hello"))));
  subs.push_back(n.subscribe<std_msgs::String, const std_msgs::String&>("chatter", 1000, boost::bind(&A::chatterCallback7, &a, _1, std::string("hello2"))));


  subs.push_back(n.subscribe("chatter", 1000, &A::chatterCallback8, &a));
  subs.push_back(n.subscribe("chatter", 1000, &A::chatterCallback9, &a));
  subs.push_back(n.subscribe("chatter", 1000, &A::chatterCallback10, &a));

  subs.push_back(n.subscribe("chatter", 1000, &A::constCallback, &a));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init( argc, argv, "subscription_callback_types" );
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
