/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include <gtest/gtest.h>

#include "ros/rt/publisher.h"

#include <ros/ros.h>

#include <std_msgs/UInt32.h>

using namespace ros::rt;

struct Helper
{
  Helper()
  : count(0)
  {}

  void cb(const std_msgs::UInt32ConstPtr& msg)
  {
    latest = msg;
    ++count;
  }

  std_msgs::UInt32ConstPtr latest;
  uint32_t count;
};

TEST(Publisher, publish)
{
  ros::NodeHandle nh;

  Publisher<std_msgs::UInt32> pub(nh.advertise<std_msgs::UInt32>("test", 0), 1, std_msgs::UInt32());

  Helper h;
  ros::Subscriber sub = nh.subscribe("test", 0, &Helper::cb, &h);

  std_msgs::UInt32Ptr msg = pub.allocate();
  msg->data = 5;
  pub.publish(msg);

  while (h.count == 0)
  {
    ros::WallDuration(0.001).sleep();
    ros::spinOnce();
  }

  ASSERT_EQ(h.count, 1UL);
  ASSERT_EQ(h.latest->data, 5UL);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_rt_publisher");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;
  initThread();

  return RUN_ALL_TESTS();
}
