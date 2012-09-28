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

/* Author: Tony Pratkanis */

/*
 * Subscribe to a topic multiple times
 */

#include <string>
#include <gtest/gtest.h>
#include <time.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

int g_argc;
char** g_argv;

class RosClockTest : public testing::Test
{
public:
  void setTime(ros::Time t)
  {
    rosgraph_msgs::Clock message;
    message.clock = t;
    pub_.publish(message);
  }

protected:
  RosClockTest()
  {
    pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
    while (pub_.getNumSubscribers() == 0)
    {
      ros::WallDuration(0.01).sleep();
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_;

};

TEST_F(RosClockTest, SimClockTest)
{
  //Get the start time.
  ros::Time start = ros::Time::now();

  //The start time should be zero before a message is published.
  ASSERT_TRUE(start.isZero());

  //Publish a rostime of 42.
  setTime(ros::Time(42, 0));

  //Wait half a second to get the message.
  ros::WallDuration(0.5).sleep();

  //Make sure that it is really set
  ASSERT_EQ(42.0, ros::Time::now().toSec());
}

void sleepThread(bool* done)
{
  bool ok = ros::Duration(1.0).sleep();
  if (!ok)
  {
    ROS_ERROR("!OK");
  }
  *done = true;
}

TEST(Clock, sleepFromZero)
{
  ros::Time::setNow(ros::Time());
  bool done = false;
  boost::thread t(boost::bind(sleepThread, &done));

  ros::WallDuration(1.0).sleep();
  ros::WallTime start = ros::WallTime::now();
  ros::Time::setNow(ros::Time(ros::WallTime::now().sec, ros::WallTime::now().nsec));
  while (!done)
  {
    ros::WallDuration(0.001).sleep();
    ros::WallTime now = ros::WallTime::now();
    ros::Time::setNow(ros::Time(now.sec, now.nsec));
  }
  ros::WallTime end = ros::WallTime::now();
  EXPECT_GE(end - start, ros::WallDuration(1.0));
}

TEST(Clock, isTimeValid)
{
  ros::Time::setNow(ros::Time());
  ASSERT_FALSE(ros::Time::isValid());
  ros::Time::setNow(ros::TIME_MIN);
  ASSERT_TRUE(ros::Time::isValid());
}

void waitThread(bool* done)
{
  ros::Time::waitForValid();
  *done = true;
}

TEST(Clock, waitForValid)
{
  ros::Time::setNow(ros::Time());

  // Test timeout
  ros::WallTime start = ros::WallTime::now();
  ASSERT_FALSE(ros::Time::waitForValid(ros::WallDuration(1.0)));
  ros::WallTime end = ros::WallTime::now();
  ASSERT_GT(end - start, ros::WallDuration(1.0));

  bool done = false;
  boost::thread t(boost::bind(waitThread, &done));

  ros::WallDuration(1.0).sleep();
  ASSERT_FALSE(done);
  ros::Time::setNow(ros::TIME_MIN);
  while (!done)
  {
    ros::WallDuration(0.01).sleep();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "sim_time_test");
  ros::NodeHandle nh;
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}

