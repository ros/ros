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

int g_argc;
char** g_argv;


class RosTimeTest : public testing::Test
{
public:
  void setTime(ros::Time t)
  {
    rosgraph_msgs::Clock message;
    message.clock = t;
    pub_.publish(message);
  }

protected:
  RosTimeTest()
  {
    pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_;

};

TEST_F(RosTimeTest, RealTimeTest)
{
  //Get the start time.
  ros::Time start = ros::Time::now();

  //Checks to see if the time is larger than a thousand seconds
  //this is a good indication that we are getting the system time.
  ASSERT_TRUE(start.toSec() > 1000.0);

  //Wait a second
  ros::Duration wait(1, 0); wait.sleep();
  ros::Time end = ros::Time::now();
  ros::Duration d = end - start;

  //After waiting one second, see if we really waited on second.
  ASSERT_LT(d.toSec(), 1.1);
  ASSERT_GT(d.toSec(), 0.9);

  //Publish a rostime of 42.
  setTime(ros::Time(42, 0));

  //Wait half a second to make sure we get the message.
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  //Make sure that it has not been set
  ASSERT_NE(ros::Time::now().toSec(), 42.0);


  SUCCEED();
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "real_time_test");
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}

