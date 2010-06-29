/*********************************************************************
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
********************************************************************/

// Bring in my package's API, which is what I'm testing
#include "ros/ros.h"
#include "topic_tools/shape_shifter.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

// Bring in gtest
#include <gtest/gtest.h>

class ShapeShifterSubscriber : public testing::Test
{
  public:

  bool success;

  void messageCallbackInt(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    try {
      std_msgs::Int32::Ptr s = msg->instantiate<std_msgs::Int32>();
    } catch (topic_tools::ShapeShifterException& e)
    {
      success = true;
    }
  }

  void messageCallbackString(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    try {
      std_msgs::String::Ptr s = msg->instantiate<std_msgs::String>();
      if (s->data == "chatter")
        success = true;
    } catch (topic_tools::ShapeShifterException& e)
    {

    }
  }

  void messageCallbackLoopback(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    try {
      std_msgs::String::Ptr s = msg->instantiate<std_msgs::String>();
      printf("Got data: %s", s->data.c_str());
      if (s->data == "abc123")
        success = true;
    } catch (topic_tools::ShapeShifterException& e)
    {
        printf("Instantiate failed!\n");
    }
  }
  
protected:
  ShapeShifterSubscriber() {}

  void SetUp()
  {
    success = false;
  }

  void TearDown()  {}
};


TEST_F(ShapeShifterSubscriber, testInstantiateString)
{
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>("input",1,&ShapeShifterSubscriber::messageCallbackString, (ShapeShifterSubscriber*)this);

  ros::Time t1(ros::Time::now()+ros::Duration(10.0));

  while(ros::Time::now() < t1 && !success)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  EXPECT_FALSE(topic_tools::ShapeShifter::uses_old_API_);

  if(success)
    SUCCEED();
  else
    FAIL();
}

TEST_F(ShapeShifterSubscriber, testInstantiateInt)
{
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>("input",1,&ShapeShifterSubscriber::messageCallbackInt, (ShapeShifterSubscriber*)this);

  ros::Time t1(ros::Time::now()+ros::Duration(10.0));

  while(ros::Time::now() < t1 && !success)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  EXPECT_FALSE(topic_tools::ShapeShifter::uses_old_API_);

  if(success)
    SUCCEED();
  else
    FAIL();
}

TEST_F(ShapeShifterSubscriber, testLoopback)
{
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>("loopback",1,&ShapeShifterSubscriber::messageCallbackLoopback, (ShapeShifterSubscriber*)this);

  ros::Time t1(ros::Time::now()+ros::Duration(10.0));

  ros::Publisher pub = nh.advertise<std_msgs::String>("loopback", 1);
  std_msgs::String s;
  s.data = "abc123";
  pub.publish(s);

  while(ros::Time::now() < t1 && !success)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  EXPECT_FALSE(topic_tools::ShapeShifter::uses_old_API_);

  if(success)
    SUCCEED();
  else
    FAIL();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "test_shapeshifter");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
