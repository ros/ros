/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

/* Author: Josh Faust */

/*
 * Subscribe intraprocess, ensuring that no copy happens
 */

#include <string>

#include <gtest/gtest.h>

#include <stdlib.h>

#include "ros/ros.h"

#include "roscpp/TestEmpty.h"


struct ConstHelper
{
  void callback(const roscpp::TestEmptyConstPtr& msg)
  {
    message_ = msg;
  }

  roscpp::TestEmptyConstPtr message_;
};

struct NonConstHelper
{
  void callback(const roscpp::TestEmptyPtr& msg)
  {
    message_ = msg;
  }

  roscpp::TestEmptyPtr message_;
};

TEST(NonConstSubscriptions, oneNonConstSubscriber)
{
  NonConstHelper h;
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, &NonConstHelper::callback, &h);
  ros::Publisher pub = nh.advertise<roscpp::TestEmpty>("test", 0);

  roscpp::TestEmptyPtr msg(new roscpp::TestEmpty);
  pub.publish(msg);
  ros::spinOnce();

  ASSERT_TRUE(h.message_);
  EXPECT_EQ(h.message_, msg);
}

TEST(NonConstSubscriptions, oneConstOneNonConst)
{
  NonConstHelper h;
  ConstHelper h2;
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, &NonConstHelper::callback, &h);
  ros::Subscriber sub2 = nh.subscribe("test", 0, &ConstHelper::callback, &h2);
  ros::Publisher pub = nh.advertise<roscpp::TestEmpty>("test", 0);

  roscpp::TestEmptyPtr msg(new roscpp::TestEmpty);
  pub.publish(msg);
  ros::spinOnce();

  ASSERT_TRUE(h.message_);
  EXPECT_NE(h.message_, msg);
  EXPECT_EQ(h2.message_, msg);
}

TEST(NonConstSubscriptions, twoNonConst)
{
  NonConstHelper h;
  NonConstHelper h2;
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, &NonConstHelper::callback, &h);
  ros::Subscriber sub2 = nh.subscribe("test", 0, &NonConstHelper::callback, &h2);
  ros::Publisher pub = nh.advertise<roscpp::TestEmpty>("test", 0);

  roscpp::TestEmptyPtr msg(new roscpp::TestEmpty);
  pub.publish(msg);
  ros::spinOnce();

  ASSERT_TRUE(h.message_);
  EXPECT_NE(h.message_, msg);
}

TEST(NonConstSubscriptions, twoConst)
{
  ConstHelper h;
  ConstHelper h2;
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 0, &ConstHelper::callback, &h);
  ros::Subscriber sub2 = nh.subscribe("test", 0, &ConstHelper::callback, &h2);
  ros::Publisher pub = nh.advertise<roscpp::TestEmpty>("test", 0);

  roscpp::TestEmptyPtr msg(new roscpp::TestEmpty);
  pub.publish(msg);
  ros::spinOnce();

  ASSERT_TRUE(h.message_);
  EXPECT_EQ(h.message_, msg);
  EXPECT_EQ(h2.message_, msg);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "intraprocess_subscriptions");

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}

