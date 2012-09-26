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

#include <string>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <roscpp/TestArray.h>

static int g_argc;
static char** g_argv;

bool failure;
bool advertised;

class Publications : public testing::Test
{
public:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  void subscriberCallback(const ros::SingleSubscriberPublisher&)
  {
    ROS_INFO("subscriberCallback invoked");
    if(!advertised)
    {
      ROS_INFO("but not advertised");
      failure = true;
    }
  }

  bool adv()
  {
    pub_ = nh_.advertise<roscpp::TestArray>("roscpp/pubsub_test", 1, boost::bind(&Publications::subscriberCallback, this, _1));
    return pub_;
  }

  void unadv()
  {
    pub_.shutdown();
  }

protected:
  Publications() {}
  void SetUp()
  {
    advertised = false;
    failure = false;

    ASSERT_TRUE(g_argc == 1);
  }
  void TearDown()
  {
  }
};

TEST_F(Publications, pubUnadvertise)
{
  advertised = true;
  ROS_INFO("advertising");
  ASSERT_TRUE(adv());
  ros::Time t1(ros::Time::now()+ros::Duration(2.0));

  while(ros::Time::now() < t1 && !failure)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  unadv();

  ROS_INFO("unadvertised");
  advertised = false;

  ros::Time t2(ros::Time::now()+ros::Duration(2.0));
  while(ros::Time::now() < t2 && !failure)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  advertised = true;
  ASSERT_TRUE(adv());
  unadv();
  ASSERT_TRUE(adv());

  if(failure)
    FAIL();
  else
    SUCCEED();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_unadvertise");
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
