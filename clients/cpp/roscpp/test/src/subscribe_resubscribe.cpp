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

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <roscpp/TestArray.h>

int g_argc;
char** g_argv;

class Subscriptions : public testing::Test
{
  public:
    bool success;
    bool failure;
    int msg_count;
    int msg_i;
    ros::Duration dt;

    void messageCallback(const roscpp::TestArrayConstPtr& msg)
    {
      if(failure || success)
        return;

      ROS_INFO("received message %d", msg->counter);
      msg_i++;
      if(msg_i != msg->counter)
      {
        failure = true;
        ROS_INFO("failed");
      }
      if(msg_i == (msg_count-1))
      {
        success = true;
        ROS_INFO("success");
      }
    }

  protected:
    Subscriptions() {}
    void SetUp()
    {
      success = false;
      failure = false;

      msg_i = -1;
      ASSERT_TRUE(g_argc == 3);
      msg_count = atoi(g_argv[1]);
      dt.fromSec(atof(g_argv[2]));
    }
    void TearDown()
    {
    }
};

TEST_F(Subscriptions, resubscribe)
{
  ros::NodeHandle nh;

  {
    ros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", 0, &Subscriptions::messageCallback, (Subscriptions*)this);
    ASSERT_TRUE(sub);
    ros::Time t1(ros::Time::now()+dt);

    while(ros::Time::now() < t1 && !success)
    {
      ros::WallDuration(0.01).sleep();
      ros::spinOnce();
    }
  }

  if(!success)
    FAIL();
  else
  {
    success = false;
    failure = false;
    msg_i = -1;

    {
      ros::Subscriber sub = nh.subscribe("roscpp/pubsub_test2", 0, &Subscriptions::messageCallback, (Subscriptions*)this);
      ASSERT_TRUE(sub);
      ros::Time t1(ros::Time::now()+dt);

      while(ros::Time::now() < t1 && !success)
      {
        ros::WallDuration(0.01).sleep();
        ros::spinOnce();
      }
    }

    if(success)
      SUCCEED();
    else
      FAIL();
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "subscribe_resubscribe");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
