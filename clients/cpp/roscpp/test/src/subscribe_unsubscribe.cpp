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

#include <boost/thread.hpp>

int g_argc;
char** g_argv;

class Subscriptions : public testing::Test
{
public:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  void messageCallback(const roscpp::TestArrayConstPtr& msg)
  {
    ROS_INFO("in callback");

    if(!sub_)
    {
      ROS_INFO("but not subscribed!");
      FAIL();
    }
  }

  void autoUnsubscribeCallback(const roscpp::TestArrayConstPtr& msg)
  {
    ROS_INFO("in autounsub callback");
    sub_.shutdown();
  }

protected:
  Subscriptions() {}
};

TEST_F(Subscriptions, subUnsub)
{
  sub_.shutdown();

  for(int i=0;i<100;i++)
  {
    if(!sub_)
    {
      sub_ = nh_.subscribe("roscpp/pubsub_test", 0, &Subscriptions::messageCallback, (Subscriptions*)this);
      ASSERT_TRUE(sub_);
    }
    else
    {
      sub_.shutdown();
      ASSERT_FALSE(sub_);
    }

    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }
}

TEST_F(Subscriptions, unsubInCallback)
{
  sub_ = nh_.subscribe("roscpp/pubsub_test", 0, &Subscriptions::autoUnsubscribeCallback, (Subscriptions*)this);

  while (sub_ && ros::ok())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }
}

void spinThread(bool volatile* cont)
{
  while (*cont)
  {
    ros::spinOnce();
    ros::Duration(0.001).sleep();
  }
}

void unsubscribeAfterBarrierWait(boost::barrier* barrier, ros::Subscriber& sub)
{
  barrier->wait();

  sub.shutdown();
}

TEST_F(Subscriptions, unsubInCallbackAndOtherThread)
{
  boost::barrier barrier(2);
  for (int i = 0; i < 100; ++i)
  {
    ros::Subscriber sub;
    sub_ = nh_.subscribe<roscpp::TestArray>("roscpp/pubsub_test", 1, boost::bind(unsubscribeAfterBarrierWait, &barrier, boost::ref(sub)));
    sub = sub_;

    bool cont = true;
    boost::thread t(spinThread, &cont);

    barrier.wait();

    sub_.shutdown();
    cont = false;
    t.join();
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  g_argc = argc;
  g_argv = argv;

  ros::init(g_argc, g_argv, "subscribe_unsubscribe");

  if (g_argc != 1)
  {
    puts("Too many arguments\n");
    return -1;
  }

  return RUN_ALL_TESTS();
}
