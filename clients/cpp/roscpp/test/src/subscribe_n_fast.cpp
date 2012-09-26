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
    // A node is needed to make a service call
    ros::NodeHandle n;
    bool success;
    bool failure;
    std::string transport;
    bool reliable;
    int msgs_expected;
    int msgs_received;
    ros::Duration dt;

    void MsgCallback(const roscpp::TestArray::ConstPtr& msg)
    {
      if (failure || success)
        return;

      printf("received message %d\n", msg->counter);
      msgs_received++;
      if (reliable)
      {
        if (msgs_received != msg->counter)
        {
          failure = true;
          puts("failed");
        }
        if (msgs_received == (msgs_expected - 1))
        {
          success = true;
          puts("success");
        }
      }
      else
      {
        if (msgs_received > msg->counter)
        {
          failure = true;
          printf("failed: %d > %d\n", msgs_received, msg->counter);
        }
        if (msgs_received > (0.01 * msgs_expected))
        {
          success = true;
          puts("success");
        }
      }

    }

  protected:
    Subscriptions() {}
    void SetUp()
    {
      success = false;
      failure = false;

      msgs_received = -1;
      ASSERT_TRUE(g_argc == 4);
      transport = g_argv[1];
      msgs_expected = atoi(g_argv[2]);
      dt.fromSec(atof(g_argv[3]));
      if (transport == "tcp")
        reliable = true;
      else if (transport == "udp")
        reliable = false;
      else
      {
        ROS_ERROR("Unknown transport: %s", transport.c_str());
        FAIL();
      }
    }
    void TearDown()
    {
    }
};

TEST_F(Subscriptions, pubSubNFast)
{
  ros::TransportHints hints;
  if (reliable)
    hints.reliable();
  else
    hints.unreliable();

  ros::Subscriber sub = n.subscribe("roscpp/pubsub_test", msgs_expected, &Subscriptions::MsgCallback, (Subscriptions *)this, hints);
  
  ASSERT_TRUE(sub);

  ros::Time t1(ros::Time::now() + dt);
  
  while(ros::Time::now() < t1 && !success)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  
  printf("msgs_received == %d\n", msgs_received);
  if(success)
    SUCCEED();
  else
    FAIL();
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscriber");
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
