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

#include <stdlib.h>

#include "ros/node.h"
#include <test_roscpp/TestArray.h>

int g_msg_count;
ros::Duration g_dt;
uint32_t g_options = 0;

class SelfSubscribe : public testing::Test, public ros::Node
{
  public:
    test_roscpp::TestArray msg;
    bool success;
    bool failure;
    int msg_i;

    void MsgCallback()
    {
      printf("received message %d\n", msg.counter);
      if(failure || success)
        return;

      msg_i++;
      if(msg_i != msg.counter)
      {
        failure = true;
        puts("failed");
      }
      else if(msg_i == (g_msg_count-1))
      {
        success = true;
        puts("success");
      }
    }

    void sub_cb(const ros::SingleSubscriberPublisher&)
    {
      test_roscpp::TestArray outmsg;
      outmsg.set_float_arr_size(100);
      for(int i=0;i<g_msg_count;i++)
      {
        outmsg.counter = i;
        publish("test_roscpp/pubsub_test", outmsg);
        printf("published %d\n", i);
      }
    }

  protected:
    SelfSubscribe() : ros::Node("subscriber", g_options ) {}
    void SetUp()
    {
    }
    void TearDown()
    {

    }
};

TEST_F(SelfSubscribe, advSub)
{
  ros::Duration d;
  d.fromNSec(10000000);

  success = false;
  failure = false;
  msg_i = -1;

  test_roscpp::TestArray tmp;
  ASSERT_TRUE(advertise("test_roscpp/pubsub_test", tmp, &SelfSubscribe::sub_cb, g_msg_count));
  ASSERT_TRUE(subscribe("test_roscpp/pubsub_test", msg, &SelfSubscribe::MsgCallback, g_msg_count));
  ros::Time t1(ros::Time::now()+g_dt);
  while(ros::Time::now() < t1 && !success && !failure)
  {
    d.sleep();
  }

  ASSERT_TRUE(unsubscribe("test_roscpp/pubsub_test"));
  ASSERT_TRUE(unadvertise("test_roscpp/pubsub_test"));

  if(success)
    SUCCEED();
  else
    FAIL();
  // Now try the other order
  success = false;
  failure = false;
  msg_i = -1;

  ASSERT_TRUE(subscribe("test_roscpp/pubsub_test", msg, &SelfSubscribe::MsgCallback, g_msg_count));
  ASSERT_TRUE(advertise("test_roscpp/pubsub_test", tmp, &SelfSubscribe::sub_cb, g_msg_count));
  t1 = ros::Time(ros::Time::now()+g_dt);

  while(ros::Time::now() < t1 && !success && !failure)
  {
    d.sleep();
  }

  if(success)
    SUCCEED();
  else
    FAIL();
}

#define USAGE "USAGE: sub_pub <count> <time>"

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv);

  if(argc != 3)
  {
    puts(USAGE);
    return -1;
  }
  g_msg_count = atoi(argv[1]);
  g_dt.fromSec(atof(argv[2]));

  return RUN_ALL_TESTS();
}
