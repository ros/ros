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

#include "ros/node.h"
#include <test_roscpp/TestArray.h>

#include <boost/thread/barrier.hpp>

int g_argc;
char** g_argv;

ros::Node* g_n;

class Subscriptions : public testing::Test
{
  public:
    test_roscpp::TestArray msg;
    bool subscribed;

    void MsgCallback()
    {
      puts("in callback");

      if(!subscribed)
      {
        puts("but not subscribed!");
        FAIL();
      }
    }

    void AutoUnsubscribeCallback()
    {
      puts("in autounsub callback");
      g_n->unsubscribe("test_roscpp/pubsub_test");

      subscribed = false;
    }

  protected:
    Subscriptions() {}
};

TEST_F(Subscriptions, subUnsub)
{
  subscribed = false;
  for(int i=0;i<100;i++)
  {
    if(!subscribed)
    {
      subscribed = true;
      ASSERT_TRUE(g_n->subscribe("test_roscpp/pubsub_test", msg, &Subscriptions::MsgCallback, (Subscriptions*)this, 1));
    }
    else
    {
      ASSERT_TRUE(g_n->unsubscribe("test_roscpp/pubsub_test"));
      subscribed = false;
    }

    ros::WallDuration(0.01).sleep();
  }
}

TEST_F(Subscriptions, unsubInCallback)
{
  ASSERT_TRUE(g_n->subscribe("test_roscpp/pubsub_test", msg, &Subscriptions::AutoUnsubscribeCallback, (Subscriptions*)this, 1));
  subscribed = true;

  while (subscribed && g_n->ok())
  {
    ros::WallDuration(0.01).sleep();
  }
}

boost::barrier* g_barrier = NULL;
void unsubscribeAfterBarrierWait()
{
  g_barrier->wait();

  g_n->unsubscribe("test_roscpp/pubsub_test");
}

TEST_F(Subscriptions, unsubInCallbackAndOtherThread)
{
  for (int i = 0; i < 100; ++i)
  {
    g_barrier = new boost::barrier(2);

    ASSERT_TRUE(g_n->subscribe("test_roscpp/pubsub_test", msg, unsubscribeAfterBarrierWait, 1));
    g_barrier->wait();

    g_n->unsubscribe("test_roscpp/pubsub_test");

    delete g_barrier;
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  g_argc = argc;
  g_argv = argv;

  ros::init(g_argc, g_argv);

  if (g_argc != 1)
  {
    puts("Too many arguments\n");
    return -1;
  }

  g_n = new ros::Node("subscriber");

  int ret = RUN_ALL_TESTS();


  delete g_n;

  return ret;
}
