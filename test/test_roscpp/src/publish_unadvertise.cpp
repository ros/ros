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

#include "ros/node.h"
#include <test_roscpp/TestArray.h>

static int g_argc;
static char** g_argv;

static bool failure;
static bool advertised;

// An auxilliary class created to hold subscriber callback, because there's
// no way to advertise() with such a callback from outside a ros::Node
class Dummy : public ros::Node
{
  public:
    test_roscpp::TestArray msg;
    Dummy(std::string name, uint32_t options) : ros::Node(name,options) {}

    void sub_cb(const ros::SingleSubscriberPublisher&)
    {
      puts("sub_cb invoked");
      if(!advertised)
        failure = true;
    }

    bool adv()
    { return advertise("test_roscpp/pubsub_test", msg, &Dummy::sub_cb, 1); }
    bool unadv()
    { return unadvertise("test_roscpp/pubsub_test"); }
};

class Pub : public testing::Test
{
  public:
    Dummy* n;
    ros::Duration dt;


  protected:
    Pub() {}
    void SetUp()
    {
      ros::init(g_argc, g_argv);
      advertised = false;
      failure = false;

      ASSERT_TRUE(g_argc == 1);
      n = new Dummy("publisher",0);
    }
    void TearDown()
    {

      delete n;
    }
};

TEST_F(Pub, pubUnadvertise)
{
  advertised = true;
  puts("advertising");
  ASSERT_TRUE(n->adv());
  ASSERT_FALSE(n->adv());
  ros::Time t1(ros::Time::now()+ros::Duration(2.0));

  while(ros::Time::now() < t1 && !failure)
  {
    ros::WallDuration(0.01).sleep();
  }

  ASSERT_TRUE(n->unadv());
  ASSERT_FALSE(n->unadv());
  puts("unadvertised");
  advertised = false;

  ros::Time t2(ros::Time::now()+ros::Duration(2.0));
  while(ros::Time::now() < t2 && !failure)
  {
    ros::WallDuration(0.01).sleep();
  }

  advertised = true;
  ASSERT_TRUE(n->adv());
  ASSERT_TRUE(n->unadv());
  ASSERT_TRUE(n->adv());

  if(failure)
    FAIL();
  else
    SUCCEED();
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
