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
 * Advertise a service
 */

#include <gtest/gtest.h>

#include "ros/ros.h"
#include <test_roscpp/TestStringString.h>


static int g_argc;
static char** g_argv;

class ServiceAdvertiser : public testing::Test
{
  public:
    ros::NodeHandle nh_;
    ros::ServiceServer srv_;

    bool advertised_;
    bool failure_;

    bool srvCallback(test_roscpp::TestStringString::Request  &req,
                     test_roscpp::TestStringString::Response &res)
    {
      ROS_INFO("in callback");
      if(!advertised_)
      {
        ROS_INFO("but not advertised!");
        failure_ = true;
      }
      return true;
    }
  protected:
    ServiceAdvertiser() {}
    void SetUp()
    {
      failure_ = false;
      advertised_ = false;

      ASSERT_TRUE(g_argc == 1);
    }

    bool adv()
    {
      ROS_INFO("advertising");
      srv_ = nh_.advertiseService("service_adv", &ServiceAdvertiser::srvCallback, this);
      ROS_INFO("advertised");
      return srv_;
    }
    bool unadv()
    {
      ROS_INFO("unadvertising");
      srv_.shutdown();
      ROS_INFO("unadvertised");
      return true;
    }
};

TEST_F(ServiceAdvertiser, advUnadv)
{
  advertised_ = true;
  ASSERT_TRUE(adv());

  for(int i=0;i<100;i++)
  {
    if(advertised_)
    {
      ASSERT_TRUE(unadv());
      advertised_ = false;
    }
    else
    {
      advertised_ = true;
      ASSERT_TRUE(adv());
    }

    ros::WallDuration(0.01).sleep();
  }

  if(failure_)
    FAIL();
  else
    SUCCEED();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "service_adv_unadv");
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}

