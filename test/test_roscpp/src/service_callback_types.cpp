
/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

/**
 * Author: Josh Faust
 */

/*
 * Test compilation of all the different service callback types
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "test_roscpp/TestStringString.h"

#include <vector>

bool add(test_roscpp::TestStringString::Request  &req,
         test_roscpp::TestStringString::Response &res )
{
  return true;
}

bool add2(ros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>& event)
{
  return true;
}

bool add3(ros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>& event, const std::string& bound)
{
  return true;
}

struct A
{
  bool add(test_roscpp::TestStringString::Request  &req,
           test_roscpp::TestStringString::Response &res )
  {
    return true;
  }

  bool add2(ros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>& event)
  {
    return true;
  }

  bool add3(ros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response>& event, const std::string& bound)
  {
    return true;
  }
};

TEST(ServiceCallbackTypes, compile)
{
  ros::NodeHandle n;

  std::vector<ros::ServiceServer> srvs;
  srvs.push_back(n.advertiseService("add_two_ints", add));
  srvs.push_back(n.advertiseService("add_two_ints2", add2));
  srvs.push_back(n.advertiseService<ros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response> >("add_two_ints3", boost::bind(add3, _1, std::string("blah"))));

  A a;
  srvs.push_back(n.advertiseService("add_two_ints10", &A::add, &a));
  srvs.push_back(n.advertiseService("add_two_ints11", &A::add2, &a));
  srvs.push_back(n.advertiseService<ros::ServiceEvent<test_roscpp::TestStringString::Request, test_roscpp::TestStringString::Response> >("add_two_ints12", boost::bind(&A::add3, &a, _1, std::string("blah"))));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init( argc, argv, "subscription_callback_types" );
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
