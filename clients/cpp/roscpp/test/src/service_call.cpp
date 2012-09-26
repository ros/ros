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
 * Call a service
 */

#include <string>

#include <gtest/gtest.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/service.h"
#include "ros/connection.h"
#include "ros/service_client.h"
#include <roscpp/TestStringString.h>
#include <roscpp/BadTestStringString.h>

TEST(SrvCall, callSrv)
{
  roscpp::TestStringString::Request req;
  roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(ros::service::waitForService("service_adv"));
  ASSERT_TRUE(ros::service::call("service_adv", req, res));

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvMultipleTimes)
{
  roscpp::TestStringString::Request req;
  roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(ros::service::waitForService("service_adv"));

  ros::Time start = ros::Time::now();

  for (int i = 0; i < 100; ++i)
  {
    ASSERT_TRUE(ros::service::call("service_adv", req, res));
  }

  ros::Time end = ros::Time::now();
  ros::Duration d = end - start;
  printf("100 calls took %f secs\n", d.toSec());

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvWithWrongType)
{
  roscpp::BadTestStringString::Request req;
  roscpp::BadTestStringString::Response res;

  ASSERT_TRUE(ros::service::waitForService("service_adv"));

  for ( int i = 0; i < 4; ++i )
  {
    bool call_result = ros::service::call("service_adv", req, res);
    ASSERT_FALSE(call_result);
  }
}

TEST(SrvCall, callSrvHandle)
{
  roscpp::TestStringString::Request req;
  roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  std::map<std::string, std::string> header;
  header["test1"] = "testing 1";
  header["test2"] = "testing 2";
  ros::NodeHandle nh;
  ros::ServiceClient handle = nh.serviceClient<roscpp::TestStringString>("service_adv", false, header);
  ASSERT_TRUE(handle.waitForExistence());

  ros::Time start = ros::Time::now();

  for (int i = 0; i < 100; ++i)
  {
    ASSERT_TRUE(handle.call(req, res));
  }

  ros::Time end = ros::Time::now();
  ros::Duration d = end - start;
  printf("100 calls took %f secs\n", d.toSec());

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvPersistentHandle)
{
  roscpp::TestStringString::Request req;
  roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(ros::service::waitForService("service_adv"));

  std::map<std::string, std::string> header;
  header["test1"] = "testing 1";
  header["test2"] = "testing 2";
  ros::NodeHandle nh;
  ros::ServiceClient handle = nh.serviceClient<roscpp::TestStringString>("service_adv", true, header);

  ros::Time start = ros::Time::now();

  for (int i = 0; i < 10000; ++i)
  {
    ASSERT_TRUE(handle.call(req, res));
  }

  ros::Time end = ros::Time::now();
  ros::Duration d = end - start;
  printf("10000 calls took %f secs\n", d.toSec());

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvLongRunning)
{
  roscpp::TestStringString::Request req;
  roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(ros::service::waitForService("service_adv_long"));
  ASSERT_TRUE(ros::service::call("service_adv_long", req, res));

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, callSrvWhichUnadvertisesInCallback)
{
  roscpp::TestStringString::Request req;
  roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(ros::service::waitForService("service_adv_unadv_in_callback"));
  ASSERT_FALSE(ros::service::call("service_adv_unadv_in_callback", req, res));
}

TEST(SrvCall, handleValid)
{
  roscpp::TestStringString::Request req;
  roscpp::TestStringString::Response res;

  req.str = std::string("case_FLIP");

  ASSERT_TRUE(ros::service::waitForService("service_adv"));

  std::map<std::string, std::string> header;
  header["test1"] = "testing 1";
  header["test2"] = "testing 2";
  ros::NodeHandle nh;
  ros::ServiceClient handle = nh.serviceClient<roscpp::TestStringString>("service_adv", true, header);
  ASSERT_TRUE(handle.call(req, res));
  ASSERT_TRUE(handle.isValid());
  handle.shutdown();
  ASSERT_FALSE(handle.isValid());

  ASSERT_STREQ(res.str.c_str(), "CASE_flip");
}

TEST(SrvCall, waitForServiceTimeout)
{
  ros::NodeHandle nh;
  ASSERT_FALSE(ros::service::waitForService("iojergoiwjoiewg", 1000));
  ASSERT_FALSE(ros::service::waitForService("iojergoiwjoiewg", ros::Duration(1)));

  ros::ServiceClient handle = nh.serviceClient<roscpp::TestStringString>("migowiowejowieuhwejg", false);
  ASSERT_FALSE(handle.waitForExistence(ros::Duration(1)));
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "service_call");
  ros::NodeHandle nh;

  int ret = RUN_ALL_TESTS();



  return ret;
}


