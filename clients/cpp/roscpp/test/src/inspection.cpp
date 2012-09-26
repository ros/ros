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
 * Test inspection functionality
 */

#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/names.h>
#include <roscpp/TestArray.h>
#include <roscpp/TestStringInt.h>
#include <roscpp/TestEmpty.h>

const char* g_node_name = "inspection_test";

int g_argc;
char* g_argv[8];

TEST(Inspection, getAdvertisedTopics)
{
  ros::NodeHandle nh;

  std::vector<std::string> topics;

  ros::this_node::getAdvertisedTopics(topics);
  // Note that it's 1, not 0, because the rosout appender has already snuck
  // in and advertised.
  ASSERT_EQ((int)topics.size(),1);
  ASSERT_EQ(topics[0], "/rosout");

  {
    ros::Publisher pub1 = nh.advertise<roscpp::TestArray>("topic",1);
    ros::Publisher pub2 = nh.advertise<roscpp::TestArray>("ns/topic",1);
    ros::Publisher pub3 = nh.advertise<roscpp::TestArray>("/global/topic",1);

    topics.clear();
    ros::this_node::getAdvertisedTopics(topics);
    // Note that it's 4, not 3, because the rosout appender has already snuck
    // in and advertised.
    ASSERT_EQ((int)topics.size(),4);

    // The following tests assume strict ordering of the topics, which is not
    // guaranteed by the API.
    ASSERT_EQ(topics[0], "/rosout");
    ASSERT_EQ(topics[1], "/topic");
    ASSERT_EQ(topics[2], "/ns/topic");
    ASSERT_EQ(topics[3], "/global/topic");
  }

  topics.clear();
  ros::this_node::getAdvertisedTopics(topics);
  // Note that it's 1, not 0, because the rosout appender has already snuck
  // in and advertised.
  ASSERT_EQ((int)topics.size(),1);
  ASSERT_EQ(topics[0], "/rosout");
}

TEST(Inspection, commandLineParsing)
{
  ASSERT_EQ(g_argc, 5);
  ros::M_string remappings = ros::names::getRemappings();

  ASSERT_STREQ(remappings["/foo"].c_str(), "/bar");
  ASSERT_STREQ(remappings["/baz"].c_str(), "/bang");
  ASSERT_STREQ(remappings["/bomb"].c_str(), "/barn");
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  g_argc = 8;

  g_argv[0] = strdup(argv[0]);
  g_argv[1] = strdup("foo:=bar");
  g_argv[2] = strdup("bat");
  g_argv[3] = strdup("baz:=bang");
  g_argv[4] = strdup("boom");
  g_argv[5] = strdup("baz=bomb");
  g_argv[6] = strdup("bomb:=barn");
  g_argv[7] = strdup("--bangbang");

  ros::init( g_argc, g_argv, "inspection" );
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
