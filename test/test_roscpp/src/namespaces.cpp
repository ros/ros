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

/* Author: Josh Faust */

/*
 * Test namespaces
 */

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/param.h>

TEST(namespaces, param)
{
  std::string param;
  ASSERT_TRUE( ros::param::get( "parent", param ) );
  ROS_INFO("parent=%s", param.c_str());
  ASSERT_EQ(param, ":ROS_NAMESPACE:parent");
}

TEST(namespaces, localParam)
{
  std::string param;
  ASSERT_TRUE( ros::param::get( "~/local", param ) );
  ROS_INFO("~/local=%s", param.c_str());
  ASSERT_EQ(param, ":ROS_NAMESPACE:NODE_NAME:local");

  ros::NodeHandle n("~");
  std::string param2;
  n.param<std::string>("local", param2, param);
  ASSERT_STREQ(param2.c_str(), param.c_str());
  ASSERT_STREQ(param2.c_str(), ":ROS_NAMESPACE:NODE_NAME:local");
}

TEST(namespaces, globalParam)
{
  std::string param;
  ASSERT_TRUE( ros::param::get( "/global", param ) );
  ASSERT_EQ(param, ":global");
}

TEST(namespaces, otherNamespaceParam)
{
  std::string param;
  ASSERT_TRUE( ros::param::get( "/other_namespace/param", param ) );
  ASSERT_EQ(param, ":other_namespace:param");
}

TEST(namespaces, name)
{
  std::string name = ros::this_node::getName();
  ASSERT_EQ(name, "/ROS_NAMESPACE/NODE_NAME");
  std::string nspace = ros::this_node::getNamespace();
  ASSERT_EQ(nspace, "/ROS_NAMESPACE");
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init( argc, argv, "namespaces" );

  return RUN_ALL_TESTS();
}
