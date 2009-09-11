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
 * Test parameters
 */

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "ros/node.h"

ros::Node* g_node;
const char* g_node_name = "test_node";

TEST(params, allParamTypes)
{
  std::string string_param;
  EXPECT_TRUE( g_node->getParam( "string", string_param ) );
  EXPECT_TRUE( string_param == "test" );

  int int_param = 0;
  EXPECT_TRUE( g_node->getParam( "int", int_param ) );
  EXPECT_TRUE( int_param == 10 );

  double double_param = 0.0;
  EXPECT_TRUE( g_node->getParam( "double", double_param ) );
  EXPECT_DOUBLE_EQ( double_param, 10.5 );

  bool bool_param = true;
  EXPECT_TRUE( g_node->getParam( "bool", bool_param ) );
  EXPECT_FALSE( bool_param );
}

TEST(params, setThenGetString)
{
  g_node->setParam( "test_set_param", std::string("asdf") );
  std::string param;
  ASSERT_TRUE( g_node->getParam( "test_set_param", param ) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST(params, setThenGetStringCached)
{
  std::string param;
  ASSERT_FALSE( g_node->getParam( "test_set_param_setThenGetStringCached", param, true ) );

  g_node->setParam( "test_set_param_setThenGetStringCached", std::string("asdf") );

  ASSERT_TRUE( g_node->getParam( "test_set_param_setThenGetStringCached", param, true ) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST(params, setThenGetCString)
{
  g_node->setParam( "test_set_param", "asdf" );
  std::string param;
  ASSERT_TRUE( g_node->getParam( "test_set_param", param ) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST(params, setThenGetInt)
{
  g_node->setParam( "test_set_param", 42);
  int param;
  ASSERT_TRUE( g_node->getParam( "test_set_param", param ) );
  ASSERT_EQ( 42, param );
}

TEST(params, unknownParam)
{
  std::string param;
  ASSERT_FALSE( g_node->getParam( "this_param_really_should_not_exist", param ) );
}

TEST(params, deleteParam)
{
  g_node->setParam( "test_delete_param", "asdf" );
  g_node->deleteParam( "test_delete_param" );
  std::string param;
  ASSERT_FALSE( g_node->getParam( "test_delete_param", param ) );
}

TEST(params, hasParam)
{
  ASSERT_TRUE( g_node->hasParam( "string" ) );
}

TEST(params, setIntDoubleGetInt)
{
  g_node->setParam("test_set_int_as_double", 1);
  g_node->setParam("test_set_int_as_double", 3.0f);

  int i = -1;
  ASSERT_TRUE(g_node->getParam("test_set_int_as_double", i));
  ASSERT_EQ(3, i);
  double d = 0.0f;
  ASSERT_TRUE(g_node->getParam("test_set_int_as_double", d));
  ASSERT_EQ(3.0, d);
}

TEST(params, getIntAsDouble)
{
  g_node->setParam("int_param", 1);
  double d = 0.0;
  ASSERT_TRUE(g_node->getParam("int_param", d));
  ASSERT_EQ(1.0, d);
}

TEST(params, getDoubleAsInt)
{
  g_node->setParam("double_param", 2.3);
  int i = -1;
  ASSERT_TRUE(g_node->getParam("double_param", i));
  ASSERT_EQ(2, i);

  g_node->setParam("double_param", 3.8);
  i = -1;
  ASSERT_TRUE(g_node->getParam("double_param", i));
  ASSERT_EQ(4, i);
}

TEST(params, searchParam)
{
  ros::NodeHandle n("/a/b/c/d/e/f");
  std::string result;

  n.setParam("/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/s_i");

  n.setParam("/a/b/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/s_i");

  n.setParam("/a/b/c/d/e/f/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/c/d/e/f/s_i");

  ASSERT_FALSE(n.searchParam("s_j", result));
}

TEST(params, searchParamWithRemapping)
{
  ros::M_string remappings;
  remappings["s_c"] = "s_b";
  ros::NodeHandle n("/a/b/c/d/e/f", remappings);
  std::string result;

  n.setParam("/s_c", 1);
  ASSERT_FALSE(n.searchParam("s_c", result));
  n.setParam("/s_b", 1);
  ASSERT_TRUE(n.searchParam("s_c", result));
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init( argc, argv );

  g_node = new ros::Node( g_node_name );

  int ret = RUN_ALL_TESTS();


  delete g_node;

  return ret;
}
