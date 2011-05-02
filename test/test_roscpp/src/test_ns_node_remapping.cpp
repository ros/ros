/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <ros/ros.h>
#include <gtest/gtest.h>

static int argc_;
static char** argv_;

#define PRINT(cmd) printf(#cmd"\n"); cmd; printf("\n");

TEST(NamespaceRemappingTest, unqualified_remaps)
{
  ros::M_string local_remappings;
  local_remappings.insert(std::make_pair("a", "Ra"));
  local_remappings.insert(std::make_pair("b", "Rb"));
  local_remappings.insert(std::make_pair("c", "Rc"));

  PRINT(ros::NodeHandle base("a", local_remappings));
  PRINT(ros::NodeHandle a1(base, "a"));
  PRINT(ros::NodeHandle a2(base, "a", ros::M_string()));
  PRINT(ros::NodeHandle  b(base, "b"));
  PRINT(ros::NodeHandle  c(base, "c", ros::M_string()));  // Same as b, but different constructor

  EXPECT_STREQ(base.getNamespace().c_str(), "/a");
  EXPECT_STREQ(a1.getNamespace().c_str(), "/a/Ra");
  EXPECT_STREQ(a2.getNamespace().c_str(), "/a/Ra");
  EXPECT_STREQ( b.getNamespace().c_str(), "/a/Rb");
  EXPECT_STREQ( c.getNamespace().c_str(), "/a/Rc");
}

TEST(NamespaceRemappingTest, qualified_remaps)
{
  ros::M_string local_remappings;
  local_remappings.insert(std::make_pair("/a", "/Ra"));

  PRINT(ros::NodeHandle a("a", local_remappings));  // local_remappings don't apply to this nodehandle's name
  PRINT(ros::NodeHandle sub_a(a, "a"));             // remapping were fully qualified, so don't apply to /a/a

  EXPECT_STREQ(    a.getNamespace().c_str(), "/a");
  EXPECT_STREQ(sub_a.getNamespace().c_str(), "/a/a");
}

TEST(NamespaceRemappingTest, unqualified_root_remaps)
{
  ros::M_string local_remappings;
  local_remappings.insert(std::make_pair("a", "Ra"));
  local_remappings.insert(std::make_pair("b", "Rb"));

  ros::NodeHandle base("", local_remappings);
  ros::NodeHandle a(base, "a");
  ros::NodeHandle b(base, "b", ros::M_string());

  EXPECT_STREQ(a.getNamespace().c_str(), "/Ra");
  EXPECT_STREQ(b.getNamespace().c_str(), "/Rb");
}

TEST(NamespaceRemappingTest, tilde_namespaces)
{
  ros::M_string local_remappings;
  local_remappings.insert(std::make_pair("a", "Ra"));
  local_remappings.insert(std::make_pair("b", "Rb"));

  ros::NodeHandle base("~", local_remappings);
  ros::NodeHandle a(base, "a");
  ros::NodeHandle b(base, "b", ros::M_string());

  EXPECT_STREQ(base.getNamespace().c_str(), ros::this_node::getName().c_str());
  EXPECT_STREQ(a.getNamespace().c_str(), (ros::this_node::getName() + "/Ra").c_str());
  EXPECT_STREQ(b.getNamespace().c_str(), (ros::this_node::getName() + "/Rb").c_str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "remapping_tester");
  argc_ = argc;
  argv_ = argv;
  return RUN_ALL_TESTS();
}
