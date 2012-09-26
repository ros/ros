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

TEST(RemappingTest, remapping_test)
{
  EXPECT_TRUE(argc_ >= 3);

  std::string expected_base_ns = argv_[1];
  std::string expected_sub_ns  = argv_[2];

  ros::NodeHandle nh;
  //ros::NodeHandle pnh("~");

  bool use_local_remap = false;
  EXPECT_TRUE(nh.getParam("use_local_remap", use_local_remap)) 
    << "Param [~use_local_remap] must be defined\n";

  if (use_local_remap)
  {
    std::string remap_from, remap_to;
    EXPECT_TRUE(nh.getParam("remap_from", remap_from)) << "Param [~remap_from] must be defined\n";
    EXPECT_TRUE(nh.getParam("remap_to",   remap_to))   << "Param [~remap_to] must be defined\n";

    // Construct the new node by passing in a dictionary of remaps
    ros::M_string local_remappings;
    local_remappings.insert(std::make_pair(remap_from, remap_to));

    ros::NodeHandle base_nh(nh, "base_namespace", local_remappings);
    EXPECT_TRUE(base_nh.getNamespace() == expected_base_ns) 
      << "Error: \"" << base_nh.getNamespace() << "\" != \""  << expected_base_ns << "\"\n";

    ros::NodeHandle sub_nh(base_nh, "sub_namespace");
    EXPECT_TRUE(sub_nh.getNamespace() == expected_sub_ns) 
      << "Error: \"" << sub_nh.getNamespace() << "\" != \""  << expected_sub_ns << "\"\n";
  }
  else
  {
    std::cout << "***********************************************************************\n";
    ros::NodeHandle base_nh(nh, "base_namespace");
    EXPECT_EQ(base_nh.getNamespace(), expected_base_ns); 

    ros::NodeHandle sub_nh(base_nh, "sub_namespace");
    EXPECT_EQ(sub_nh.getNamespace(), expected_sub_ns); 
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "remapping_tester");
  argc_ = argc;
  argv_ = argv;
  return RUN_ALL_TESTS();
}
