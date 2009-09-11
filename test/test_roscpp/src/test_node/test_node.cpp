/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/node.h"
#include "std_msgs/String.h"
#include "test_ros/TestPrimitives.h"
#include "test_ros/TestArrays.h"
#include "test_ros/TestHeader.h"

class TestNode : public ros::Node
{
public:
  
  std_msgs::String probe_msg;  
  std_msgs::String string_msg;
  test_ros::TestPrimitives prim_msg;
  test_ros::TestArrays arrays_msg;
  test_ros::TestHeader header_msg;
  
  TestNode() : ros::Node("test_node")
  { 
    // required subscriptions
    subscribe("test_string_in", string_msg,  &TestNode::test_string_in_cb, 1000);
    subscribe("test_primitives_in", prim_msg,&TestNode::test_primitives_in_cb, 1000);
    subscribe("test_arrays_in", arrays_msg,  &TestNode::test_arrays_in_cb, 1000);
    subscribe("test_headers_in", header_msg, &TestNode::test_headers_in_cb, 1000);    
    
    // required publications
    advertise<std_msgs::String>("test_string_out", 1000);    
    advertise<test_ros::TestPrimitives>("test_primitives_out", 1000);    
    advertise<test_ros::TestArrays>("test_arrays_out", 1000);    
    advertise<test_ros::TestHeader>("test_header_out", 1000);    

    // subscription with no publisher
    // NOTE: probe_topic is a unpublished topic that merely exists to test
    // APIs that talk about subscriptions (e.g. publisherUpdate)
    subscribe("probe_in", probe_msg, &TestNode::probe_in_cb, 1000);    
  }
  
  ///////////////////////////////////////////////
  void probe_in_cb()
  {
    //NOOP
  }
  void test_string_in_cb()
  {
    //TODO: republish on _out
  }
  void test_primitives_in_cb()
  {
    //TODO: republish on _out
  }
  void test_arrays_in_cb()
  {
    //TODO: republish on _out
  }
  void test_headers_in_cb()
  {
    //TODO: republish on _out
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  TestNode l;
  l.spin();
  
  return 0;
}
