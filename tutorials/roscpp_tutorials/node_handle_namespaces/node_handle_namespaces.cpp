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

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial shows the use of NodeHandle namespaces.
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_handle_namespaces");

  /**
   * Each NodeHandle can be assigned a namespace the same way every node can be assigned
   * a namespace (this is usually done through the environment variable ROS_NAMESPACE).
   *
   * A namespace per NodeHandle allows you to separate different pieces of your application
   * into different namespaces.
   */
  ros::NodeHandle node1("level1");
  std::string s = node1.resolveName("a");
  /// This prints /level1/a
  ROS_INFO_STREAM(s);

  /**
   * If another NodeHandle is provided as the first argument to the constructor, it uses
   * that NodeHandle's namespace as a parent namespace.  Therefore, this NodeHandle will be
   * in the /level1/level2 namespace.
   */
  ros::NodeHandle node2(node1, "level2");
  s = node2.resolveName("a");
  /// This prints /level1/level2/a
  ROS_INFO_STREAM(s);

  /**
   * Accessing private names (inside the nodes' name) is possible by creating a NodeHandle whose name starts with a
   * tilde (~).
   */
  ros::NodeHandle node3("~");
  s = node3.resolveName("a");
  /// This prints /node_handle_namespaces/a
  ROS_INFO_STREAM(s);

  return 0;
}
