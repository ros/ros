/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef ROS_INIT_H
#define ROS_INIT_H

#include "ros/forwards.h"

namespace ros
{

namespace init_options
{
/**
 * \brief Flags for ROS initialization
 */
enum InitOption
{
  /**
   * Use this option to tell ROS not to install a
   * SIGINT handler.  You should install your own SIGINT handler in this
   * case, to ensure that the node is deleted when the application exits.
   */
  NoSigintHandler = 1 << 0,
  /** @brief Flag for node() constructor
   *
   * Use this option to tell ROS to add a
   * random number to the end of your node's name, to make it unique.
   */
  AnonymousName = 1 << 1,
  /**
   * EXPERIMENTAL; do not use.
   *
   * Use this option to tell ROS not
   * to start a thread for communications handling.  If you do this, you must
   * call tcprosServerUpdate() often.
   */
  NoServerThread = 1 << 2,

  /**
   * Use this option in the ros::Node() constructor to tell ROS to not
   * broadcast rosconsole output to the /rosout topic
   */
  NoRosout = 1 << 3,
};
}
typedef init_options::InitOption InitOption;

/** @brief ROS initialization function.
 *
 * This function will parse any ROS arguments (e.g., topic name
 * remappings), and will consume them (i.e., argc and argv may be modified
 * as a result of this call).
 *
 */
void init(int &argc, char **argv);

/**
 * \brief alternate ROS initialization function.
 *
 * This version of init takes a vector of string pairs, where each one constitutes
 * a name remapping, or one of the special remappings like __name, __master, __ns, etc.
 */
void init(const VP_string& remapping_args);

/** @brief ROS initialization function.
 *
 * This function will parse any ROS arguments (e.g., topic name
 * remappings), and will consume them (i.e., argc and argv may be modified
 * as a result of this call).
 *
 * Use this version if you are using the NodeHandle API
 *
 */
void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);

/**
 * \brief alternate ROS initialization function.
 *
 * This version of init takes a vector of string pairs, where each one constitutes
 * a name remapping, or one of the special remappings like __name, __master, __ns, etc.
 *
 * Use this version if you are using the NodeHandle API
 *
 */
void init(const VP_string& remapping_args, const std::string& name, uint32_t options = 0);

}

#endif
