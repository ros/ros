/*
 * Copyright (C) 2009, Willow Garage, Inc.
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

#ifndef ROSCPP_THIS_NODE_H
#define ROSCPP_THIS_NODE_H

#include "common.h"
#include "forwards.h"

namespace ros
{

/**
 * \brief Contains functions which provide information about this process' ROS node
 */
namespace this_node
{

/**
 * \brief Returns the name of the current node.
 */
ROSCPP_DECL const std::string& getName();
/**
 * \brief Returns the namespace of the current node.
 */
ROSCPP_DECL const std::string& getNamespace();

/** @brief Get the list of topics advertised by this node
 *
 * @param[out] topics The advertised topics
 */
ROSCPP_DECL void getAdvertisedTopics(V_string& topics);

/** @brief Get the list of topics subscribed to by this node
 *
 * @param[out] The subscribed topics
 */
ROSCPP_DECL void getSubscribedTopics(V_string& topics);

} // namespace this_node

} // namespace ros

#endif // ROSCPP_THIS_NODE_H


