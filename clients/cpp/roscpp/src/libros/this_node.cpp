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

#include <cstdio>
#include "ros/this_node.h"
#include "ros/names.h"
#include "ros/topic_manager.h"
#include "ros/init.h"

#ifdef _MSC_VER
  #ifdef snprintf
    #undef snprintf
  #endif
  #define snprintf _snprintf_s
#endif

namespace ros
{

namespace names
{
void init(const M_string& remappings);
}

namespace this_node
{

std::string g_name = "empty";
std::string g_namespace;

const std::string& getName()
{
  return g_name;
}

const std::string& getNamespace()
{
  return g_namespace;
}

void getAdvertisedTopics(V_string& topics)
{
  TopicManager::instance()->getAdvertisedTopics(topics);
}

void getSubscribedTopics(V_string& topics)
{
  TopicManager::instance()->getSubscribedTopics(topics);
}

void init(const std::string& name, const M_string& remappings, uint32_t options)
{
  char *ns_env = NULL;
#ifdef _MSC_VER
  _dupenv_s(&ns_env, NULL, "ROS_NAMESPACE");
#else
  ns_env = getenv("ROS_NAMESPACE");
#endif

  if (ns_env)
  {
    g_namespace = ns_env;
#ifdef _MSC_VER
    free(ns_env);
#endif
  }

  g_name = name;

  bool disable_anon = false;
  M_string::const_iterator it = remappings.find("__name");
  if (it != remappings.end())
  {
    g_name = it->second;
    disable_anon = true;
  }

  it = remappings.find("__ns");
  if (it != remappings.end())
  {
    g_namespace = it->second;
  }

  if (g_namespace.empty())
  {
    g_namespace = "/";
  }

  g_namespace = (g_namespace == "/")
    ? std::string("/") 
    : ("/" + g_namespace)
    ;


  std::string error;
  if (!names::validate(g_namespace, error))
  {
    std::stringstream ss;
    ss << "Namespace [" << g_namespace << "] is invalid: " << error;
    throw InvalidNameException(ss.str());
  }

  // names must be initialized here, because it requires the namespace to already be known so that it can properly resolve names.
  // It must be done before we resolve g_name, because otherwise the name will not get remapped.
  names::init(remappings);

  if (g_name.find("/") != std::string::npos)
  {
    throw InvalidNodeNameException(g_name, "node names cannot contain /");
  }
  if (g_name.find("~") != std::string::npos)
  {
    throw InvalidNodeNameException(g_name, "node names cannot contain ~");
  }

  g_name = names::resolve(g_namespace, g_name);

  if (options & init_options::AnonymousName && !disable_anon)
  {
    char buf[200];
    snprintf(buf, sizeof(buf), "_%llu", (unsigned long long)WallTime::now().toNSec());
    g_name += buf;
  }

  ros::console::setFixedFilterToken("node", g_name);
}

} // namespace this_node

} // namespace ros
