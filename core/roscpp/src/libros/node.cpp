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

#include <cstdarg>
#include <sstream>
#include <cerrno>
#include <algorithm>

#include <sys/poll.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>

// TEMP to remove warnings during build while things internally still use deprecated APIs
#include "ros/macros.h"
#undef ROSCPP_DEPRECATED
#define ROSCPP_DEPRECATED
// END TEMP

#include "ros/node.h"
#include "ros/subscription.h"
#include "ros/header.h"
#include "ros/time.h"
#include "ros/connection.h"
#include "ros/publication.h"
#include "ros/subscriber_link.h"
#include "ros/service_publication.h"
#include "ros/service_client_link.h"
#include "ros/service_server_link.h"
#include "ros/transport/transport_tcp.h"
#include "ros/transport/transport_udp.h"
#include "ros/callback_queue.h"
#include "ros/xmlrpc_manager.h"
#include "ros/master.h"
#include "ros/names.h"
#include "ros/param.h"
#include "ros/network.h"
#include "ros/this_node.h"
#include "ros/init.h"
#include "ros/file_log.h"
#include "ros/connection_manager.h"
#include "ros/poll_manager.h"
#include "ros/topic_manager.h"
#include "ros/service_manager.h"

#include <boost/bind.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/lexical_cast.hpp>

#include <config.h>
#ifdef HAVE_IFADDRS_H
  #include <ifaddrs.h>
#endif

using namespace std;
using namespace XmlRpc;

namespace ros
{

Node* g_node = NULL;
bool ros::Node::s_initialized_ = false;
std::string ros::Node::s_name_;
uint32_t ros::Node::s_flags_ = 0;
ros::V_string ros::Node::s_args_;
ros::VP_string ros::Node::s_remappings_;

boost::mutex ros::Node::s_refcount_mutex_;
uint32_t ros::Node::s_refcount_ = 0;
bool ros::Node::s_created_by_handle_ = true;

Node *Node::instance()
{
  return g_node;
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

Node::Node(string _name, uint32_t options, int32_t master_retry_timeout)
{
  M_string remappings;
  VP_string::iterator it = s_remappings_.begin();
  VP_string::iterator end = s_remappings_.end();
  for (; it != end; ++it)
  {
    remappings[it->first] = it->second;
  }

  ros::init(remappings, _name, options);
  master::setRetryTimeout(ros::WallDuration(master_retry_timeout / 1000.));

  init(options, master_retry_timeout);
}

Node::Node()
{
  init(s_flags_, -1);
}

Node::~Node()
{
  ROS_DEBUG( "entering ros::Node destructor");
  shutdown(); // if we haven't disconnected already, do so

  // Set the instance pointer to NULL, to allow applications that create
  // and destroy the node multiple times in one run.
  g_node = NULL;
}

void Node::init(uint32_t options, int32_t master_retry_timeout)
{
  g_node = this;

  shutting_down_ = false;
  master_retry_timeout_ = master_retry_timeout;

  if (!s_initialized_)
  {
    ROS_ERROR("You didn't call ros::init(...) before you tried to call a node constructor!");
    ROS_BREAK();
  }

  ros::start();
}

bool Node::ok() const
{
  return ros::ok();
}

void Node::shutdown()
{
  shutting_down_ = true;
  ros::shutdown();
}

const std::string& Node::getLogFilePath()
{
  return file_log::getLogFilename();
}

std::string Node::getUnreliableServerIp() const
{
  return network::getHost();
}

const std::string& Node::getName() const
{
  return this_node::getName();
}

const std::string& Node::getNamespace() const
{
  return this_node::getNamespace();
}

void Node::requestShutdown()
{
  ros::requestShutdown();
}

std::string Node::cleanName(const std::string& name)
{
  return names::clean(name);
}

std::string Node::resolveName(const std::string& name, bool remap)
{
  return names::resolve(name, remap);
}

void Node::getAdvertisedTopics(V_string& topics)
{
  TopicManager::instance()->getAdvertisedTopics(topics);
}

void Node::getSubscribedTopics(V_string& topics)
{
  TopicManager::instance()->getSubscribedTopics(topics);
}

// this function has the subscription code that doesn't need to be templated.
bool Node::subscribe(const SubscribeOptions& ops, Message* m, AbstractFunctor *cb)
{
  SubscribeOptions copy = ops;
  copy.topic = resolveName(ops.topic);
  return TopicManager::instance()->subscribe(copy, m, cb);
}

void Node::setMasterRetryTimeout(int32_t milliseconds)
{
  master_retry_timeout_ = milliseconds;
}

// this function handles all the stuff that doesn't need to be templated
bool Node::advertise(const AdvertiseOptions& ops,
                      bool allow_multiple)
{
  AdvertiseOptions copy = ops;
  copy.topic = resolveName(ops.topic);
  return TopicManager::instance()->advertise(copy, allow_multiple);
}

bool Node::unadvertise(const std::string &topic, const SubscriberCallbacksPtr& callbacks)
{
  return TopicManager::instance()->unadvertise(resolveName(topic), callbacks);
}

bool Node::advertiseService(const AdvertiseServiceOptions& ops, int32_t thread_pool_size)
{
  AdvertiseServiceOptions copy = ops;
  copy.service = resolveName(ops.service);
  return ServiceManager::instance()->advertiseService(copy, thread_pool_size);
}

bool Node::unadvertiseService(const string &_serv_name)
{
  return ServiceManager::instance()->unadvertiseService(resolveName(_serv_name));
}

bool Node::advertise(const string &_topic,
                     const Message& msgref, size_t max_queue)
{
  AdvertiseOptions ops(resolveName(_topic),
                       max_queue,
                       msgref.__getMD5Sum(),
                       msgref.__getDataType(),
                       msgref.__getMessageDefinition());
  return advertise(ops);
}

void Node::spin()
{
  while (ros::ok())
  {
    WallDuration(0.01).sleep();
  }
}

bool Node::getPublishedTopics(VP_string* topics)
{
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = ""; //TODO: Fix this

  if (!master::execute("getPublishedTopics", args, result, payload, true))
  {
    return false;
  }

  topics->clear();
  for (int i = 0; i < payload.size(); i++)
  {
    topics->push_back(pair<string, string>(string(payload[i][0]), string(payload[i][1])));
  }

  return true;
}

bool Node::getNodes(V_string& nodes)
{
  return master::getNodes(nodes);
}

bool Node::splitURI(const string &uri, string &host, int &_port)
{
  uint32_t port = 0;
  bool b = network::splitURI(uri, host, port);
  _port = port;

  return b;
}

bool Node::unsubscribe(const std::string &_topic)
{
  return TopicManager::instance()->unsubscribe(resolveName(_topic));
}

bool Node::unsubscribe(const Message& _msg)
{
  return TopicManager::instance()->unsubscribe(_msg);
}

bool Node::unsubscribe(const std::string &_topic, const SubscriptionMessageHelperPtr& helper)
{
  return TopicManager::instance()->unsubscribe(resolveName(_topic), helper);
}

bool Node::unsubscribe(const std::string &topic, AbstractFunctor *afp)
{
  return TopicManager::instance()->unsubscribe(resolveName(topic), afp);
}

size_t Node::numSubscribers(const std::string &_topic)
{
  return TopicManager::instance()->getNumSubscribers(resolveName(_topic));
}

bool Node::checkMaster()
{
  return master::check();
}

void Node::publish(const std::string& topic, const Message& msg)
{
  TopicManager::instance()->publish(resolveName(topic), msg);
}

void Node::setParam(const std::string &key, const XmlRpc::XmlRpcValue &v)
{
  return param::set(resolveName(key), v);
}

void Node::setParam(const std::string &key, const std::string &s)
{
  return param::set(resolveName(key), s);
}

void Node::setParam(const std::string &key, const char* s)
{
  return param::set(resolveName(key), s);
}

void Node::setParam(const std::string &key, double d)
{
  return param::set(resolveName(key), d);
}

void Node::setParam(const std::string &key, int i)
{
  return param::set(resolveName(key), i);
}

void Node::setParam(const std::string &key, bool b)
{
  return param::set(resolveName(key), b);
}

bool Node::hasParam(const std::string &key)
{
  return param::has(resolveName(key));
}

bool Node::deleteParam(const std::string &key)
{
  return param::del(resolveName(key));
}

bool Node::getParam(const std::string &key, XmlRpc::XmlRpcValue &v, bool use_cache)
{
  return param::get(resolveName(key), v, use_cache);
}

bool Node::getParam(const std::string &key, std::string &s, bool use_cache)
{
  return param::get(resolveName(key), s, use_cache);
}

bool Node::getParam(const std::string &key, double &d, bool use_cache)
{
  return param::get(resolveName(key), d, use_cache);
}

bool Node::getParam(const std::string &key, int &i, bool use_cache)
{
  return param::get(resolveName(key), i, use_cache);
}

bool Node::getParam(const std::string &key, bool &b, bool use_cache)
{
  return param::get(resolveName(key), b, use_cache);
}

size_t Node::numSubscriptions()
{
  return TopicManager::instance()->getNumSubscriptions();
}

std::string Node::getIP()
{
  return network::getHost();
}

} // namespace ros
