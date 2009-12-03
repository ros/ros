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

#include "ros/node_handle.h"
#include "ros/this_node.h"
#include "ros/service.h"
#include "ros/callback_queue.h"
#include "ros/timer_manager.h"

#include "ros/time.h"

#include "ros/xmlrpc_manager.h"
#include "ros/topic_manager.h"
#include "ros/service_manager.h"
#include "ros/master.h"
#include "ros/param.h"
#include "ros/names.h"
#include "ros/init.h"
#include "ros/this_node.h"
#include "XmlRpc.h"

#include <boost/thread.hpp>

namespace ros
{

boost::mutex g_nh_refcount_mutex;
int32_t g_nh_refcount = 0;
bool g_node_started_by_nh = false;

class NodeHandleBackingCollection
{
public:
  typedef std::vector<Publisher::ImplWPtr> V_PubImpl;
  typedef std::vector<ServiceServer::ImplWPtr> V_SrvImpl;
  typedef std::vector<Subscriber::ImplWPtr> V_SubImpl;
  typedef std::vector<ServiceClient::ImplWPtr> V_SrvCImpl;
  V_PubImpl pubs_;
  V_SrvImpl srvs_;
  V_SubImpl subs_;
  V_SrvCImpl srv_cs_;

  boost::mutex mutex_;
};

NodeHandle::NodeHandle(const std::string& ns, const M_string& remappings)
: namespace_(ns)
, callback_queue_(0)
, collection_(0)
{
  construct();

  initRemappings(remappings);
}

NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns)
: collection_(0)
{
  namespace_ = parent.getNamespace() + "/" + ns;
  callback_queue_ = parent.callback_queue_;

  construct();
}

NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns, const M_string& remappings)
: collection_(0)
{
  namespace_ = parent.getNamespace() + "/" + ns;
  callback_queue_ = parent.callback_queue_;

  construct();

  initRemappings(remappings);
}

NodeHandle::NodeHandle(const NodeHandle& rhs)
: collection_(0)
{
  namespace_ = rhs.namespace_;
  callback_queue_ = rhs.callback_queue_;
  remappings_ = rhs.remappings_;

  construct();
}

NodeHandle::~NodeHandle()
{
  destruct();
}

NodeHandle& NodeHandle::operator=(const NodeHandle& rhs)
{
  ROS_ASSERT(collection_);
  namespace_ = rhs.namespace_;
  callback_queue_ = rhs.callback_queue_;
  remappings_ = rhs.remappings_;

  return *this;
}

void spinThread()
{
  ros::spin();
}

void NodeHandle::construct()
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("You must call ros::init() before creating the first NodeHandle");
    ROS_BREAK();
  }

  collection_ = new NodeHandleBackingCollection;
  unresolved_namespace_ = namespace_;
  namespace_ = names::resolve(namespace_);
  ok_ = true;

  boost::mutex::scoped_lock lock(g_nh_refcount_mutex);

  if (g_nh_refcount == 0 && !ros::isStarted())
  {
    g_node_started_by_nh = true;
    ros::start();
  }

  ++g_nh_refcount;
}

void NodeHandle::destruct()
{
  delete collection_;

  boost::mutex::scoped_lock lock(g_nh_refcount_mutex);

  --g_nh_refcount;

  if (g_nh_refcount == 0 && g_node_started_by_nh)
  {
    ros::shutdown();
  }
}

void NodeHandle::initRemappings(const M_string& remappings)
{
  {
    M_string::const_iterator it = remappings.begin();
    M_string::const_iterator end = remappings.end();
    for (; it != end; ++it)
    {
      const std::string& from = it->first;
      const std::string& to = it->second;

      remappings_.insert(std::make_pair(resolveName(from, false), resolveName(to, false)));
      unresolved_remappings_.insert(std::make_pair(from, to));
    }
  }
}

void NodeHandle::setCallbackQueue(CallbackQueueInterface* queue)
{
  callback_queue_ = queue;
}

std::string NodeHandle::remapName(const std::string& name) const
{
  std::string resolved = resolveName(name, false);

  // First search any remappings that were passed in specifically for this NodeHandle
  M_string::const_iterator it = remappings_.find(resolved);
  if (it != remappings_.end())
  {
    return it->second;
  }

  // If not in our local remappings, perhaps in the global ones
  return names::remap(resolved);
}

std::string NodeHandle::resolveName(const std::string& name, bool remap) const
{
  if (name.empty())
  {
    return namespace_;
  }

  std::string final = name;

  if (final[0] == '~')
  {
    std::stringstream ss;
    ss << "Using ~ names with NodeHandle methods is not allowed.  If you want to use private names with the NodeHandle ";
    ss << "interface, construct a NodeHandle using a private name as its namespace.  e.g. ";
    ss << "ros::NodeHandle nh(\"~\");  ";
    ss << "nh.getParam(\"my_private_name\");";
    ss << " (name = [" << name << "])";
    throw InvalidNameException(ss.str());
  }
  else if (final[0] == '/')
  {
    // do nothing
  }
  else if (!namespace_.empty())
  {
    final = names::append(namespace_, final);
  }

  final = names::clean(final);

  if (remap)
  {
    final = remapName(final);
  }

  return names::resolve(final, false);
}

Publisher NodeHandle::advertise(AdvertiseOptions& ops)
{
  SubscriberCallbacksPtr callbacks(new SubscriberCallbacks(ops.connect_cb, ops.disconnect_cb, ops.tracked_object));

  ops.topic = resolveName(ops.topic);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  if (TopicManager::instance()->advertise(ops, callbacks))
  {
    Publisher pub(ops.topic, *this, callbacks);

    {
      boost::mutex::scoped_lock lock(collection_->mutex_);
      collection_->pubs_.push_back(pub.impl_);
    }

    return pub;
  }

  return Publisher();
}

Subscriber NodeHandle::subscribe(SubscribeOptions& ops)
{
  ops.topic = resolveName(ops.topic);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  if (TopicManager::instance()->subscribe(ops))
  {
    Subscriber sub(ops.topic, *this, ops.helper);

    {
      boost::mutex::scoped_lock lock(collection_->mutex_);
      collection_->subs_.push_back(sub.impl_);
    }

    return sub;
  }

  return Subscriber();
}

ServiceServer NodeHandle::advertiseService(AdvertiseServiceOptions& ops)
{
  ops.service = resolveName(ops.service);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  if (ServiceManager::instance()->advertiseService(ops))
  {
    ServiceServer srv(ops.service, *this);

    {
      boost::mutex::scoped_lock lock(collection_->mutex_);
      collection_->srvs_.push_back(srv.impl_);
    }

    return srv;
  }

  return ServiceServer();
}

ServiceClient NodeHandle::serviceClient(ServiceClientOptions& ops)
{
  ops.service = resolveName(ops.service);
  ServiceClient client(ops.service, ops.persistent, ops.header, ops.md5sum);

  if (client)
  {
    boost::mutex::scoped_lock lock(collection_->mutex_);
    collection_->srv_cs_.push_back(client.impl_);
  }

  return client;
}

Timer NodeHandle::createTimer(Duration period, const TimerCallback& callback, bool oneshot) const
{
  TimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  ops.oneshot = oneshot;
  return createTimer(ops);
}

Timer NodeHandle::createTimer(TimerOptions& ops) const
{
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  Timer timer(ops);
  timer.start();
  return timer;
}

WallTimer NodeHandle::createWallTimer(WallDuration period, const WallTimerCallback& callback, bool oneshot) const
{
  WallTimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  ops.oneshot = oneshot;
  return createWallTimer(ops);
}

WallTimer NodeHandle::createWallTimer(WallTimerOptions& ops) const
{
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  WallTimer timer(ops);
  timer.start();
  return timer;
}

void NodeHandle::shutdown()
{
  {
    NodeHandleBackingCollection::V_SubImpl::iterator it = collection_->subs_.begin();
    NodeHandleBackingCollection::V_SubImpl::iterator end = collection_->subs_.end();
    for (; it != end; ++it)
    {
      Subscriber::ImplPtr impl = it->lock();

      if (impl)
      {
        impl->unsubscribe();
      }
    }
  }

  {
    NodeHandleBackingCollection::V_PubImpl::iterator it = collection_->pubs_.begin();
    NodeHandleBackingCollection::V_PubImpl::iterator end = collection_->pubs_.end();
    for (; it != end; ++it)
    {
      Publisher::ImplPtr impl = it->lock();

      if (impl)
      {
        impl->unadvertise();
      }
    }
  }

  {
    NodeHandleBackingCollection::V_SrvImpl::iterator it = collection_->srvs_.begin();
    NodeHandleBackingCollection::V_SrvImpl::iterator end = collection_->srvs_.end();
    for (; it != end; ++it)
    {
      ServiceServer::ImplPtr impl = it->lock();

      if (impl)
      {
        impl->unadvertise();
      }
    }
  }

  {
    NodeHandleBackingCollection::V_SrvCImpl::iterator it = collection_->srv_cs_.begin();
    NodeHandleBackingCollection::V_SrvCImpl::iterator end = collection_->srv_cs_.end();
    for (; it != end; ++it)
    {
      ServiceClient::ImplPtr impl = it->lock();

      if (impl)
      {
        impl->shutdown();
      }
    }
  }

  ok_ = false;
}

void NodeHandle::setParam(const std::string &key, const XmlRpc::XmlRpcValue &v) const
{
  return param::set(resolveName(key), v);
}

void NodeHandle::setParam(const std::string &key, const std::string &s) const
{
  return param::set(resolveName(key), s);
}

void NodeHandle::setParam(const std::string &key, const char* s) const
{
  return param::set(resolveName(key), s);
}

void NodeHandle::setParam(const std::string &key, double d) const
{
  return param::set(resolveName(key), d);
}

void NodeHandle::setParam(const std::string &key, int i) const
{
  return param::set(resolveName(key), i);
}

void NodeHandle::setParam(const std::string &key, bool b) const
{
  return param::set(resolveName(key), b);
}

bool NodeHandle::hasParam(const std::string &key) const
{
  return param::has(resolveName(key));
}

bool NodeHandle::deleteParam(const std::string &key) const
{
  return param::del(resolveName(key));
}

bool NodeHandle::getParam(const std::string &key, XmlRpc::XmlRpcValue &v) const
{
  return param::get(resolveName(key), v);
}

bool NodeHandle::getParam(const std::string &key, std::string &s) const
{
  return param::get(resolveName(key), s);
}

bool NodeHandle::getParam(const std::string &key, double &d) const
{
  return param::get(resolveName(key), d);
}

bool NodeHandle::getParam(const std::string &key, int &i) const
{
  return param::get(resolveName(key), i);
}

bool NodeHandle::getParam(const std::string &key, bool &b) const
{
  return param::get(resolveName(key), b);
}

bool NodeHandle::getParamCached(const std::string &key, XmlRpc::XmlRpcValue &v) const
{
  return param::getCached(resolveName(key), v);
}

bool NodeHandle::getParamCached(const std::string &key, std::string &s) const
{
  return param::getCached(resolveName(key), s);
}

bool NodeHandle::getParamCached(const std::string &key, double &d) const
{
  return param::getCached(resolveName(key), d);
}

bool NodeHandle::getParamCached(const std::string &key, int &i) const
{
  return param::getCached(resolveName(key), i);
}

bool NodeHandle::getParamCached(const std::string &key, bool &b) const
{
  return param::getCached(resolveName(key), b);
}

////////////////////////////////////////////////////////////////////////////
// Deprecated caching versions of getParam()
bool NodeHandle::getParam(const std::string &key, XmlRpc::XmlRpcValue &v, bool use_cache) const
{
  return param::getCached(resolveName(key), v);
}

bool NodeHandle::getParam(const std::string &key, std::string &s, bool use_cache) const
{
  return param::getCached(resolveName(key), s);
}

bool NodeHandle::getParam(const std::string &key, double &d, bool use_cache) const
{
  return param::getCached(resolveName(key), d);
}

bool NodeHandle::getParam(const std::string &key, int &i, bool use_cache) const
{
  return param::getCached(resolveName(key), i);
}

bool NodeHandle::getParam(const std::string &key, bool &b, bool use_cache) const
{
  return param::getCached(resolveName(key), b);
}
////////////////////////////////////////////////////////////////////////////

bool NodeHandle::searchParam(const std::string &key, std::string& result_out) const
{
  // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
  // resolved one.

  std::string remapped = key;
  M_string::const_iterator it = unresolved_remappings_.find(key);
  // First try our local remappings
  if (it != unresolved_remappings_.end())
  {
    remapped = it->second;
  }

  return param::search(resolveName(""), remapped, result_out);
}

bool NodeHandle::ok() const
{
  return ros::ok() && ok_;
}

void NodeHandle::setMasterRetryTimeout(int32_t milliseconds)
{
  master::setRetryTimeout(WallDuration(milliseconds / 1000.));
}

void NodeHandle::getAdvertisedTopics(V_string& topics) const
{
  this_node::getAdvertisedTopics(topics);
}

void NodeHandle::getSubscribedTopics(V_string& topics) const
{
  this_node::getSubscribedTopics(topics);
}

const std::string& NodeHandle::getName() const
{
  return this_node::getName();
}

const std::string& NodeHandle::getMasterHost() const
{
  return ros::master::getHost();
}
int NodeHandle::getMasterPort() const
{
  return ros::master::getPort();
}

bool NodeHandle::checkMaster() const
{
  return ros::master::check();
}

bool NodeHandle::getPublishedTopics(VP_string& topics) const
{
  master::V_TopicInfo info;
  if (!master::getTopics(info))
  {
    return false;
  }

  master::V_TopicInfo::iterator it = info.begin();
  master::V_TopicInfo::iterator end = info.end();
  for (; it != end; ++it)
  {
    topics.push_back(std::make_pair(it->name, it->datatype));
  }

  return true;
}

bool NodeHandle::getNodes(V_string& nodes) const
{
  return master::getNodes(nodes);
}

} // namespace ros
