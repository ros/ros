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
#include "ros/rate.h"

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
  : namespace_(this_node::getNamespace())
  , callback_queue_(0)
  , collection_(0)
{
  std::string tilde_resolved_ns;
  if (!ns.empty() && ns[0] == '~')// starts with tilde
    tilde_resolved_ns = names::resolve(ns);
  else
    tilde_resolved_ns = ns;

  construct(tilde_resolved_ns, true);

  initRemappings(remappings);
}

NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns)
: collection_(0)
{
  namespace_ = parent.getNamespace();
  callback_queue_ = parent.callback_queue_;

  remappings_ = parent.remappings_;
  unresolved_remappings_ = parent.unresolved_remappings_;

  construct(ns, false);
}

NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns, const M_string& remappings)
: collection_(0)
{
  namespace_ = parent.getNamespace();
  callback_queue_ = parent.callback_queue_;

  remappings_ = parent.remappings_;
  unresolved_remappings_ = parent.unresolved_remappings_;

  construct(ns, false);

  initRemappings(remappings);
}

NodeHandle::NodeHandle(const NodeHandle& rhs)
: collection_(0)
{
  callback_queue_ = rhs.callback_queue_;
  remappings_ = rhs.remappings_;
  unresolved_remappings_ = rhs.unresolved_remappings_;

  construct(rhs.namespace_, true); 

  unresolved_namespace_ = rhs.unresolved_namespace_;
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
  unresolved_remappings_ = rhs.unresolved_remappings_;

  return *this;
}

void spinThread()
{
  ros::spin();
}

void NodeHandle::construct(const std::string& ns, bool validate_name)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("You must call ros::init() before creating the first NodeHandle");
    ROS_BREAK();
  }

  collection_ = new NodeHandleBackingCollection;
  unresolved_namespace_ = ns;
  // if callback_queue_ is nonnull, we are in a non-nullary constructor

  if (validate_name)
    namespace_ = resolveName(ns, true);
  else
    {
      namespace_ = resolveName(ns, true, no_validate());
      // FIXME validate namespace_ now
    }
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
    // ROSCPP_LOG_DEBUG("found 'local' remapping: %s", it->second.c_str());
    return it->second;
  }

  // If not in our local remappings, perhaps in the global ones
  return names::remap(resolved);
}

std::string NodeHandle::resolveName(const std::string& name, bool remap) const
{
  // ROSCPP_LOG_DEBUG("resolveName(%s, %s)", name.c_str(), remap ? "true" : "false");
  std::string error;
  if (!names::validate(name, error))
  {
    throw InvalidNameException(error);
  }

  return resolveName(name, remap, no_validate());
}

std::string NodeHandle::resolveName(const std::string& name, bool remap, no_validate) const
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
    // ROSCPP_LOG_DEBUG("Appending namespace_ (%s)", namespace_.c_str());
    final = names::append(namespace_, final);
  }

  // ROSCPP_LOG_DEBUG("resolveName, pre-clean: %s", final.c_str());
  final = names::clean(final);
  // ROSCPP_LOG_DEBUG("resolveName, post-clean: %s", final.c_str());

  if (remap)
  {
    final = remapName(final);
    // ROSCPP_LOG_DEBUG("resolveName, remapped: %s", final.c_str());
  }

  return names::resolve(final, false);
}

Publisher NodeHandle::advertise(AdvertiseOptions& ops)
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

  SubscriberCallbacksPtr callbacks(new SubscriberCallbacks(ops.connect_cb, ops.disconnect_cb, 
							   ops.tracked_object, ops.callback_queue));

  if (TopicManager::instance()->advertise(ops, callbacks))
  {
    Publisher pub(ops.topic, ops.md5sum, ops.datatype, *this, callbacks);

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

Timer NodeHandle::createTimer(Duration period, const TimerCallback& callback, 
                              bool oneshot, bool autostart) const
{
  TimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  ops.oneshot = oneshot;
  ops.autostart = autostart;
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
  if (ops.autostart)
    timer.start();
  return timer;
}

WallTimer NodeHandle::createWallTimer(WallDuration period, const WallTimerCallback& callback, 
                                      bool oneshot, bool autostart) const
{
  WallTimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  ops.oneshot = oneshot;
  ops.autostart = autostart;
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
  if (ops.autostart)
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

} // namespace ros
