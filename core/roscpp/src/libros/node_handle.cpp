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
#include "ros/node.h"
#include "ros/service.h"
#include "ros/callback_queue.h"
#include "ros/timer_manager.h"

#include "ros/time.h"

#include "XmlRpc.h"

#include <boost/thread.hpp>

namespace ros
{

CallbackQueue g_global_queue;

void spin()
{
  SingleThreadedSpinner s;
  spin(s);
}

void spin(Spinner& s)
{
  s.spin();
}

void spinOnce()
{
  g_global_queue.callAvailable(ros::WallDuration());
}

CallbackQueue* getGlobalCallbackQueue()
{
  return &g_global_queue;
}

bool ok()
{
  return ros::Node::instance() && ros::Node::instance()->ok();
}

void shutdown()
{
  if (ros::Node::instance())
  {
    ros::Node::instance()->shutdown();
  }
}

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
, node_(0)
, callback_queue_(0)
{
  construct();

  initRemappings(remappings);
}

NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns)
{
  namespace_ = parent.getNamespace() + "/" + ns;
  callback_queue_ = parent.callback_queue_;

  construct();
}

NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns, const M_string& remappings)
{
  namespace_ = parent.getNamespace() + "/" + ns;
  callback_queue_ = parent.callback_queue_;

  construct();

  initRemappings(remappings);
}

NodeHandle::NodeHandle(const NodeHandle& rhs)
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

void spinThread()
{
  ros::spin();
}

void NodeHandle::construct()
{
  collection_ = new NodeHandleBackingCollection;

  boost::mutex::scoped_lock lock(ros::Node::s_refcount_mutex_);

  if (ros::Node::s_refcount_ == 0)
  {
    node_ = ros::Node::instance();
    if (node_)
    {
      ros::Node::s_created_by_handle_ = false;

      ROS_WARN("NodeHandle API is being used inside an application started with the old Node API. "
               "Automatically starting a ros::spin() thread.  Please switch this application to use the NodeHandle API.");
      // start a thread that automatically calls spin() for us
      boost::thread t(spinThread);
      t.detach();
    }
    else
    {
      ros::Node::s_created_by_handle_ = true;
      node_ = new ros::Node();
    }
  }
  else
  {
    node_ = ros::Node::instance();
  }

  ++ros::Node::s_refcount_;
}

void NodeHandle::destruct()
{
  delete collection_;

  boost::mutex::scoped_lock lock(ros::Node::s_refcount_mutex_);

  --ros::Node::s_refcount_;

  if (ros::Node::s_refcount_ == 0)
  {
    if (ros::Node::s_created_by_handle_)
    {
      node_->shutdown();
      delete node_;
    }
  }
}

void NodeHandle::initRemappings(const M_string& remappings)
{
  {
    M_string::const_iterator it = remappings.begin();
    M_string::const_iterator end = remappings.end();
    for (; it != end; ++it)
    {
      std::string from = mapName(it->first);
      std::string to = mapName(it->second);

      remappings_.insert(std::make_pair(from, to));
    }
  }
}

void NodeHandle::setCallbackQueue(CallbackQueueInterface* queue)
{
  callback_queue_ = queue;
}

std::string NodeHandle::mapName(const std::string& name)
{
  if (name.empty())
  {
    ROS_ASSERT_MSG(false, "Tried to map an empty name");
    return "";
  }

  std::string final = name;
  if (name[0] == '~')
  {
    std::string mapped_namespace = node_->mapName("~" + namespace_);
    final = mapped_namespace + "/" + name.substr(1);
  }
  else if (name[0] == '/')
  {
    final = name;
  }
  else if (!namespace_.empty())
  {
    final = namespace_ + "/" + name;
  }

  final = node_->mapName(final);

  M_string::iterator it = remappings_.find(final);
  if (it != remappings_.end())
  {
    final = it->second;
  }

  return final;
}

Publisher NodeHandle::advertise(AdvertiseOptions& ops)
{
  SubscriberCallbacksPtr callbacks(new SubscriberCallbacks(ops.connect_cb, ops.disconnect_cb));

  ops.topic = mapName(ops.topic);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = &g_global_queue;
    }
  }

  if (node_->advertise(ops, true))
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
  ops.topic = mapName(ops.topic);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = &g_global_queue;
    }
  }

  if (node_->subscribe(ops, 0, 0))
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
  ops.service = mapName(ops.service);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = &g_global_queue;
    }
  }

  if (node_->advertiseService(ops, 0))
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
  ops.service = mapName(ops.service);
  ServiceClient client = service::createClientImpl(ops.service, ops.persistent, ops.header, ops.md5sum);

  if (client)
  {
    boost::mutex::scoped_lock lock(collection_->mutex_);
    collection_->srv_cs_.push_back(client.impl_);
  }

  return client;
}

Timer NodeHandle::createTimer(Duration period, const TimerCallback& callback)
{
  TimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  return createTimer(ops);
}

Timer NodeHandle::createTimer(TimerOptions& ops)
{
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = &g_global_queue;
    }
  }

  Timer timer(ops);
  timer.start();
  return timer;
}

WallTimer NodeHandle::createWallTimer(WallDuration period, const WallTimerCallback& callback)
{
  WallTimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  return createWallTimer(ops);
}

WallTimer NodeHandle::createWallTimer(WallTimerOptions& ops)
{
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = &g_global_queue;
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
}

void NodeHandle::setParam(const std::string &key, const XmlRpc::XmlRpcValue &v)
{
  return node_->setParam(mapName(key), v);
}

void NodeHandle::setParam(const std::string &key, const std::string &s)
{
  return node_->setParam(mapName(key), s);
}

void NodeHandle::setParam(const std::string &key, const char* s)
{
  return node_->setParam(mapName(key), s);
}

void NodeHandle::setParam(const std::string &key, double d)
{
  return node_->setParam(mapName(key), d);
}

void NodeHandle::setParam(const std::string &key, int i)
{
  return node_->setParam(mapName(key), i);
}

void NodeHandle::setParam(const std::string &key, bool b)
{
  return node_->setParam(mapName(key), b);
}

bool NodeHandle::hasParam(const std::string &key)
{
  return node_->hasParam(mapName(key));
}

bool NodeHandle::deleteParam(const std::string &key)
{
  return node_->deleteParam(mapName(key));
}

bool NodeHandle::getParam(const std::string &key, XmlRpc::XmlRpcValue &v, bool use_cache)
{
  return node_->getParam(mapName(key), v, use_cache);
}

bool NodeHandle::getParam(const std::string &key, std::string &s, bool use_cache)
{
  return node_->getParam(mapName(key), s, use_cache);
}

bool NodeHandle::getParam(const std::string &key, double &d, bool use_cache)
{
  return node_->getParam(mapName(key), d, use_cache);
}

bool NodeHandle::getParam(const std::string &key, int &i, bool use_cache)
{
  return node_->getParam(mapName(key), i, use_cache);
}

bool NodeHandle::getParam(const std::string &key, bool &b, bool use_cache)
{
  return node_->getParam(mapName(key), b, use_cache);
}

bool NodeHandle::ok() const
{
  return node_->ok();
}

void NodeHandle::setMasterRetryTimeout(int32_t milliseconds)
{
  node_->setMasterRetryTimeout(milliseconds);
}

void NodeHandle::getAdvertisedTopics(V_string& topics) const
{
  node_->getAdvertisedTopics(topics);
}

void NodeHandle::getSubscribedTopics(V_string& topics) const
{
  node_->getSubscribedTopics(topics);
}

const std::string& NodeHandle::getName() const
{
  return node_->getName();
}

const V_string& NodeHandle::getParsedArgs()
{
  return ros::Node::getParsedArgs();
}

const std::string& NodeHandle::getMasterHost() const
{
  return node_->getMasterHost();
}
int NodeHandle::getMasterPort() const
{
  return node_->getMasterPort();
}

const std::string& NodeHandle::getXMLRPCURI() const
{
  return node_->getXMLRPCURI();
}

bool NodeHandle::checkMaster()
{
  return node_->checkMaster();
}

bool NodeHandle::getPublishedTopics(VP_string& topics)
{
  return node_->getPublishedTopics(&topics);
}

bool NodeHandle::getNodes(V_string& nodes)
{
  return node_->getNodes(nodes);
}

} // namespace ros
