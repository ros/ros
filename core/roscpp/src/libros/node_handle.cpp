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
      const std::string& from = it->first;
      const std::string& to = it->second;

      remappings_.insert(std::make_pair(from, to));
    }
  }
}

void NodeHandle::setCallbackQueue(CallbackQueueInterface* queue)
{
  callback_queue_ = queue;
}

std::string NodeHandle::remapName(const std::string& name) const
{
  // First search any remappings that were passed in specifically for this NodeHandle
  M_string::const_iterator it = remappings_.find(name);
  if (it != remappings_.end())
  {
    return it->second;
  }
  else
  {
    // Now search the Node remappings
    it = node_->name_map_.find(name);
    if (it != node_->name_map_.end())
    {
      return it->second;
    }
  }

  return name;
}

std::string NodeHandle::resolveName(const std::string& name, bool remap) const
{
  if (name.empty())
  {
    return node_->resolveName(namespace_, false);
  }

  std::string final = remap ? remapName(name) : name;

  if (final[0] == '~')
  {
    std::string mapped_namespace = node_->resolveName("~" + namespace_, false);
    final = mapped_namespace + "/" + final.substr(1);
  }
  else if (final[0] == '/')
  {
    // do nothing
  }
  else if (!namespace_.empty())
  {
    final = namespace_ + "/" + final;
  }

  return node_->resolveName(final, false);
}

Publisher NodeHandle::advertise(AdvertiseOptions& ops)
{
  SubscriberCallbacksPtr callbacks(new SubscriberCallbacks(ops.connect_cb, ops.disconnect_cb));

  ops.topic = resolveName(ops.topic);
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
  ops.topic = resolveName(ops.topic);
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
  ops.service = resolveName(ops.service);
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
  ops.service = resolveName(ops.service);
  ServiceClient client = service::createClientImpl(ops.service, ops.persistent, ops.header, ops.md5sum);

  if (client)
  {
    boost::mutex::scoped_lock lock(collection_->mutex_);
    collection_->srv_cs_.push_back(client.impl_);
  }

  return client;
}

Timer NodeHandle::createTimer(Duration period, const TimerCallback& callback) const
{
  TimerOptions ops;
  ops.period = period;
  ops.callback = callback;
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
      ops.callback_queue = &g_global_queue;
    }
  }

  Timer timer(ops);
  timer.start();
  return timer;
}

WallTimer NodeHandle::createWallTimer(WallDuration period, const WallTimerCallback& callback) const
{
  WallTimerOptions ops;
  ops.period = period;
  ops.callback = callback;
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

void NodeHandle::setParam(const std::string &key, const XmlRpc::XmlRpcValue &v) const
{
  return node_->setParam(resolveName(key), v);
}

void NodeHandle::setParam(const std::string &key, const std::string &s) const
{
  return node_->setParam(resolveName(key), s);
}

void NodeHandle::setParam(const std::string &key, const char* s) const
{
  return node_->setParam(resolveName(key), s);
}

void NodeHandle::setParam(const std::string &key, double d) const
{
  return node_->setParam(resolveName(key), d);
}

void NodeHandle::setParam(const std::string &key, int i) const
{
  return node_->setParam(resolveName(key), i);
}

void NodeHandle::setParam(const std::string &key, bool b) const
{
  return node_->setParam(resolveName(key), b);
}

bool NodeHandle::hasParam(const std::string &key) const
{
  return node_->hasParam(resolveName(key));
}

bool NodeHandle::deleteParam(const std::string &key) const
{
  return node_->deleteParam(resolveName(key));
}

bool NodeHandle::getParam(const std::string &key, XmlRpc::XmlRpcValue &v, bool use_cache) const
{
  return node_->getParam(resolveName(key), v, use_cache);
}

bool NodeHandle::getParam(const std::string &key, std::string &s, bool use_cache) const
{
  return node_->getParam(resolveName(key), s, use_cache);
}

bool NodeHandle::getParam(const std::string &key, double &d, bool use_cache) const
{
  return node_->getParam(resolveName(key), d, use_cache);
}

bool NodeHandle::getParam(const std::string &key, int &i, bool use_cache) const
{
  return node_->getParam(resolveName(key), i, use_cache);
}

bool NodeHandle::getParam(const std::string &key, bool &b, bool use_cache) const
{
  return node_->getParam(resolveName(key), b, use_cache);
}

bool NodeHandle::searchParam(const std::string &key, std::string& result_out) const
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = resolveName("");
  params[1] = remapName(key);
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!node_->executeMasterXmlrpc("searchParam", params, result, payload, false))
  {
    return false;
  }

  result_out = (std::string)payload;

  return true;
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

bool NodeHandle::checkMaster() const
{
  return node_->checkMaster();
}

bool NodeHandle::getPublishedTopics(VP_string& topics) const
{
  return node_->getPublishedTopics(&topics);
}

bool NodeHandle::getNodes(V_string& nodes) const
{
  return node_->getNodes(nodes);
}

} // namespace ros
