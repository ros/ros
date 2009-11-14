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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#include "ros/topic_manager.h"
#include "ros/xmlrpc_manager.h"
#include "ros/connection_manager.h"
#include "ros/poll_manager.h"
#include "ros/publication.h"
#include "ros/subscription.h"
#include "ros/this_node.h"
#include "ros/network.h"
#include "ros/master.h"
#include "ros/transport/transport_tcp.h"
#include "ros/transport/transport_udp.h"
#include "ros/rosout_appender.h"
#include "ros/init.h"
#include "ros/file_log.h"

#include "XmlRpc.h"

#include <ros/console.h>

using namespace XmlRpc; // A battle to be fought later
using namespace std; // sigh

/// \todo Locking can be significantly simplified here once the Node API goes away.

namespace ros
{

TopicManagerPtr g_topic_manager;
boost::mutex g_topic_manager_mutex;
const TopicManagerPtr& TopicManager::instance()
{
  if (!g_topic_manager)
  {
    boost::mutex::scoped_lock lock(g_topic_manager_mutex);
    if (!g_topic_manager)
    {
      g_topic_manager.reset(new TopicManager);
    }
  }

  return g_topic_manager;
}

TopicManager::TopicManager()
: shutting_down_(false)
{
}

TopicManager::~TopicManager()
{
  shutdown();
}

void TopicManager::start()
{
  boost::mutex::scoped_lock shutdown_lock(shutting_down_mutex_);
  shutting_down_ = false;

  poll_manager_ = PollManager::instance();
  connection_manager_ = ConnectionManager::instance();
  xmlrpc_manager_ = XMLRPCManager::instance();

  xmlrpc_manager_->bind("publisherUpdate", boost::bind(&TopicManager::pubUpdateCallback, this, _1, _2));
  xmlrpc_manager_->bind("requestTopic", boost::bind(&TopicManager::requestTopicCallback, this, _1, _2));
  xmlrpc_manager_->bind("getBusStats", boost::bind(&TopicManager::getBusStatsCallback, this, _1, _2));
  xmlrpc_manager_->bind("getBusInfo", boost::bind(&TopicManager::getBusInfoCallback, this, _1, _2));

  poll_manager_->addPollThreadListener(boost::bind(&TopicManager::processPublishQueue, this));
}

void TopicManager::shutdown()
{
  boost::mutex::scoped_lock shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return;
  }

  {
    boost::recursive_mutex::scoped_lock lock1(advertised_topics_mutex_);
    boost::mutex::scoped_lock lock2(subs_mutex_);
    boost::mutex::scoped_lock lock3(publish_queue_mutex_);
    shutting_down_ = true;
  }

  xmlrpc_manager_->unbind("publisherUpdate");
  xmlrpc_manager_->unbind("requestTopic");
  xmlrpc_manager_->unbind("getBusStats");
  xmlrpc_manager_->unbind("getBusInfo");

  ROSCPP_LOG_DEBUG("Shutting down topics...");
  ROSCPP_LOG_DEBUG("  shutting down publishers");
  {
    boost::recursive_mutex::scoped_lock adv_lock(advertised_topics_mutex_);

    for (V_Publication::iterator i = advertised_topics_.begin();
         i != advertised_topics_.end(); ++i)
    {
      if(!(*i)->isDropped())
      {
        unregisterPublisher((*i)->getName());
      }
      (*i)->drop();
    }
    advertised_topics_.clear();
  }

  // unregister all of our subscriptions
  ROSCPP_LOG_DEBUG("  shutting down subscribers");
  {
    boost::mutex::scoped_lock subs_lock(subs_mutex_);

    for (L_Subscription::iterator s = subscriptions_.begin(); s != subscriptions_.end(); ++s)
    {
      // Remove us as a subscriber from the master
      unregisterSubscriber((*s)->getName());
      // now, drop our side of the connection
      (*s)->shutdown();
    }
    subscriptions_.clear();
  }

  publish_queue_.clear();
}

void TopicManager::processPublishQueue()
{
  V_PublicationAndSerializedMessagePair queue;
  {
    boost::mutex::scoped_lock lock(publish_queue_mutex_);

    if (isShuttingDown())
    {
      return;
    }

    queue.insert(queue.end(), publish_queue_.begin(), publish_queue_.end());
    publish_queue_.clear();
  }

  if (queue.empty())
  {
    return;
  }

  V_PublicationAndSerializedMessagePair::iterator it = queue.begin();
  V_PublicationAndSerializedMessagePair::iterator end = queue.end();
  for (; it != end; ++it)
  {
    PublicationPtr pub = it->first;
    pub->enqueueMessage(it->second);
  }
}

void TopicManager::getAdvertisedTopics(V_string& topics)
{
  boost::mutex::scoped_lock lock(advertised_topic_names_mutex_);

  topics.resize(advertised_topic_names_.size());
  std::copy(advertised_topic_names_.begin(),
            advertised_topic_names_.end(),
            topics.begin());
}

void TopicManager::getSubscribedTopics(V_string& topics)
{
  boost::mutex::scoped_lock lock(subs_mutex_);

  topics.reserve(subscriptions_.size());
  L_Subscription::const_iterator it = subscriptions_.begin();
  L_Subscription::const_iterator end = subscriptions_.end();
  for (; it != end; ++it)
  {
    const SubscriptionPtr& sub = *it;
    topics.push_back(sub->getName());
  }
}

PublicationPtr TopicManager::lookupPublication(const std::string& topic)
{
  boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

  return lookupPublicationWithoutLock(topic);
}

bool TopicManager::addSubCallback(const SubscribeOptions& ops)
{
  // spin through the subscriptions and see if we find a match. if so, use it.
  bool found = false;

  SubscriptionPtr sub;

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (isShuttingDown())
    {
      return false;
    }

    for (L_Subscription::iterator s = subscriptions_.begin();
         s != subscriptions_.end() && !found; ++s)
    {
      sub = *s;
      if (!sub->isDropped() && sub->getName() == ops.topic)
      {
        if (sub->md5sum() == ops.helper->getMD5Sum())
        {
          found = true;
          break;
        }
      }
    }
  }

  if (found)
  {
    if (!sub->addCallback(ops.helper, ops.callback_queue, ops.queue_size, ops.tracked_object))
    {
      return false;
    }
  }

  return found;
}

// this function has the subscription code that doesn't need to be templated.
bool TopicManager::subscribe(const SubscribeOptions& ops)
{
  if (addSubCallback(ops))
  {
    return true;
  }

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (isShuttingDown())
    {
      return false;
    }
  }

  std::string md5sum = ops.helper->getMD5Sum();
  std::string datatype = ops.helper->getDataType();

  SubscriptionPtr s(new Subscription(ops.topic, md5sum, datatype, ops.transport_hints));
  s->addCallback(ops.helper, ops.callback_queue, ops.queue_size, ops.tracked_object);

  if (!registerSubscriber(s, ops.datatype))
  {
    ROS_WARN("couldn't register subscriber on topic [%s]", ops.topic.c_str());
    s->shutdown();
    return false;
  }

  {
    boost::mutex::scoped_lock lock(subs_mutex_);
    subscriptions_.push_back(s);
  }

  return true;
}

bool TopicManager::advertise(const AdvertiseOptions& ops, const SubscriberCallbacksPtr& callbacks)
{
  if (ops.datatype == "*")
  {
    ROS_WARN("Advertising on topic [%s] with datatype [*].  If you are not playing back an old bag file, this is a problem.", ops.topic.c_str());
  }

  if (ops.md5sum == "*")
  {
    ROS_WARN("Advertising on topic [%s] with md5sum [*].  If you are not playing back an old bag file, this is a problem.", ops.topic.c_str());
  }

  {
    boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

    if (isShuttingDown())
    {
      return false;
    }

    PublicationPtr pub = lookupPublicationWithoutLock(ops.topic);
    if (pub && pub->getRefcount() == 0)
    {
      pub.reset();
    }

    if (pub)
    {
      if (pub->getMD5Sum() != ops.md5sum)
      {
        ROS_ERROR("Tried to advertise on topic [%s] with md5sum [%s] and datatype [%s], but the topic is already advertised as md5sum [%s] and datatype [%s]",
                  ops.topic.c_str(), ops.md5sum.c_str(), ops.datatype.c_str(), pub->getMD5Sum().c_str(), pub->getDataType().c_str());
        return false;
      }

      pub->incrementRefcount();
      pub->addCallbacks(callbacks);

      return true;
    }

    pub = PublicationPtr(new Publication(ops.topic, ops.datatype, ops.md5sum, ops.message_definition, ops.queue_size, ops.callback_queue, ops.latch));
    pub->addCallbacks(callbacks);
    advertised_topics_.push_back(pub);
  }


  {
    boost::mutex::scoped_lock lock(advertised_topic_names_mutex_);
    advertised_topic_names_.push_back(ops.topic);
  }

  // Check whether we've already subscribed to this topic.  If so, we'll do
  // the self-subscription here, to avoid the deadlock that would happen if
  // the ROS thread later got the publisherUpdate with its own XMLRPC URI.
  // The assumption is that advertise() is called from somewhere other
  // than the ROS thread.
  bool found = false;
  SubscriptionPtr sub;
  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    for (L_Subscription::iterator s = subscriptions_.begin();
         s != subscriptions_.end() && !found; ++s)
    {
      if ((*s)->getName() == ops.topic && (*s)->md5sum() == ops.md5sum && !(*s)->isDropped())
      {
        found = true;
        sub = *s;
        break;
      }
    }
  }

  if(found)
  {
    sub->negotiateConnection(xmlrpc_manager_->getServerURI());
  }

  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = ops.topic;
  args[2] = ops.datatype;
  args[3] = xmlrpc_manager_->getServerURI();
  master::execute("registerPublisher", args, result, payload, true);

  return true;
}

bool TopicManager::unadvertise(const std::string &topic, const SubscriberCallbacksPtr& callbacks)
{
  boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

  if (isShuttingDown())
  {
    return false;
  }

  for (V_Publication::iterator i = advertised_topics_.begin();
       i != advertised_topics_.end(); ++i)
  {
    if(((*i)->getName() == topic) && (!(*i)->isDropped()))
    {
      PublicationPtr pub = *i;
      pub->decrementRefcount();
      pub->removeCallbacks(callbacks);

      if (pub->getRefcount() == 0)
      {
        unregisterPublisher(pub->getName());
        pub->drop();

        {
          advertised_topics_.erase(i);
        }

        {
          boost::mutex::scoped_lock lock(advertised_topic_names_mutex_);
          advertised_topic_names_.remove(pub->getName());
        }
      }

      return true;
    }
  }

  return false;
}

bool TopicManager::unregisterPublisher(const std::string& topic)
{
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = topic;
  args[2] = xmlrpc_manager_->getServerURI();
  master::execute("unregisterPublisher", args, result, payload, false);

  return true;
}

bool TopicManager::isTopicAdvertised(const string &topic)
{
  for (V_Publication::iterator t = advertised_topics_.begin(); t != advertised_topics_.end(); ++t)
  {
    if (((*t)->getName() == topic) && (!(*t)->isDropped()))
    {
      return true;
    }
  }

  return false;
}

bool TopicManager::registerSubscriber(const SubscriptionPtr& s, const string &datatype)
{
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = s->getName();
  args[2] = datatype;
  args[3] = xmlrpc_manager_->getServerURI();

  if (!master::execute("registerSubscriber", args, result, payload, true))
  {
    return false;
  }

  vector<string> pub_uris;
  bool self_subscribed = false;
  for (int i = 0; i < payload.size(); i++)
  {
    if (payload[i] == xmlrpc_manager_->getServerURI())
    {
      self_subscribed = true;
    }
    else
    {
      pub_uris.push_back(string(payload[i]));
    }
  }

  s->pubUpdate(pub_uris);
  if (self_subscribed)
  {
    s->negotiateConnection(xmlrpc_manager_->getServerURI());
  }

  return true;
}

bool TopicManager::unregisterSubscriber(const string &topic)
{
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = topic;
  args[2] = xmlrpc_manager_->getServerURI();

  master::execute("unregisterSubscriber", args, result, payload, false);

  return true;
}

bool TopicManager::pubUpdate(const string &topic, const vector<string> &pubs)
{
  SubscriptionPtr sub;
  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (isShuttingDown())
    {
      return false;
    }

    ROS_DEBUG("Received update for topic [%s] (%d publishers)", topic.c_str(), (int)pubs.size());
    // find the subscription
    for (L_Subscription::const_iterator s  = subscriptions_.begin();
                                            s != subscriptions_.end(); ++s)
    {
      if ((*s)->getName() != topic || (*s)->isDropped())
        continue;

      sub = *s;
      break;
    }

  }

  if (sub)
  {
    sub->pubUpdate(pubs);
  }
  else
  {
    ROSCPP_LOG_DEBUG("got a request for updating publishers of topic %s, but I " \
              "don't have any subscribers to that topic.", topic.c_str());
  }

  return false;
}

bool TopicManager::requestTopic(const string &topic,
                         XmlRpcValue &protos,
                         XmlRpcValue &ret)
{
  for (int proto_idx = 0; proto_idx < protos.size(); proto_idx++)
  {
    XmlRpcValue proto = protos[proto_idx]; // save typing
    if (proto.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR( "requestTopic protocol list was not a list of lists");
      return false;
    }

    if (proto[0].getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR( "requestTopic received a protocol list in which a sublist " \
                 "did not start with a string");
      return false;
    }

    string proto_name = proto[0];
    if (proto_name == string("TCPROS"))
    {
      XmlRpcValue tcpros_params;
      tcpros_params[0] = string("TCPROS");
      tcpros_params[1] = network::getHost();
      tcpros_params[2] = int(connection_manager_->getTCPPort());
      ret[0] = int(1);
      ret[1] = string();
      ret[2] = tcpros_params;
      return true;
    }
    else if (proto_name == string("UDPROS"))
    {
      if (proto.size() != 5 ||
          proto[1].getType() != XmlRpcValue::TypeBase64 ||
          proto[2].getType() != XmlRpcValue::TypeString ||
          proto[3].getType() != XmlRpcValue::TypeInt ||
          proto[4].getType() != XmlRpcValue::TypeInt)
      {
        ROS_ERROR("Invalid protocol parameters for UDPROS");
        return false;
      }
      std::vector<char> header_bytes = proto[1];
      boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[header_bytes.size()]);
      memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
      Header h;
      string err;
      if (!h.parse(buffer, header_bytes.size(), err))
      {
        ROS_ERROR("Unable to parse UDPROS connection header: %s", err.c_str());
        return false;
      }

      std::string host = proto[2];
      int port = proto[3];
      int max_datagram_size = proto[4];
      int conn_id = connection_manager_->getUDPServerTransport()->generateConnectionId();
      TransportUDPPtr transport = connection_manager_->getUDPServerTransport()->createOutgoing(host, port, conn_id, max_datagram_size);
      connection_manager_->udprosIncomingConnection(transport, h);

      XmlRpcValue udpros_params;
      udpros_params[0] = string("UDPROS");
      udpros_params[1] = network::getHost();
      udpros_params[2] = connection_manager_->getUDPServerTransport()->getServerPort();
      udpros_params[3] = conn_id;
      udpros_params[4] = max_datagram_size;
      ret[0] = int(1);
      ret[1] = string();
      ret[2] = udpros_params;
      return true;
    }
    else
    {
      ROSCPP_LOG_DEBUG( "an unsupported protocol was offered: [%s]",
          proto_name.c_str());
    }
  }

  ROS_ERROR( "Currently, roscpp only supports TCPROS. The caller to " \
             "requestTopic did not support TCPROS, so there are no " \
             "protocols in common.");
  return false;
}

void TopicManager::publish(const std::string &topic, const Message& m)
{
  boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

  if (isShuttingDown())
  {
    return;
  }

  if(!isTopicAdvertised(topic))
  {
    ROS_ERROR("attempted to publish to topic %s, which was not " \
        "previously advertised. call advertise(\"%s\") first.",
        topic.c_str(), topic.c_str());
    return;
  }


  for (V_Publication::iterator t = advertised_topics_.begin();
       t != advertised_topics_.end(); ++t)
  {
    if ((*t)->getName() == topic)
    {
      if (m.__getDataType() != ((*t)->getDataType()))
      {
        ROS_ERROR("Topic [%s] advertised as [%s], but published as [%s]", topic.c_str(), (*t)->getDataType().c_str(), m.__getDataType().c_str());
      }
      else
      {
        publish(*t, m);
      }
      break;
    }
  }
}

void TopicManager::publish(const PublicationPtr& p, const Message& m)
{
  p->incrementSequence();
  if (p->hasSubscribers() || p->isLatching())
  {
    uint32_t msg_len = m.serializationLength();
    boost::shared_array<uint8_t> buf = boost::shared_array<uint8_t>(new uint8_t[msg_len + 4]);

    *((uint32_t*)buf.get()) = msg_len;
    m.serialize(buf.get() + 4, p->getSequence());
    ROS_DEBUG_NAMED("superdebug", "Publishing message on topic [%s] with sequence number [%d] of length [%d]", p->getName().c_str(), p->getSequence(), msg_len);

    boost::mutex::scoped_lock lock(publish_queue_mutex_);
    publish_queue_.push_back(std::make_pair(p, SerializedMessage(buf, msg_len + 4)));
    poll_manager_->getPollSet().signal();
  }
}

PublicationPtr TopicManager::lookupPublicationWithoutLock(const string &topic)
{
  PublicationPtr t;
  for (V_Publication::iterator i = advertised_topics_.begin();
       !t && i != advertised_topics_.end(); ++i)
  {
    if (((*i)->getName() == topic) && (!(*i)->isDropped()))
    {
      t = *i;
      break;
    }
  }

  return t;
}

bool TopicManager::unsubscribe(const std::string &topic, const SubscriptionMessageHelperPtr& helper)
{
  SubscriptionPtr sub;
  L_Subscription::iterator it;

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (isShuttingDown())
    {
      return false;
    }

    for (it = subscriptions_.begin();
         it != subscriptions_.end() && !sub; ++it)
    {
      if ((*it)->getName() == topic)
      {
        sub = *it;
        break;
      }
    }
  }

  if (!sub)
  {
    return false;
  }

  sub->removeCallback(helper);

  if (sub->getNumCallbacks() == 0)
  {
    // nobody is left. blow away the subscription.
    {
      boost::mutex::scoped_lock lock(subs_mutex_);

      subscriptions_.erase(it);

      if (!unregisterSubscriber(topic))
      {
        ROS_ERROR("Couldn't unregister subscriber for topic [%s]", topic.c_str());
      }
    }

    sub->shutdown();
    return true;
  }

  return true;
}

size_t TopicManager::getNumSubscribers(const std::string &topic)
{
  boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

  if (isShuttingDown())
  {
    return 0;
  }

  for (V_Publication::const_iterator t = advertised_topics_.begin();
       t != advertised_topics_.end(); ++t)
  {
    if ((*t)->getName() == topic)
    {
      return (*t)->getNumSubscribers();
    }
  }

  return 0;
}

size_t TopicManager::getNumSubscriptions()
{
  boost::mutex::scoped_lock lock(subs_mutex_);
  return subscriptions_.size();
}

void TopicManager::getBusStats(XmlRpcValue &stats)
{
  XmlRpcValue publish_stats, subscribe_stats, service_stats;
  // force these guys to be arrays, even if we don't populate them
  publish_stats.setSize(0);
  subscribe_stats.setSize(0);
  service_stats.setSize(0);

  uint32_t pidx = 0;
  {
    boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);
    for (V_Publication::iterator t = advertised_topics_.begin();
         t != advertised_topics_.end(); ++t)
    {
      publish_stats[pidx++] = (*t)->getStats();
    }
  }

  {
    uint32_t sidx = 0;

    boost::mutex::scoped_lock lock(subs_mutex_);
    for (L_Subscription::iterator t = subscriptions_.begin(); t != subscriptions_.end(); ++t)
    {
      subscribe_stats[sidx++] = (*t)->getStats();
    }
  }

  stats[0] = publish_stats;
  stats[1] = subscribe_stats;
  stats[2] = service_stats;
}

void TopicManager::getBusInfo(XmlRpcValue &info)
{
  // force these guys to be arrays, even if we don't populate them
  info.setSize(0);

  {
    boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

    for (V_Publication::iterator t = advertised_topics_.begin();
         t != advertised_topics_.end(); ++t)
    {
      (*t)->getInfo(info);
    }
  }

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    for (L_Subscription::iterator t = subscriptions_.begin(); t != subscriptions_.end(); ++t)
    {
      (*t)->getInfo(info);
    }
  }
}

extern ROSOutAppenderPtr g_rosout_appender;

void TopicManager::pubUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  std::vector<std::string> pubs;
  for (int idx = 0; idx < params[2].size(); idx++)
  {
    pubs.push_back(params[2][idx]);
  }
  if (pubUpdate(params[1], pubs))
  {
    result = xmlrpc::responseInt(1, "", 0);
  }
  else
  {
    std::string last_error = "Unknown Error";
    if ( g_rosout_appender != 0 )
    {
      last_error = g_rosout_appender->getLastError();
    }

    result = xmlrpc::responseInt(0, last_error, 0);
  }
}

void TopicManager::requestTopicCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!requestTopic(params[1], params[2], result))
  {
    std::string last_error = "Unknown Error";
    if ( g_rosout_appender != 0 )
    {
      last_error = g_rosout_appender->getLastError();
    }

    result = xmlrpc::responseInt(0, last_error, 0);
  }
}

void TopicManager::getBusStatsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  result[0] = 1;
  result[1] = std::string("");
  XmlRpcValue response;
  getBusStats(result);
  result[2] = response;
}

void TopicManager::getBusInfoCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  result[0] = 1;
  result[1] = std::string("");
  XmlRpcValue response;
  getBusInfo(response);
  result[2] = response;
}

} // namespace ros
