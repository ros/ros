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
#include "ros/subscribe_options.h"

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
  xmlrpc_manager_->bind("getSubscriptions", boost::bind(&TopicManager::getSubscriptionsCallback, this, _1, _2));
  xmlrpc_manager_->bind("getPublications", boost::bind(&TopicManager::getPublicationsCallback, this, _1, _2));

  poll_manager_->addPollThreadListener(boost::bind(&TopicManager::processPublishQueues, this));
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
    shutting_down_ = true;
  }

  xmlrpc_manager_->unbind("publisherUpdate");
  xmlrpc_manager_->unbind("requestTopic");
  xmlrpc_manager_->unbind("getBusStats");
  xmlrpc_manager_->unbind("getBusInfo");
  xmlrpc_manager_->unbind("getSubscriptions");
  xmlrpc_manager_->unbind("getPublications");

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
}

void TopicManager::processPublishQueues()
{
  boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

  V_Publication::iterator it = advertised_topics_.begin();
  V_Publication::iterator end = advertised_topics_.end();
  for (; it != end; ++it)
  {
    const PublicationPtr& pub = *it;
    pub->processPublishQueue();
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

bool md5sumsMatch(const std::string& lhs, const std::string& rhs)
{
  return lhs == "*" || rhs == "*" || lhs == rhs;
}

bool TopicManager::addSubCallback(const SubscribeOptions& ops)
{
  // spin through the subscriptions and see if we find a match. if so, use it.
  bool found = false;
  bool found_topic = false;

  SubscriptionPtr sub;

  {
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
        found_topic = true;
        if (md5sumsMatch(ops.md5sum, sub->md5sum()))
        {
          found = true;
        }
        break;
      }
    }
  }

  if (found_topic && !found)
  {
    std::stringstream ss;
    ss << "Tried to subscribe to a topic with the same name but different md5sum as a topic that was already subscribed [" << ops.datatype << "/" << ops.md5sum << " vs. " << sub->datatype() << "/" << sub->md5sum() << "]";
    throw ConflictingSubscriptionException(ss.str());
  }
  else if (found)
  {
    if (!sub->addCallback(ops.helper, ops.md5sum, ops.callback_queue, ops.queue_size, ops.tracked_object, ops.allow_concurrent_callbacks))
    {
      return false;
    }
  }

  return found;
}

// this function has the subscription code that doesn't need to be templated.
bool TopicManager::subscribe(const SubscribeOptions& ops)
{
  boost::mutex::scoped_lock lock(subs_mutex_);

  if (addSubCallback(ops))
  {
    return true;
  }

  if (isShuttingDown())
  {
    return false;
  }

  if (ops.md5sum.empty())
  {
    throw InvalidParameterException("Subscribing to topic [" + ops.topic + "] with an empty md5sum");
  }

  if (ops.datatype.empty())
  {
    throw InvalidParameterException("Subscribing to topic [" + ops.topic + "] with an empty datatype");
  }

  if (!ops.helper)
  {
    throw InvalidParameterException("Subscribing to topic [" + ops.topic + "] without a callback");
  }

  const std::string& md5sum = ops.md5sum;
  std::string datatype = ops.datatype;

  SubscriptionPtr s(new Subscription(ops.topic, md5sum, datatype, ops.transport_hints));
  s->addCallback(ops.helper, ops.md5sum, ops.callback_queue, ops.queue_size, ops.tracked_object, ops.allow_concurrent_callbacks);

  if (!registerSubscriber(s, ops.datatype))
  {
    ROS_WARN("couldn't register subscriber on topic [%s]", ops.topic.c_str());
    s->shutdown();
    return false;
  }

  subscriptions_.push_back(s);

  return true;
}

bool TopicManager::advertise(const AdvertiseOptions& ops, const SubscriberCallbacksPtr& callbacks)
{
  if (ops.datatype == "*")
  {
    std::stringstream ss;
    ss << "Advertising with * as the datatype is not allowed.  Topic [" << ops.topic << "]";
    throw InvalidParameterException(ss.str());
  }

  if (ops.md5sum == "*")
  {
    std::stringstream ss;
    ss << "Advertising with * as the md5sum is not allowed.  Topic [" << ops.topic << "]";
    throw InvalidParameterException(ss.str());
  }

  if (ops.md5sum.empty())
  {
    throw InvalidParameterException("Advertising on topic [" + ops.topic + "] with an empty md5sum");
  }

  if (ops.datatype.empty())
  {
    throw InvalidParameterException("Advertising on topic [" + ops.topic + "] with an empty datatype");
  }

  if (ops.message_definition.empty())
  {
    ROS_WARN("Advertising on topic [%s] with an empty message definition.  Some tools (e.g. rosbag) may not work correctly.", ops.topic.c_str());
  }

  PublicationPtr pub;

  {
    boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

    if (isShuttingDown())
    {
      return false;
    }

    pub = lookupPublicationWithoutLock(ops.topic);
    if (pub && pub->getNumCallbacks() == 0)
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

      pub->addCallbacks(callbacks);

      return true;
    }

    pub = PublicationPtr(new Publication(ops.topic, ops.datatype, ops.md5sum, ops.message_definition, ops.queue_size, ops.latch, ops.has_header));
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
      if ((*s)->getName() == ops.topic && md5sumsMatch((*s)->md5sum(), ops.md5sum) && !(*s)->isDropped())
      {
        found = true;
        sub = *s;
        break;
      }
    }
  }

  if(found)
  {
    sub->addLocalConnection(pub);
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
  PublicationPtr pub;
  V_Publication::iterator i;
  {
    boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

    if (isShuttingDown())
    {
      return false;
    }

    for (i = advertised_topics_.begin();
         i != advertised_topics_.end(); ++i)
    {
      if(((*i)->getName() == topic) && (!(*i)->isDropped()))
      {
        pub = *i;
        break;
      }
    }
  }

  if (!pub)
  {
    return false;
  }

  pub->removeCallbacks(callbacks);

  {
    boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);
    if (pub->getNumCallbacks() == 0)
    {
      unregisterPublisher(pub->getName());
      pub->drop();

      advertised_topics_.erase(i);

      {
        boost::mutex::scoped_lock lock(advertised_topic_names_mutex_);
        advertised_topic_names_.remove(pub->getName());
      }
    }
  }

  return true;
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
  for (int i = 0; i < payload.size(); i++)
  {
    if (payload[i] != xmlrpc_manager_->getServerURI())
    {
      pub_uris.push_back(string(payload[i]));
    }
  }

  bool self_subscribed = false;
  PublicationPtr pub;
  const std::string& sub_md5sum = s->md5sum();
  // Figure out if we have a local publisher
  {
    boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);
    V_Publication::const_iterator it = advertised_topics_.begin();
    V_Publication::const_iterator end = advertised_topics_.end();
    for (; it != end; ++it)
    {
      pub = *it;
      const std::string& pub_md5sum = pub->getMD5Sum();

      if (pub->getName() == s->getName() && !pub->isDropped())
	{
	  if (!md5sumsMatch(pub_md5sum, sub_md5sum))
	    {
	      ROS_ERROR("md5sum mismatch making local subscription to topic %s.",
			s->getName().c_str());
	      ROS_ERROR("Subscriber expects type %s, md5sum %s",
			s->datatype().c_str(), s->md5sum().c_str());
	      ROS_ERROR("Publisher provides type %s, md5sum %s",
			pub->getDataType().c_str(), pub->getMD5Sum().c_str());
	      return false;
	    }

	  self_subscribed = true;
	  break;
	}
    }
  }

  s->pubUpdate(pub_uris);
  if (self_subscribed)
  {
    s->addLocalConnection(pub);
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
    	ROSCPP_LOG_DEBUG( "requestTopic protocol list was not a list of lists");
      return false;
    }

    if (proto[0].getType() != XmlRpcValue::TypeString)
    {
    	ROSCPP_LOG_DEBUG( "requestTopic received a protocol list in which a sublist " \
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
      	ROSCPP_LOG_DEBUG("Invalid protocol parameters for UDPROS");
        return false;
      }
      std::vector<char> header_bytes = proto[1];
      boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[header_bytes.size()]);
      memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
      Header h;
      string err;
      if (!h.parse(buffer, header_bytes.size(), err))
      {
      	ROSCPP_LOG_DEBUG("Unable to parse UDPROS connection header: %s", err.c_str());
        return false;
      }

      PublicationPtr pub_ptr = lookupPublication(topic);
      if(!pub_ptr)
      {
      	ROSCPP_LOG_DEBUG("Unable to find advertised topic %s for UDPROS connection", topic.c_str());
        return false;
      }

      std::string host = proto[2];
      int port = proto[3];

      M_string m;
      std::string error_msg;
      if (!pub_ptr->validateHeader(h, error_msg))
      {
        ROSCPP_LOG_DEBUG("Error validating header from [%s:%d] for topic [%s]: %s", host.c_str(), port, topic.c_str(), error_msg.c_str());
        return false;
      }

      int max_datagram_size = proto[4];
      int conn_id = connection_manager_->getNewConnectionID();
      TransportUDPPtr transport = connection_manager_->getUDPServerTransport()->createOutgoing(host, port, conn_id, max_datagram_size);
      connection_manager_->udprosIncomingConnection(transport, h);

      XmlRpcValue udpros_params;
      udpros_params[0] = string("UDPROS");
      udpros_params[1] = network::getHost();
      udpros_params[2] = connection_manager_->getUDPServerTransport()->getServerPort();
      udpros_params[3] = conn_id;
      udpros_params[4] = max_datagram_size;
      m["topic"] = topic;
      m["md5sum"] = pub_ptr->getMD5Sum();
      m["type"] = pub_ptr->getDataType();
      m["callerid"] = this_node::getName();
      m["message_definition"] = pub_ptr->getMessageDefinition();
      boost::shared_array<uint8_t> msg_def_buffer;
      uint32_t len;
      Header::write(m, msg_def_buffer, len);
      XmlRpcValue v(msg_def_buffer.get(), len);
      udpros_params[5] = v;
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

  ROSCPP_LOG_DEBUG( "Currently, roscpp only supports TCPROS. The caller to " \
             "requestTopic did not support TCPROS, so there are no " \
             "protocols in common.");
  return false;
}

void TopicManager::publish(const std::string& topic, const boost::function<SerializedMessage(void)>& serfunc, SerializedMessage& m)
{
  boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

  if (isShuttingDown())
  {
    return;
  }

  PublicationPtr p = lookupPublicationWithoutLock(topic);
  if (p->hasSubscribers() || p->isLatching())
  {
    ROS_DEBUG_NAMED("superdebug", "Publishing message on topic [%s] with sequence number [%d]", p->getName().c_str(), p->getSequence());

    // Determine what kinds of subscribers we're publishing to.  If they're intraprocess with the same C++ type we can
    // do a no-copy publish.
    bool nocopy = false;
    bool serialize = false;

    // We can only do a no-copy publish if a shared_ptr to the message is provided, and we have type information for it
    if (m.type_info && m.message)
    {
      p->getPublishTypes(serialize, nocopy, *m.type_info);
    }
    else
    {
      serialize = true;
    }

    if (!nocopy)
    {
      m.message.reset();
      m.type_info = 0;
    }

    if (serialize || p->isLatching())
    {
      SerializedMessage m2 = serfunc();
      m.buf = m2.buf;
      m.num_bytes = m2.num_bytes;
      m.message_start = m2.message_start;
    }

    p->publish(m);

    // If we're not doing a serialized publish we don't need to signal the pollset.  The write()
    // call inside signal() is actually relatively expensive when doing a nocopy publish.
    if (serialize)
    {
      poll_manager_->getPollSet().signal();
    }
  }
  else
  {
    p->incrementSequence();
  }
}

void TopicManager::incrementSequence(const std::string& topic)
{
  PublicationPtr pub = lookupPublication(topic);
  if (pub)
  {
    pub->incrementSequence();
  }
}

bool TopicManager::isLatched(const std::string& topic)
{
  PublicationPtr pub = lookupPublication(topic);
  if (pub)
  {
    return pub->isLatched();
  }

  return false;
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

bool TopicManager::unsubscribe(const std::string &topic, const SubscriptionCallbackHelperPtr& helper)
{
  SubscriptionPtr sub;

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (isShuttingDown())
    {
      return false;
    }

    L_Subscription::iterator it;
    for (it = subscriptions_.begin();
         it != subscriptions_.end(); ++it)
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

      L_Subscription::iterator it;
      for (it = subscriptions_.begin();
           it != subscriptions_.end(); ++it)
      {
        if ((*it)->getName() == topic)
        {
          subscriptions_.erase(it);
          break;
        }
      }

      if (!unregisterSubscriber(topic))
      {
      	ROSCPP_LOG_DEBUG("Couldn't unregister subscriber for topic [%s]", topic.c_str());
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

  PublicationPtr p = lookupPublicationWithoutLock(topic);
  if (p)
  {
    return p->getNumSubscribers();
  }

  return 0;
}

size_t TopicManager::getNumSubscriptions()
{
  boost::mutex::scoped_lock lock(subs_mutex_);
  return subscriptions_.size();
}

size_t TopicManager::getNumPublishers(const std::string &topic)
{
  boost::mutex::scoped_lock lock(subs_mutex_);

  if (isShuttingDown())
  {
    return 0;
  }

  for (L_Subscription::const_iterator t = subscriptions_.begin();
       t != subscriptions_.end(); ++t)
  {
    if (!(*t)->isDropped() && (*t)->getName() == topic)
    {
      return (*t)->getNumPublishers();
    }
  }

  return 0;
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

void TopicManager::getSubscriptions(XmlRpcValue &subs)
{
  // force these guys to be arrays, even if we don't populate them
  subs.setSize(0);

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    uint32_t sidx = 0;

    for (L_Subscription::iterator t = subscriptions_.begin(); t != subscriptions_.end(); ++t)
    {
      XmlRpcValue sub;
      sub[0] = (*t)->getName();
      sub[1] = (*t)->datatype();
      subs[sidx++] = sub;
    }
  }
}

void TopicManager::getPublications(XmlRpcValue &pubs)
{
  // force these guys to be arrays, even if we don't populate them
  pubs.setSize(0);

  {
    boost::recursive_mutex::scoped_lock lock(advertised_topics_mutex_);

    uint32_t sidx = 0;

    for (V_Publication::iterator t = advertised_topics_.begin();
         t != advertised_topics_.end(); ++t)
    {
      XmlRpcValue pub;
      pub[0] = (*t)->getName();
      pub[1] = (*t)->getDataType();
      pubs[sidx++] = pub;
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

void TopicManager::getSubscriptionsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  result[0] = 1;
  result[1] = std::string("subscriptions");
  XmlRpcValue response;
  getSubscriptions(response);
  result[2] = response;
}

void TopicManager::getPublicationsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  result[0] = 1;
  result[1] = std::string("publications");
  XmlRpcValue response;
  getPublications(response);
  result[2] = response;
}

} // namespace ros
