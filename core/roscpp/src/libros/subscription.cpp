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

#include <sstream>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <sys/poll.h> // for POLLOUT
#include <cerrno>
#include <cstring>

#include "ros/common.h"
#include "ros/subscription.h"
#include "ros/publisher_link.h"
#include "ros/connection.h"
#include "ros/transport/transport_tcp.h"
#include "ros/transport/transport_udp.h"
#include "ros/callback_queue_interface.h"
#include "ros/this_node.h"
#include "ros/network.h"
#include "ros/poll_manager.h"
#include "ros/connection_manager.h"
#include "ros/message_deserializer.h"
#include "ros/subscription_queue.h"

using XmlRpc::XmlRpcValue;

namespace ros
{

Subscription::Subscription(const std::string &name, const std::string& md5sum, const std::string& datatype, const TransportHints& transport_hints)
: name_(name)
, md5sum_(md5sum)
, datatype_(datatype)
, dropped_(false)
, shutting_down_(false)
, transport_hints_(transport_hints)
{
}

Subscription::~Subscription()
{
  pending_connections_.clear();
  callbacks_.clear();
}

void Subscription::shutdown()
{
  {
    boost::mutex::scoped_lock lock(shutdown_mutex_);
    shutting_down_ = true;
  }

  drop();
}

XmlRpcValue Subscription::getStats()
{
  XmlRpcValue stats;
  stats[0] = name_;
  XmlRpcValue conn_data;
  conn_data.setSize(0);

  boost::mutex::scoped_lock lock(publisher_links_mutex_);

  uint32_t cidx = 0;
  for (V_PublisherLink::iterator c = publisher_links_.begin();
       c != publisher_links_.end(); ++c)
  {
    const PublisherLink::Stats& s = (*c)->getStats();
    conn_data[cidx][0] = (*c)->getConnectionID();
    conn_data[cidx][1] = (int)s.bytes_received_;
    conn_data[cidx][2] = (int)s.messages_received_;
    conn_data[cidx][3] = (int)s.drops_;
    conn_data[cidx][4] = 0; // figure out something for this. not sure.
  }

  stats[1] = conn_data;
  return stats;
}

// rospy returns values like this:
// (1, 'http://127.0.0.1:62365/', 'i', 'TCPROS', '/chatter')
//
// We're outputting something like this:
// (0, http://127.0.0.1:62438/, i, TCPROS, /chatter)
void Subscription::getInfo(XmlRpc::XmlRpcValue& info)
{
  boost::mutex::scoped_lock lock(publisher_links_mutex_);

  for (V_PublisherLink::iterator c = publisher_links_.begin();
       c != publisher_links_.end(); ++c)
  {
    XmlRpcValue curr_info;
    curr_info[0] = (int)(*c)->getConnectionID();
    curr_info[1] = (*c)->getPublisherXMLRPCURI();
    curr_info[2] = "i";
    curr_info[3] = (*c)->getTransportType();
    curr_info[4] = name_;
    info[info.size()] = curr_info;
  }
}

void Subscription::drop()
{
  if (!dropped_)
  {
    dropped_ = true;

    dropAllConnections();
  }
}

void Subscription::dropAllConnections()
{
  // Swap our subscribers list with a local one so we can only lock for a short period of time, because a
  // side effect of our calling drop() on connections can be re-locking the subscribers mutex
  V_PublisherLink localsubscribers;

  {
    boost::mutex::scoped_lock lock(publisher_links_mutex_);

    localsubscribers.swap(publisher_links_);
  }

  V_PublisherLink::iterator it = localsubscribers.begin();
  V_PublisherLink::iterator end = localsubscribers.end();
  for (;it != end; ++it)
  {
    (*it)->getConnection()->drop();
  }
}

bool Subscription::pubUpdate(const V_string& new_pubs)
{
  boost::mutex::scoped_lock lock(shutdown_mutex_);

  if (shutting_down_ || dropped_)
  {
    return false;
  }

  bool retval = true;

  V_string additions;
  V_PublisherLink subtractions;
  V_PublisherLink to_add;
  // could use the STL set operations... but these sets are so small
  // it doesn't really matter.
  {
    boost::mutex::scoped_lock lock(publisher_links_mutex_);

    for (V_PublisherLink::iterator spc = publisher_links_.begin();
         spc!= publisher_links_.end(); ++spc)
    {
      bool found = false;
      for (V_string::const_iterator up_i = new_pubs.begin();
           !found && up_i != new_pubs.end(); ++up_i)
      {
        if ((*spc)->getPublisherXMLRPCURI() == *up_i)
        {
          found = true;
          break;
        }
      }

      if (!found)
      {
        subtractions.push_back(*spc);
      }
    }

    for (V_string::const_iterator up_i  = new_pubs.begin(); up_i != new_pubs.end(); ++up_i)
    {
      bool found = false;
      for (V_PublisherLink::iterator spc = publisher_links_.begin();
           !found && spc != publisher_links_.end(); ++spc)
      {
        if (*up_i == (*spc)->getPublisherXMLRPCURI())
        {
          found = true;
          break;
        }
      }

      if (!found)
      {
        boost::mutex::scoped_lock lock(pending_connections_mutex_);
        S_PendingConnection::iterator it = pending_connections_.begin();
        S_PendingConnection::iterator end = pending_connections_.end();
        for (; it != end; ++it)
        {
          if (*up_i == (*it)->getRemoteURI())
          {
            found = true;
            break;
          }
        }
      }

      if (!found)
      {
        additions.push_back(*up_i);
      }
    }
  }

  for (V_string::iterator i = additions.begin();
            i != additions.end(); ++i)
  {
    // this function should never negotiate a self-subscription
    if (XMLRPCManager::instance()->getServerURI() != *i)
    {
      retval &= negotiateConnection(*i);
    }
  }

  for (V_PublisherLink::iterator i = subtractions.begin();
           i != subtractions.end(); ++i)
  {
    ROS_DEBUG("Disconnecting from publisher of topic [%s] at [%s]",
                name_.c_str(), (*i)->getPublisherXMLRPCURI().c_str());
    (*i)->getConnection()->drop();
  }

  return retval;
}

bool Subscription::negotiateConnection(const std::string& xmlrpc_uri)
{
  XmlRpcValue tcpros_array, protos_array, params;
  XmlRpcValue udpros_array;
  TransportUDPPtr udp_transport;
  int protos = 0;
  V_string transports = transport_hints_.getTransports();
  if (transports.empty())
  {
    transport_hints_.reliable();
    transports = transport_hints_.getTransports();
  }
  for (V_string::const_iterator it = transports.begin();
       it != transports.end();
       ++it)
  {
    if (*it == "UDP")
    {
      int max_datagram_size = transport_hints_.getMaxDatagramSize();
      udp_transport = TransportUDPPtr(new TransportUDP(&PollManager::instance()->getPollSet()));
      if (!max_datagram_size)
        max_datagram_size = udp_transport->getMaxDatagramSize();
      udp_transport->createIncoming(0, false);
      udpros_array[0] = "UDPROS";
      M_string m;
      m["topic"] = getName();
      m["md5sum"] = md5sum();
      m["callerid"] = this_node::getName();
      m["type"] = datatype();
      boost::shared_array<uint8_t> buffer;
      uint32_t len;
      Header::write(m, buffer, len);
      XmlRpcValue v(buffer.get(), len);
      udpros_array[1] = v;
      udpros_array[2] = network::getHost();
      udpros_array[3] = udp_transport->getServerPort();
      udpros_array[4] = max_datagram_size;

      protos_array[protos++] = udpros_array;
    }
    else if (*it == "TCP")
    {
      tcpros_array[0] = std::string("TCPROS");
      protos_array[protos++] = tcpros_array;
    }
    else
    {
      ROS_WARN("Unsupported transport type hinted: %s, skipping", it->c_str());
    }
  }
  params[0] = this_node::getName();
  params[1] = name_;
  params[2] = protos_array;
  std::string peer_host;
  uint32_t peer_port;
  if (!network::splitURI(xmlrpc_uri, peer_host, peer_port))
  {
    ROS_ERROR("Bad xml-rpc URI: [%s]", xmlrpc_uri.c_str());
    return false;
  }

  XmlRpc::XmlRpcClient* c = new XmlRpc::XmlRpcClient(peer_host.c_str(), 
                                                     peer_port, "/");
 // if (!c.execute("requestTopic", params, result) || !g_node->validateXmlrpcResponse("requestTopic", result, proto))

  // Initiate the negotiation.  We'll come back and check on it later.
  if (!c->executeNonBlock("requestTopic", params))
  {
    ROS_ERROR("Failed to contact publisher [%s:%d] for topic [%s]",
              peer_host.c_str(), peer_port, name_.c_str());
    delete c;
    return false;
  }

  // The PendingConnectionPtr takes ownership of c, and will delete it on
  // destruction.
  PendingConnectionPtr conn(new PendingConnection(c, udp_transport, shared_from_this(), xmlrpc_uri));

  XMLRPCManager::instance()->addASyncConnection(conn);
  // Put this connection on the list that we'll look at later.
  {
    boost::mutex::scoped_lock pending_connections_lock(pending_connections_mutex_);
    pending_connections_.insert(conn);
  }

  return true;
}

void Subscription::pendingConnectionDone(const PendingConnectionPtr& conn, XmlRpcValue& result)
{
  boost::mutex::scoped_lock lock(shutdown_mutex_);
  if (shutting_down_ || dropped_)
  {
    return;
  }

  {
    boost::mutex::scoped_lock pending_connections_lock(pending_connections_mutex_);
    pending_connections_.erase(conn);
  }

  TransportUDPPtr udp_transport;

  std::string peer_host = conn->getClient()->getHost();
  uint32_t peer_port = conn->getClient()->getPort();
  std::stringstream ss;
  ss << "http://" << peer_host << ":" << peer_port << "/";
  std::string xmlrpc_uri = ss.str();
  udp_transport = conn->getUDPTransport();

  XmlRpc::XmlRpcValue proto;
  if(!XMLRPCManager::instance()->validateXmlrpcResponse("requestTopic", result, proto))
  {
    ROS_ERROR("Failed to contact publisher [%s:%d] for topic [%s]",
              peer_host.c_str(), peer_port, name_.c_str());
    return;
  }

  if (proto.size() == 0)
  {
    ROS_ERROR("Couldn't agree on any common protocols with [%s] for topic [%s]", xmlrpc_uri.c_str(), name_.c_str());
    return;
  }

  if (proto.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Available protocol info returned from %s is not a list.", xmlrpc_uri.c_str());
    return;
  }
  if (proto[0].getType() != XmlRpcValue::TypeString)
  {
    ROS_ERROR("Available protocol info list doesn't have a string as its first element.");
    return;
  }

  std::string proto_name = proto[0];
  if (proto_name == "TCPROS")
  {
    if (proto.size() != 3 ||
        proto[1].getType() != XmlRpcValue::TypeString ||
        proto[2].getType() != XmlRpcValue::TypeInt)
    {
      ROS_ERROR("publisher implements TCPROS, but the " \
                "parameters aren't string,int");
      return;
    }
    std::string pub_host = proto[1];
    int pub_port = proto[2];
    ROS_DEBUG("Connecting via tcpros to topic [%s] at host [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);

    TransportTCPPtr transport(new TransportTCP(&PollManager::instance()->getPollSet()));
    if (transport->connect(pub_host, pub_port))
    {
      ConnectionPtr connection(new Connection());
      PublisherLinkPtr pub_link(new PublisherLink(shared_from_this(), xmlrpc_uri, transport_hints_));

      connection->initialize(transport, false, HeaderReceivedFunc());
      pub_link->initialize(connection);

      ConnectionManager::instance()->addConnection(connection);

      boost::mutex::scoped_lock lock(publisher_links_mutex_);
      publisher_links_.push_back(pub_link);

      ROS_DEBUG("Connected to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
    else
    {
      ROS_ERROR("Failed to connect to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
  }
  else if (proto_name == "UDPROS")
  {
    if (proto.size() != 5 ||
        proto[1].getType() != XmlRpcValue::TypeString ||
        proto[2].getType() != XmlRpcValue::TypeInt ||
        proto[3].getType() != XmlRpcValue::TypeInt ||
        proto[4].getType() != XmlRpcValue::TypeInt)
    {
      ROS_ERROR("publisher implements UDPROS, but the " \
                "parameters aren't string,int,int");
      return;
    }
    std::string pub_host = proto[1];
    int pub_port = proto[2];
    int conn_id = proto[3];
    int max_datagram_size = proto[4];
    ROS_DEBUG("Connecting via udpros to topic [%s] at host [%s:%d] connection id [%08x] max_datagram_size [%d]", name_.c_str(), pub_host.c_str(), pub_port, conn_id, max_datagram_size);

    //TransportUDPPtr transport(new TransportUDP(&g_node->getPollSet()));

    //if (udp_transport->connect(pub_host, pub_port, conn_id))
    // Using if(1) below causes a bizarre compiler error on some OS X
    // machines.  Creating a variable and testing it doesn't.  Presumably
    // it's related to the conditional compilation that goes on inside
    // ROS_ERROR.
    int foo=1;
    if (foo)
    {
      ConnectionPtr connection(new Connection());
      PublisherLinkPtr pub_link(new PublisherLink(shared_from_this(), xmlrpc_uri, transport_hints_));

      connection->initialize(udp_transport, false, NULL);
      pub_link->initialize(connection);

      ConnectionManager::instance()->addConnection(connection);

      boost::mutex::scoped_lock lock(publisher_links_mutex_);
      publisher_links_.push_back(pub_link);

      ROS_DEBUG("Connected to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
    else
    {
      ROS_ERROR("Failed to connect to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
  }
  else
  {
    ROS_ERROR("Publisher offered unsupported transport [%s]", proto_name.c_str());
  }
}

bool Subscription::handleMessage(const boost::shared_array<uint8_t>& buf, size_t num_bytes, const boost::shared_ptr<M_string>& connection_header)
{
  bool dropped = false;

  boost::mutex::scoped_lock lock(callbacks_mutex_);
  invokeCallback(buf, num_bytes, connection_header);

  return dropped;
}

class SubscriptionCallback : public CallbackInterface
{
public:
  SubscriptionCallback(const SubscriptionQueuePtr& queue, uint64_t id)
  : queue_(queue)
  , id_(id)
  {}

  virtual CallResult call()
  {
    return queue_->call(id_);
  }

  virtual bool ready()
  {
    return queue_->ready(id_);
  }

private:
  SubscriptionQueuePtr queue_;
  uint64_t id_;
};
typedef boost::shared_ptr<SubscriptionCallback> SubscriptionCallbackPtr;

bool Subscription::addCallback(const SubscriptionMessageHelperPtr& helper, CallbackQueueInterface* queue, int32_t queue_size, const VoidPtr& tracked_object)
{
  ROS_ASSERT(helper);
  ROS_ASSERT(queue);
  if (helper->getMD5Sum() != md5sum())
  {
    return false;
  }

  {
    boost::mutex::scoped_lock lock(callbacks_mutex_);

    CallbackInfoPtr info(new CallbackInfo);
    info->helper_ = helper;
    info->callback_queue_ = queue;
    info->subscription_queue_.reset(new SubscriptionQueue(name_, queue_size));
    info->tracked_object_ = tracked_object;
    info->has_tracked_object_ = false;
    if (tracked_object)
    {
      info->has_tracked_object_ = true;
    }

    callbacks_.push_back(info);
  }

  return true;
}

void Subscription::removeCallback(const SubscriptionMessageHelperPtr& helper)
{
  boost::mutex::scoped_lock cbs_lock(callbacks_mutex_);
  for (V_CallbackInfo::iterator it = callbacks_.begin();
       it != callbacks_.end(); ++it)
  {
    if ((*it)->helper_ == helper)
    {
      const CallbackInfoPtr& info = *it;
      info->subscription_queue_->clear();
      callbacks_.erase(it);
      break;
    }
  }
}

void Subscription::invokeCallback(const boost::shared_array<uint8_t>& buffer, size_t num_bytes, const boost::shared_ptr<M_string>& connection_header)
{
  MessagePtr msg;
  MessageDeserializerPtr deserializer;

  for (V_CallbackInfo::iterator cb = callbacks_.begin();
       cb != callbacks_.end(); ++cb)
  {
    const CallbackInfoPtr& info = *cb;

    ROS_ASSERT(info->callback_queue_);

    if (!deserializer)
    {
      deserializer.reset(new MessageDeserializer(info->helper_, buffer, num_bytes, connection_header));
    }

    uint64_t id = info->subscription_queue_->push(info->helper_, deserializer, info->has_tracked_object_, info->tracked_object_);
    //ROS_DEBUG_STREAM("Subscription::invokeCallback for " << (*connection_header)["callerid"] << ", pushed buffer " << (uint64_t*)buffer.get() << " seq " << *(uint32_t*)buffer.get());
    SubscriptionCallbackPtr cb(new SubscriptionCallback(info->subscription_queue_, id));
    info->callback_queue_->addCallback(cb);
  }
}

void Subscription::removePublisherLink(const PublisherLinkPtr& pub_link)
{
  boost::mutex::scoped_lock lock(publisher_links_mutex_);

  V_PublisherLink::iterator it = std::find(publisher_links_.begin(), publisher_links_.end(), pub_link);
  if (it != publisher_links_.end())
  {
    publisher_links_.erase(it);
  }
}

const std::string Subscription::datatype()
{
  return datatype_;
}

const std::string Subscription::md5sum()
{
  return md5sum_;
}

}
