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
#include "ros/node.h"
#include "ros/common.h"
#include "ros/subscription.h"
#include "ros/subscriber.h"
#include "ros/connection.h"
#include "ros/transport/transport_tcp.h"

using namespace std;
using namespace ros;
using namespace XmlRpc;
using XmlRpc::XmlRpcValue;

Subscription::Subscription(const std::string &name, Message* m, AbstractFunctor *cb,
                   bool _threaded, int _max_queue)
: name_(name)
, md5sum_(m->__getMD5Sum())
, datatype_(m->__getDataType())
, dropped_(false)
, shutting_down_(false)
, threaded_(_threaded)
, max_queue_(_max_queue)
, queue_full_(false)
{
  addFunctorMessagePair(cb, m);

  if(threaded_)
  {
    callback_thread_ = boost::thread(boost::bind(&Subscription::subscriptionThreadFunc, this));
  }
}

Subscription::~Subscription()
{
  for (V_CallbackInfo::iterator cb = callbacks_.begin();
       cb != callbacks_.end(); ++cb)
  {
    delete (*cb)->callback_;
  }

  callbacks_.clear();
  subscribers_.clear();
}

void Subscription::shutdown()
{
  {
    boost::mutex::scoped_lock lock(shutdown_mutex_);
    shutting_down_ = true;
  }

  drop();

  // Set the callback thread free


  if (threaded_)
  {
    if(callback_thread_.get_id() != boost::this_thread::get_id())
    {
      // Grab the callback lock, to ensure that we wait until the callback,
      // which might be in progress, returns before we join the thread
      boost::mutex::scoped_lock lock(callbacks_mutex_);

      // We signal the condition, in case the callback thread is waiting on it
      inbox_cond_.notify_all();
      callback_thread_.join();

      // Empty the inbox queue.  No locking because the callback thread has already
      // been joined
      while(!inbox_.empty())
      {
        inbox_.pop();
      }
    }
    else
    {
      inbox_cond_.notify_all();
    }
  }
}

XmlRpcValue Subscription::getStats()
{
  XmlRpcValue stats;
  stats[0] = name_;
  XmlRpcValue conn_data;
  conn_data.setSize(0);

  boost::mutex::scoped_lock lock(subscribers_mutex_);

  uint32_t cidx = 0;
  for (vector<SubscriberPtr>::iterator c = subscribers_.begin();
       c != subscribers_.end(); ++c)
  {
    const Subscriber::Stats& s = (*c)->getStats();
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
  boost::mutex::scoped_lock lock(subscribers_mutex_);

  for (vector<SubscriberPtr>::iterator c = subscribers_.begin();
       c != subscribers_.end(); ++c)
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
  std::vector<SubscriberPtr> localsubscribers;

  {
    boost::mutex::scoped_lock lock(subscribers_mutex_);

    localsubscribers.swap(subscribers_);
  }

  std::vector<SubscriberPtr>::iterator it = localsubscribers.begin();
  std::vector<SubscriberPtr>::iterator end = localsubscribers.end();
  for (;it != end; ++it)
  {
    (*it)->getConnection()->drop();
  }
}

bool Subscription::pubUpdate(const vector<string> &new_pubs)
{
  boost::mutex::scoped_lock lock(shutdown_mutex_);

  if (shutting_down_ || dropped_)
  {
    return false;
  }

  bool retval = true;

  vector<string> additions;
  vector<SubscriberPtr> subtractions;
  vector<SubscriberPtr> to_add;
  // could use the STL set operations... but these sets are so small
  // it doesn't really matter.
  {
    boost::mutex::scoped_lock lock(subscribers_mutex_);

    for (vector<SubscriberPtr>::iterator spc = subscribers_.begin();
         spc!= subscribers_.end(); ++spc)
    {
      bool found = false;
      for (vector<string>::const_iterator up_i = new_pubs.begin();
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

    for (vector<string>::const_iterator up_i  = new_pubs.begin();
          up_i != new_pubs.end(); ++up_i)
    {
      bool found = false;
      for (vector<SubscriberPtr>::iterator spc = subscribers_.begin();
           !found && spc != subscribers_.end(); ++spc)
      {
        if (*up_i == (*spc)->getPublisherXMLRPCURI())
        {
          found = true;
          break;
        }
      }

      if (!found)
      {
        additions.push_back(*up_i);
      }
    }
  }

  for (vector<string>::iterator i = additions.begin();
            i != additions.end(); ++i)
  {
    // this function should never negotiate a self-subscription
    if (g_node->getXMLRPCURI() != *i)
    {
      retval &= negotiateConnection(*i);
    }
  }

  for (vector<SubscriberPtr>::iterator i = subtractions.begin();
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
  XmlRpcValue tcpros_array, protos_array, params, result, proto;
  tcpros_array[0] = string("TCPROS");
  protos_array[0] = tcpros_array;
  params[0] = g_node->getName();
  params[1] = name_;
  params[2] = protos_array;
  string peer_host;
  int peer_port;
  if (!Node::splitURI(xmlrpc_uri, peer_host, peer_port))
  {
    ROS_ERROR("Bad xml-rpc URI: [%s]", xmlrpc_uri.c_str());
    return false;
  }

  XmlRpcClient c(peer_host.c_str(), peer_port, "/");
  if (!c.execute("requestTopic", params, result) || !g_node->validateXmlrpcResponse("requestTopic", result, proto))
  {
    ROS_ERROR("Failed to contact publisher [%s:%d] for topic [%s]",
              peer_host.c_str(), peer_port, name_.c_str());
    return false;
  }

  if (proto.size() == 0)
  {
    ROS_ERROR("Couldn't agree on any common protocols with [%s] for topic [%s]", xmlrpc_uri.c_str(), name_.c_str());
    return false;
  }

  if (proto.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Available protocol info returned from %s is not a list.", xmlrpc_uri.c_str());
    return false;
  }
  if (proto[0].getType() != XmlRpcValue::TypeString)
  {
    ROS_ERROR("Available protocol info list doesn't have a string as its first element.");
    return false;
  }

  string proto_name = proto[0];
  if (proto_name == string("TCPROS"))
  {
    if (proto.size() != 3 ||
        proto[1].getType() != XmlRpcValue::TypeString ||
        proto[2].getType() != XmlRpcValue::TypeInt)
    {
      ROS_ERROR("publisher implements TCPROS, but the " \
                  "parameters aren't string,int");
      return false;
    }
    string pub_host = proto[1];
    int pub_port = proto[2];
    ROS_DEBUG("Connecting via tcpros to topic [%s] at host [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);

    TransportTCPPtr transport(new TransportTCP(&g_node->getPollSet()));
    if (transport->connect(pub_host, pub_port))
    {
      ConnectionPtr connection(new Connection());
      SubscriberPtr subscriber(new Subscriber(shared_from_this(), xmlrpc_uri));

      connection->initialize(transport, false, HeaderReceivedFunc());
      subscriber->initialize(connection);

      g_node->addConnection(connection);

      boost::mutex::scoped_lock lock(subscribers_mutex_);
      subscribers_.push_back(subscriber);

      ROS_DEBUG("Connected to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
    else
    {
      ROS_ERROR("Failed to connect to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
  }
  else
  {
    ROS_ERROR("roscpp only supports TCPROS, and the publisher offered something different [%s]", proto_name.c_str());

    return false;
  }

  return true;
}

bool Subscription::handleMessage(const boost::shared_array<uint8_t>& buf, size_t num_bytes, const boost::shared_ptr<M_string>& connection_header)
{
  bool dropped = false;

  if(threaded_)
  {
    // jfaust TODO: I'm fairly certain this extra copy is no longer necessary, remove + test
    boost::shared_array<uint8_t> my_copy;
    if(num_bytes)
    {
      my_copy = boost::shared_array<uint8_t>(new uint8_t[num_bytes]);
      ROS_ASSERT(my_copy.get());
      memcpy(my_copy.get(), buf.get(), num_bytes);
    }

    SerializedMessage m(my_copy, num_bytes);

    {
      boost::mutex::scoped_lock lock(inbox_mutex_);

      if((max_queue_ > 0) &&
         (inbox_.size() >= (unsigned int)max_queue_))
      {
        inbox_.pop();

        if (!queue_full_)
        {
          ROS_DEBUG("Incoming queue full for topic \"%s\".  "
                   "Discarding oldest message\n",
                    name_.c_str());
        }

        queue_full_ = true;
        dropped = true;
      }
      else
      {
        queue_full_ = false;
      }

      inbox_.push(MessageInfo(m, connection_header));
    }

    inbox_cond_.notify_all();
  }
  else
  {
    boost::mutex::scoped_lock lock(callbacks_mutex_);

    invokeCallback(buf, num_bytes, connection_header);
  }

  return dropped;
}

void Subscription::subscriptionThreadFunc()
{
  disableAllSignalsInThisThread();

  SubscriptionPtr self;

  // service the incoming message queue, invoking callbacks
  while(!dropped_ && !shutting_down_)
  {
    MessageInfo m;

    {
      boost::mutex::scoped_lock lock(inbox_mutex_);

      while(inbox_.empty() && !dropped_ && !shutting_down_)
      {
        inbox_cond_.wait(lock);
      }

      if (dropped_ || shutting_down_)
      {
        break;
      }


      if (inbox_.size() == 0)
      {
        ROS_INFO("incoming queue sem was posted; nothing there.");
        continue;
      }

      m = inbox_.front();
      inbox_.pop();
    }

    {
      boost::mutex::scoped_lock lock(callbacks_mutex_);

      if (!dropped_)
      {
        // Keep a shared pointer to ourselves so we don't get deleted while in a callback
        // Fixes the case of unsubscribing from within a callback
        if (!self)
        {
          self = shared_from_this();
        }

        invokeCallback(m.serialized_message_.buf, m.serialized_message_.num_bytes, m.connection_header_);
      }
    }
  }
}

bool Subscription::addFunctorMessagePair(AbstractFunctor* cb, Message* m)
{
  ROS_ASSERT(m);
  if (m->__getMD5Sum() != md5sum())
  {
    return false;
  }

  {
    boost::mutex::scoped_lock lock(callbacks_mutex_);

    CallbackInfoPtr info(new CallbackInfo);
    info->callback_ = cb;
    info->message_ = m;

    callbacks_.push_back(info);
  }

  return true;
}

void Subscription::invokeCallback(const boost::shared_array<uint8_t>& buffer, size_t num_bytes, const boost::shared_ptr<M_string>& connection_header)
{
  for (V_CallbackInfo::iterator cb = callbacks_.begin();
       cb != callbacks_.end(); ++cb)
  {
    const CallbackInfoPtr& info = *cb;

    info->message_->lock();
    info->message_->__serialized_length = num_bytes;
    info->message_->__connection_header = connection_header;
    info->message_->deserialize(buffer.get());

    info->callback_->call();

    info->message_->unlock();
  }
}

void Subscription::removeFunctorMessagePair(AbstractFunctor* cb)
{
  typedef std::vector<int> V_int;
  V_int to_delete;

  boost::mutex::scoped_lock cbs_lock(callbacks_mutex_);
  for (V_CallbackInfo::iterator it = callbacks_.begin();
       it != callbacks_.end(); ++it)
  {
    if (*(*it)->callback_ == *cb)
    {
      delete (*it)->callback_;
      to_delete.push_back(it - callbacks_.begin());
    }
  }

  V_int::iterator it = to_delete.begin();
  V_int::iterator end = to_delete.end();
  for (; it != end; ++it)
  {
    callbacks_.erase(callbacks_.begin() + *it);
  }
}
bool Subscription::updatesMessage(const void* _msg)
{
  bool found = false;
  boost::mutex::scoped_lock lock(callbacks_mutex_);

  for (V_CallbackInfo::iterator it = callbacks_.begin();
       !found && it != callbacks_.end(); ++it)
  {
    if ((*it)->message_ == _msg)
    {
      found = true;
      break;
    }
  }

  return found;
}

void Subscription::removeSubscriber(const SubscriberPtr& subscriber)
{
  boost::mutex::scoped_lock lock(subscribers_mutex_);

  V_Subscriber::iterator it = std::find(subscribers_.begin(), subscribers_.end(), subscriber);
  if (it != subscribers_.end())
  {
    subscribers_.erase(it);
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

void Subscription::setMaxQueue(int max_queue)
{
  {
    boost::mutex::scoped_lock lock(inbox_mutex_);
    this->max_queue_ = max_queue;
  }
}
