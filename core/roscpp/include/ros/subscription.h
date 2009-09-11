/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#ifndef ROSCPP_SUBSCRIPTION_H
#define ROSCPP_SUBSCRIPTION_H

#include <queue>
#include "ros/common.h"
#include "ros/header.h"
#include "XmlRpc.h"

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace ros
{

class Subscriber;
typedef boost::shared_ptr<Subscriber> SubscriberPtr;

/**
 * \brief Manages a subscription on a single topic.
 */
class Subscription : public boost::enable_shared_from_this<Subscription>
{
public:
  Subscription(const std::string &name, Message* m, AbstractFunctor *cb,
           bool threaded, int max_queue);
  virtual ~Subscription();

  /**
   * \brief Terminate all our Subscribers
   */
  void drop();
  /**
   * \brief Terminate all our Subscribers and join our callback thread if it exists
   */
  void shutdown();
  /**
   * \brief Handle a publisher update list received from the master. Creates/drops Subscribers based on
   * the list.  Never handles new self-subscriptions
   */
  bool pubUpdate(const std::vector<std::string> &pubs);
  /**
   * \brief Negotiates a connection with a publisher
   * \param xmlrpc_uri The XMLRPC URI to connect to to negotiate the connection
   */
  bool negotiateConnection(const std::string& xmlrpc_uri);
  /**
   * \brief Returns whether this Subscription has been dropped or not
   */
  bool isDropped() { return dropped_; }
  /**
   * \brief Adds a Functor/message to our list of callbacks/messages.  Used for multiple subscriptions to the
   * same topic
   */
  bool addFunctorMessagePair(AbstractFunctor* cb, Message* m);
  /**
   * \brief Remove a Functor/message from our list of callbacks/messages.Used for multiple subscriptions to the
   * same topic
   */
  void removeFunctorMessagePair(AbstractFunctor* cb);
  XmlRpc::XmlRpcValue getStats();
  void getInfo(XmlRpc::XmlRpcValue& info);

  typedef std::map<std::string, std::string> M_string;

  /**
   * \brief If we're threaded, queues up a message for deserialization and callback invokation by our thread.  Otherwise invokes the callbacks immediately
   */
  bool handleMessage(const boost::shared_array<uint8_t>& buffer, size_t num_bytes, const boost::shared_ptr<M_string>& connection_header);
  /**
   * \brief Deserializes a message and invokes all our callbacks
   */
  void invokeCallback(const boost::shared_array<uint8_t>& buffer, size_t num_bytes, const boost::shared_ptr<M_string>& connection_header);

  const std::string datatype();
  const std::string md5sum();

  /**
   * \brief Returns true if we update the message pointed to by _msg
   */
  bool updatesMessage(const void *_msg);
  /**
   * \brief Removes a subscriber from our list
   */
  void removeSubscriber(const SubscriberPtr& subscriber);

  const std::string& getName() { return name_; }
  int getMaxQueue() { return max_queue_; }
  void setMaxQueue(int max_queue);
  uint32_t getNumCallbacks() { return callbacks_.size(); }

private:
  Subscription(const Subscription &); // not copyable
  Subscription &operator =(const Subscription &); // nor assignable

  void dropAllConnections();

  void subscriptionThreadFunc();

  struct CallbackInfo
  {
    AbstractFunctor* callback_;
    Message* message_;
  };
  typedef boost::shared_ptr<CallbackInfo> CallbackInfoPtr;
  typedef std::vector<CallbackInfoPtr> V_CallbackInfo;

  std::string name_;
  std::string md5sum_;
  std::string datatype_;
  boost::mutex callbacks_mutex_;
  V_CallbackInfo callbacks_;

  bool dropped_;
  bool shutting_down_;
  boost::mutex shutdown_mutex_;

  // If threaded is true, then we're running a separate thread (identified
  // by callback_thread_) that pulls message from inbox, and invokes the
  // callback on each one.
  //
  // Otherwise, the callback is invoked in place when the message is
  // received, and none of this machinery is used.
  bool threaded_;
  int max_queue_;
  boost::thread callback_thread_;

  struct MessageInfo
  {
    MessageInfo()
    {}

    MessageInfo(const SerializedMessage& m, const boost::shared_ptr<M_string>& connection_header)
    : serialized_message_(m)
    , connection_header_(connection_header)
    {}

    SerializedMessage serialized_message_;
    boost::shared_ptr<M_string> connection_header_;
  };
  std::queue<MessageInfo> inbox_;
  boost::mutex inbox_mutex_;
  boost::condition_variable inbox_cond_;
  bool queue_full_;

  typedef std::vector<SubscriberPtr> V_Subscriber;
  V_Subscriber subscribers_;
  boost::mutex subscribers_mutex_;
};

}

#endif

