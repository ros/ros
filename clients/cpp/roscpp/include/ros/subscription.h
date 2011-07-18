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
#include "ros/forwards.h"
#include "ros/transport_hints.h"
#include "ros/xmlrpc_manager.h"
#include "XmlRpc.h"

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace ros
{

class PublisherLink;
typedef boost::shared_ptr<PublisherLink> PublisherLinkPtr;

class SubscriptionCallback;
typedef boost::shared_ptr<SubscriptionCallback> SubscriptionCallbackPtr;

class SubscriptionQueue;
typedef boost::shared_ptr<SubscriptionQueue> SubscriptionQueuePtr;

class MessageDeserializer;
typedef boost::shared_ptr<MessageDeserializer> MessageDeserializerPtr;

class SubscriptionCallbackHelper;
typedef boost::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

/**
 * \brief Manages a subscription on a single topic.
 */
class ROSCPP_DECL Subscription : public boost::enable_shared_from_this<Subscription>
{
public:
  Subscription(const std::string &name, const std::string& md5sum, const std::string& datatype, const TransportHints& transport_hints);
  virtual ~Subscription();

  /**
   * \brief Terminate all our PublisherLinks
   */
  void drop();
  /**
   * \brief Terminate all our PublisherLinks and join our callback thread if it exists
   */
  void shutdown();
  /**
   * \brief Handle a publisher update list received from the master. Creates/drops PublisherLinks based on
   * the list.  Never handles new self-subscriptions
   */
  bool pubUpdate(const std::vector<std::string> &pubs);
  /**
   * \brief Negotiates a connection with a publisher
   * \param xmlrpc_uri The XMLRPC URI to connect to to negotiate the connection
   */
  bool negotiateConnection(const std::string& xmlrpc_uri);

  void addLocalConnection(const PublicationPtr& pub);

  /**
   * \brief Returns whether this Subscription has been dropped or not
   */
  bool isDropped() { return dropped_; }
  XmlRpc::XmlRpcValue getStats();
  void getInfo(XmlRpc::XmlRpcValue& info);

  bool addCallback(const SubscriptionCallbackHelperPtr& helper, const std::string& md5sum, CallbackQueueInterface* queue, int32_t queue_size, const VoidConstPtr& tracked_object, bool allow_concurrent_callbacks);
  void removeCallback(const SubscriptionCallbackHelperPtr& helper);

  typedef std::map<std::string, std::string> M_string;

  /**
   * \brief Called to notify that a new message has arrived from a publisher.
   * Schedules the callback for invokation with the callback queue
   */
  uint32_t handleMessage(const SerializedMessage& m, bool ser, bool nocopy, const boost::shared_ptr<M_string>& connection_header, const PublisherLinkPtr& link);

  const std::string datatype();
  const std::string md5sum();

  /**
   * \brief Removes a subscriber from our list
   */
  void removePublisherLink(const PublisherLinkPtr& pub_link);

  const std::string& getName() const { return name_; }
  uint32_t getNumCallbacks() const { return callbacks_.size(); }
  uint32_t getNumPublishers();

  // We'll keep a list of these objects, representing in-progress XMLRPC 
  // connections to other nodes.
  class ROSCPP_DECL PendingConnection : public ASyncXMLRPCConnection
  {
    public:
      PendingConnection(XmlRpc::XmlRpcClient* client, TransportUDPPtr udp_transport, const SubscriptionWPtr& parent, const std::string& remote_uri)
      : client_(client)
      , udp_transport_(udp_transport)
      , parent_(parent)
      , remote_uri_(remote_uri)
      {}

      ~PendingConnection()
      {
        delete client_;
      }

      XmlRpc::XmlRpcClient* getClient() const { return client_; }
      TransportUDPPtr getUDPTransport() const { return udp_transport_; }

      virtual void addToDispatch(XmlRpc::XmlRpcDispatch* disp)
      {
        disp->addSource(client_, XmlRpc::XmlRpcDispatch::WritableEvent | XmlRpc::XmlRpcDispatch::Exception);
      }

      virtual void removeFromDispatch(XmlRpc::XmlRpcDispatch* disp)
      {
        disp->removeSource(client_);
      }

      virtual bool check()
      {
        SubscriptionPtr parent = parent_.lock();
        if (!parent)
        {
          return true;
        }

        XmlRpc::XmlRpcValue result;
        if (client_->executeCheckDone(result))
        {
          parent->pendingConnectionDone(boost::dynamic_pointer_cast<PendingConnection>(shared_from_this()), result);
          return true;
        }

        return false;
      }

      const std::string& getRemoteURI() { return remote_uri_; }

    private:
      XmlRpc::XmlRpcClient* client_;
      TransportUDPPtr udp_transport_;
      SubscriptionWPtr parent_;
      std::string remote_uri_;
  };
  typedef boost::shared_ptr<PendingConnection> PendingConnectionPtr;

  void pendingConnectionDone(const PendingConnectionPtr& pending_conn, XmlRpc::XmlRpcValue& result);

  void getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti);

  void headerReceived(const PublisherLinkPtr& link, const Header& h);

private:
  Subscription(const Subscription &); // not copyable
  Subscription &operator =(const Subscription &); // nor assignable

  void dropAllConnections();

  void addPublisherLink(const PublisherLinkPtr& link);

  struct CallbackInfo
  {
    CallbackQueueInterface* callback_queue_;

    // Only used if callback_queue_ is non-NULL (NodeHandle API)
    SubscriptionCallbackHelperPtr helper_;
    SubscriptionQueuePtr subscription_queue_;
    bool has_tracked_object_;
    VoidConstWPtr tracked_object_;
  };
  typedef boost::shared_ptr<CallbackInfo> CallbackInfoPtr;
  typedef std::vector<CallbackInfoPtr> V_CallbackInfo;

  std::string name_;
  boost::mutex md5sum_mutex_;
  std::string md5sum_;
  std::string datatype_;
  boost::mutex callbacks_mutex_;
  V_CallbackInfo callbacks_;
  uint32_t nonconst_callbacks_;

  bool dropped_;
  bool shutting_down_;
  boost::mutex shutdown_mutex_;

  typedef std::set<PendingConnectionPtr> S_PendingConnection;
  S_PendingConnection pending_connections_;
  boost::mutex pending_connections_mutex_;

  typedef std::vector<PublisherLinkPtr> V_PublisherLink;
  V_PublisherLink publisher_links_;
  boost::mutex publisher_links_mutex_;

  TransportHints transport_hints_;

  struct LatchInfo
  {
    SerializedMessage message;
    PublisherLinkPtr link;
    boost::shared_ptr<std::map<std::string, std::string> > connection_header;
    ros::Time receipt_time;
  };

  typedef std::map<PublisherLinkPtr, LatchInfo> M_PublisherLinkToLatchInfo;
  M_PublisherLinkToLatchInfo latched_messages_;

  typedef std::vector<std::pair<const std::type_info*, MessageDeserializerPtr> > V_TypeAndDeserializer;
  V_TypeAndDeserializer cached_deserializers_;
};

}

#endif

