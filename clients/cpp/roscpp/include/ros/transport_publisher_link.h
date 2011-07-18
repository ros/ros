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

#ifndef ROSCPP_TRANSPORT_PUBLISHER_LINK_H
#define ROSCPP_TRANSPORT_PUBLISHER_LINK_H

#include "common.h"
#include "publisher_link.h"
#include "connection.h"

namespace ros
{
class Header;
class Message;
class Subscription;
typedef boost::shared_ptr<Subscription> SubscriptionPtr;
typedef boost::weak_ptr<Subscription> SubscriptionWPtr;
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;

class WallTimerEvent;

/**
 * \brief Handles a connection to a single publisher on a given topic.  Receives messages from a publisher
 * and hands them off to its parent Subscription
 */
class ROSCPP_DECL TransportPublisherLink : public PublisherLink
{
public:
  TransportPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const TransportHints& transport_hints);
  virtual ~TransportPublisherLink();

  //
  bool initialize(const ConnectionPtr& connection);

  const ConnectionPtr& getConnection() { return connection_; }

  virtual std::string getTransportType();
  virtual void drop();

private:
  void onConnectionDropped(const ConnectionPtr& conn, Connection::DropReason reason);
  bool onHeaderReceived(const ConnectionPtr& conn, const Header& header);

  /**
   * \brief Handles handing off a received message to the subscription, where it will be deserialized and called back
   */
  virtual void handleMessage(const SerializedMessage& m, bool ser, bool nocopy);

  void onHeaderWritten(const ConnectionPtr& conn);
  void onMessageLength(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);
  void onMessage(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);

  void onRetryTimer(const ros::WallTimerEvent&);

  ConnectionPtr connection_;

  int32_t retry_timer_handle_;
  bool needs_retry_;
  WallDuration retry_period_;
  WallTime next_retry_;
  bool dropping_;
};
typedef boost::shared_ptr<TransportPublisherLink> TransportPublisherLinkPtr;

} // namespace ros

#endif // ROSCPP_TRANSPORT_PUBLISHER_LINK_H



