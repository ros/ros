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

#ifndef ROSCPP_TRANSPORT_SUBSCRIBER_LINK_H
#define ROSCPP_TRANSPORT_SUBSCRIBER_LINK_H
#include "common.h"
#include "subscriber_link.h"

#include <boost/signals/connection.hpp>

namespace ros
{

/**
 * \brief SubscriberLink handles broadcasting messages to a single subscriber on a single topic
 */
class ROSCPP_DECL TransportSubscriberLink : public SubscriberLink
{
public:
  TransportSubscriberLink();
  virtual ~TransportSubscriberLink();

  //
  bool initialize(const ConnectionPtr& connection);
  bool handleHeader(const Header& header);

  const ConnectionPtr& getConnection() { return connection_; }

  virtual void enqueueMessage(const SerializedMessage& m, bool ser, bool nocopy);
  virtual void drop();
  virtual std::string getTransportType();

private:
  void onConnectionDropped(const ConnectionPtr& conn);

  void onHeaderWritten(const ConnectionPtr& conn);
  void onMessageWritten(const ConnectionPtr& conn);
  void startMessageWrite(bool immediate_write);

  bool writing_message_;
  bool header_written_;

  ConnectionPtr connection_;
  boost::signals::connection dropped_conn_;

  std::queue<SerializedMessage> outbox_;
  boost::mutex outbox_mutex_;
  bool queue_full_;
};
typedef boost::shared_ptr<TransportSubscriberLink> TransportSubscriberLinkPtr;

} // namespace ros

#endif // ROSCPP_TRANSPORT_SUBSCRIBER_LINK_H
