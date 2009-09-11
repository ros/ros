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

#ifndef ROSCPP_SUBSCRIBER_LINK_H
#define ROSCPP_SUBSCRIBER_LINK_H

#include "ros/common.h"

#include <boost/thread/mutex.hpp>
#include <boost/shared_array.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <queue>

namespace ros
{
class Header;
class Message;
class Publication;
typedef boost::shared_ptr<Publication> PublicationPtr;
typedef boost::weak_ptr<Publication> PublicationWPtr;
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;

/**
 * \brief SubscriberLink handles broadcasting messages to a single subscriber on a single topic
 */
class SubscriberLink : public boost::enable_shared_from_this<SubscriberLink>
{
public:
  class Stats
  {
  public:
    uint64_t bytes_sent_, message_data_sent_, messages_sent_;
    Stats()
    : bytes_sent_(0), message_data_sent_(0), messages_sent_(0) { }
  };


  SubscriberLink();
  virtual ~SubscriberLink();

  //
  bool initialize(const ConnectionPtr& connection);
  bool handleHeader(const Header& header);

  /**
   * \brief Publish a message directly to our subscriber.  Useful for publication connection callbacks
   * to publish directly to the new subscriber and no-one else
   */
  bool publish(const Message& m);
  /**
   * \brief Queue up a message for publication.  Throws out old messages if we've reached our Publication's max queue size
   */
  void enqueueMessage(const SerializedMessage& m);

  const ConnectionPtr& getConnection() { return connection_; }

  const std::string& getTopic() const { return topic_; }
  std::string getTransportType();
  const Stats &getStats() { return stats_; }
  inline const std::string &getDestinationCallerID() const { return destination_caller_id_; }
  inline int getConnectionID() const { return connection_id_; }

private:
  bool verifyDatatype(const std::string &datatype);
  void onConnectionDropped(const ConnectionPtr& conn);

  void onHeaderWritten(const ConnectionPtr& conn);
  void onMessageWritten(const ConnectionPtr& conn);
  void startMessageWrite(bool immediate_write);

  bool writing_message_;
  bool header_written_;

  ConnectionPtr connection_;
  PublicationWPtr parent_;

  std::queue<SerializedMessage> outbox_;
  boost::mutex outbox_mutex_;
  bool queue_full_;
  unsigned int connection_id_;
  std::string destination_caller_id_;

  Stats stats_;

  std::string topic_;
};

} // namespace ros

#endif // ROSCPP_SUBSCRIBER_LINK_H


