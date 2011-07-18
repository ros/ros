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

class ROSCPP_DECL SubscriberLink : public boost::enable_shared_from_this<SubscriberLink>
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

  const std::string& getTopic() const { return topic_; }
  const Stats &getStats() { return stats_; }
  const std::string &getDestinationCallerID() const { return destination_caller_id_; }
  int getConnectionID() const { return connection_id_; }

  /**
   * \brief Queue up a message for publication.  Throws out old messages if we've reached our Publication's max queue size
   */
  virtual void enqueueMessage(const SerializedMessage& m, bool nocopy, bool ser) = 0;

  virtual void drop() = 0;

  virtual std::string getTransportType() = 0;

  virtual bool isIntraprocess() { return false; }
  virtual void getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti) { ser = true; nocopy = false; }

  const std::string& getMD5Sum();
  const std::string& getDataType();
  const std::string& getMessageDefinition();

protected:
  bool verifyDatatype(const std::string &datatype);

  PublicationWPtr parent_;
  unsigned int connection_id_;
  std::string destination_caller_id_;
  Stats stats_;
  std::string topic_;
};

} // namespace ros

#endif // ROSCPP_SUBSCRIBER_LINK_H


