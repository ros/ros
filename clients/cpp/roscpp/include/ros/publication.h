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

#ifndef ROSCPP_PUBLICATION_H
#define ROSCPP_PUBLICATION_H

#include "ros/forwards.h"
#include "ros/advertise_options.h"
#include "common.h"
#include "XmlRpc.h"

#include <boost/thread/mutex.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <vector>

namespace ros
{

class SubscriberLink;
typedef boost::shared_ptr<SubscriberLink> SubscriberLinkPtr;
typedef std::vector<SubscriberLinkPtr> V_SubscriberLink;

/**
 * \brief A Publication manages an advertised topic
 */
class ROSCPP_DECL Publication
{
public:
  Publication(const std::string &name,
            const std::string& datatype,
            const std::string& _md5sum,
            const std::string& message_definition,
            size_t max_queue,
            bool latch,
            bool has_header);

  ~Publication();

  void addCallbacks(const SubscriberCallbacksPtr& callbacks);
  void removeCallbacks(const SubscriberCallbacksPtr& callbacks);

  /**
   * \brief queues an outgoing message into each of the publishers, so that it gets sent to every subscriber
   */
  bool enqueueMessage(const SerializedMessage& m);
  /**
   * \brief returns the max queue size of this publication
   */
  inline size_t getMaxQueue() { return max_queue_; }
  /**
   * \brief Get the accumulated stats for this publication
   */
  XmlRpc::XmlRpcValue getStats();
  /**
   * \brief Get the accumulated info for this publication
   */
  void getInfo(XmlRpc::XmlRpcValue& info);

  /**
   * \brief Returns whether or not this publication has any subscribers
   */
  bool hasSubscribers();
  /**
   * \brief Returns the number of subscribers this publication has
   */
  uint32_t getNumSubscribers();

  void getPublishTypes(bool& serialize, bool& nocopy, const std::type_info& ti);

  /**
   * \brief Returns the name of the topic this publication broadcasts to
   */
  const std::string& getName() const { return name_; }
  /**
   * \brief Returns the data type of the message published by this publication
   */
  const std::string& getDataType() const { return datatype_; }
  /**
   * \brief Returns the md5sum of the message published by this publication
   */
  const std::string& getMD5Sum() const { return md5sum_; }
  /**
   * \brief Returns the full definition of the message published by this publication
   */
  const std::string& getMessageDefinition() const { return message_definition_; }
  /**
   * \brief Returns the sequence number
   */
  uint32_t getSequence() { return seq_; }

  bool isLatched() { return latch_; }

  /**
   * \brief Adds a publisher to our list
   */
  void addSubscriberLink(const SubscriberLinkPtr& sub_link);
  /**
   * \brief Removes a publisher from our list (deleting it if it's the last reference)
   */
  void removeSubscriberLink(const SubscriberLinkPtr& sub_link);

  /**
   * \brief Drop this publication.  Disconnects all publishers.
   */
  void drop();
  /**
   * \brief Returns if this publication is valid or not
   */
  bool isDropped() { return dropped_; }

  uint32_t incrementSequence();

  size_t getNumCallbacks();

  bool isLatching() { return latch_; }

  void publish(SerializedMessage& m);
  void processPublishQueue();

  bool validateHeader(const Header& h, std::string& error_msg);

private:
  void dropAllConnections();

  /**
   * \brief Called when a new peer has connected. Calls the connection callback
   */
  void peerConnect(const SubscriberLinkPtr& sub_link);
  /**
   * \brief Called when a peer has disconnected. Calls the disconnection callback
   */
  void peerDisconnect(const SubscriberLinkPtr& sub_link);

  std::string name_;
  std::string datatype_;
  std::string md5sum_;
  std::string message_definition_;
  size_t max_queue_;
  uint32_t seq_;
  boost::mutex seq_mutex_;

  typedef std::vector<SubscriberCallbacksPtr> V_Callback;
  V_Callback callbacks_;
  boost::mutex callbacks_mutex_;

  V_SubscriberLink subscriber_links_;
  // We use a recursive mutex here for the rare case that a publish call causes another one (like in the case of a rosconsole call)
  boost::mutex subscriber_links_mutex_;

  bool dropped_;

  bool latch_;
  bool has_header_;
  SerializedMessage last_message_;

  uint32_t intraprocess_subscriber_count_;

  typedef std::vector<SerializedMessage> V_SerializedMessage;
  V_SerializedMessage publish_queue_;
  boost::mutex publish_queue_mutex_;
};

}

#endif // ROSCPP_PUBLICATION_H
