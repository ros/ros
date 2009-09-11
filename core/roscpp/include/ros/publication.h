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

#include "XmlRpc.h"

#include <boost/thread/mutex.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <vector>

namespace ros
{

class Publisher;
typedef boost::shared_ptr<Publisher> PublisherPtr;
typedef std::vector<PublisherPtr> V_Publisher;

typedef boost::function<void (const PublisherPtr&)> SubscriptionConnectionCallback;

/**
 * \brief A Publication manages an advertised topic
 */
class Publication
{
public:
  Publication(const std::string &name, const std::string &_original_name,
            const std::string &datatype,
            const std::string &_md5sum,
            const SubscriptionConnectionCallback& connect_cb,
            const SubscriptionConnectionCallback& disconnect_cb,
            size_t max_queue);

  ~Publication();

  /**
   * \brief queues an outgoing message into each of the publishers, so that it gets sent to every subscriber
   */
  bool enqueueMessage(boost::shared_array<uint8_t> buf, size_t num_bytes);
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
  bool hasSubscribers() { return !publishers_.empty(); }
  /**
   * \brief Returns the number of subscribers this publication has
   */
  int getNumSubscribers() { return (int)publishers_.size(); }

  /**
   * \brief Returns the name of the topic this publication broadcasts to
   */
  const std::string& getName() { return name_; }
  /**
   * \brief Returns the original (non-mapped) name of the topic this publication broadcasts to
   */
  const std::string& getOriginalName() { return original_name_; }
  /**
   * \brief Returns the data type of the message published by this publication
   */
  const std::string& getDataType() { return datatype_; }
  /**
   * \brief Returns the md5sum of the message published by this publication
   */
  const std::string& getMD5Sum() { return md5sum_; }
  /**
   * \brief Returns the sequence number
   */
  uint32_t getSequence() { return seq_; }

  /**
   * \brief Adds a publisher to our list
   */
  void addPublisher(const PublisherPtr& publisher);
  /**
   * \brief Removes a publisher from our list (deleting it if it's the last reference)
   */
  void removePublisher(const PublisherPtr& publisher);

  /**
   * \brief Drop this publication.  Disconnects all publishers.
   */
  void drop();
  /**
   * \brief Returns if this publication is valid or not
   */
  bool isDropped() { return dropped_; }

  void incrementSequence() { ++seq_; }

private:
  void dropAllConnections();

  /**
   * \brief Called when a new peer has connected. Calls the connection callback
   */
  void peerConnect(const PublisherPtr& publisher);
  /**
   * \brief Called when a peer has disconnected. Calls the disconnection callback
   */
  void peerDisconnect(const PublisherPtr& publisher);

  std::string name_;
  std::string original_name_;
  std::string datatype_;
  std::string md5sum_;
  SubscriptionConnectionCallback sub_connect_cb_;
  SubscriptionConnectionCallback sub_disconnect_cb_;
  size_t max_queue_;
  uint32_t seq_;

  V_Publisher publishers_;
  // We use a recursive mutex here for the rare case that a publish call causes another one (like in the case of a rosconsole call)
  boost::mutex publishers_mutex_;

  bool dropped_;
};

}

#endif // ROSCPP_PUBLICATION_H
