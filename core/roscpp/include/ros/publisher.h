/*
 * Copyright (C) 2009, Willow Garage, Inc.
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

#ifndef ROSCPP_PUBLISHER_HANDLE_H
#define ROSCPP_PUBLISHER_HANDLE_H

#include "ros/forwards.h"
#include "ros/message.h"

namespace ros
{

/**
 * \brief Manages an advertisement on a specific topic.
 *
 * A Publisher should always be created through a call to NodeHandle::advertise(), or copied from one
 * that was. Once all copies of a specific
 * Publisher go out of scope, any subscriber status callbacks associated with that handle will stop
 * being called.  Once all Publishers for a given topic go out of scope the topic will be unadvertised.
 */
class Publisher
{
public:
  Publisher() {}
  Publisher(const Publisher& rhs);
  ~Publisher();

  /**
   * \brief Publish a message on the topic associated with this Publisher.
   *
   * This version of publish will allow fast intra-process message-passing in the future,
   * so you may not mutate the message after it has been passed in here (since it will be
   * passed directly into a callback function)
   *
   */
  void publish(const Message::ConstPtr& message) const;
  /**
   * \brief Publish a message on the topic associated with this Publisher.
   */
  void publish(const Message& message) const;

  /**
   * \brief Shutdown the advertisement associated with this Publisher
   *
   * This method usually does not need to be explicitly called, as automatic shutdown happens when
   * all copies of this Publisher go out of scope
   *
   * This method overrides the automatic reference counted unadvertise, and does so immediately.
   * \note Note that if multiple advertisements were made through NodeHandle::advertise(), this will
   * only remove the one associated with this Publisher
   */
  void shutdown();

  /**
   * \brief Returns the topic that this Publisher will publish on.
   */
  std::string getTopic() const;

  /**
   * \brief Returns the number of subscribers that are currently connected to this Publisher
   */
  uint32_t getNumSubscribers() const;

  operator void*() const { return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0; }

  bool operator<(const Publisher& rhs) const
  {
    return impl_ < rhs.impl_;
  }

  bool operator==(const Publisher& rhs) const
  {
    return impl_ == rhs.impl_;
  }

  bool operator!=(const Publisher& rhs) const
  {
    return impl_ != rhs.impl_;
  }

private:
  Publisher(const std::string& topic, const NodeHandle& node_handle, const SubscriberCallbacksPtr& callbacks);

  class Impl
  {
  public:
    Impl();
    ~Impl();

    void unadvertise();
    bool isValid() const;

    std::string topic_;
    NodeHandlePtr node_handle_;
    SubscriberCallbacksPtr callbacks_;
    bool unadvertised_;
    double constructed_;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;

  friend class NodeHandle;
  friend class NodeHandleBackingCollection;
};
typedef std::vector<Publisher> V_Publisher;

}

#endif // ROSCPP_PUBLISHER_HANDLE_H

