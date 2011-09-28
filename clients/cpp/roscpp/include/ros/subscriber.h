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

#ifndef ROSCPP_SUBSCRIBER_HANDLE_H
#define ROSCPP_SUBSCRIBER_HANDLE_H

#include "common.h"
#include "ros/forwards.h"
#include "ros/subscription_callback_helper.h"

namespace ros
{

/**
 * \brief Manages an subscription callback on a specific topic.
 *
 * A Subscriber should always be created through a call to NodeHandle::subscribe(), or copied from one
 * that was. Once all copies of a specific
 * Subscriber go out of scope, the subscription callback associated with that handle will stop
 * being called.  Once all Subscriber for a given topic go out of scope the topic will be unsubscribed.
 */
class ROSCPP_DECL Subscriber
{
public:
  Subscriber() {}
  Subscriber(const Subscriber& rhs);
  ~Subscriber();

  /**
   * \brief Unsubscribe the callback associated with this Subscriber
   *
   * This method usually does not need to be explicitly called, as automatic shutdown happens when
   * all copies of this Subscriber go out of scope
   *
   * This method overrides the automatic reference counted unsubscribe, and immediately
   * unsubscribes the callback associated with this Subscriber
   */
  void shutdown();

  std::string getTopic() const;

  /**
   * \brief Returns the number of publishers this subscriber is connected to
   */
  uint32_t getNumPublishers() const;

  operator void*() const { return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0; }

  bool operator<(const Subscriber& rhs) const
  {
    return impl_ < rhs.impl_;
  }

  bool operator==(const Subscriber& rhs) const
  {
    return impl_ == rhs.impl_;
  }

  bool operator!=(const Subscriber& rhs) const
  {
    return impl_ != rhs.impl_;
  }

private:

  Subscriber(const std::string& topic, const NodeHandle& node_handle, 
	     const SubscriptionCallbackHelperPtr& helper);

  class Impl
  {
  public:
    Impl();
    ~Impl();

    void unsubscribe();
    bool isValid() const;

    std::string topic_;
    NodeHandlePtr node_handle_;
    SubscriptionCallbackHelperPtr helper_;
    bool unsubscribed_;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;

  friend class NodeHandle;
  friend class NodeHandleBackingCollection;
};
typedef std::vector<Subscriber> V_Subscriber;

}

#endif // ROSCPP_PUBLISHER_HANDLE_H


