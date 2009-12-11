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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#ifndef ROSCPP_SUBSCRIPTION_QUEUE_H
#define ROSCPP_SUBSCRIPTION_QUEUE_H

#include "forwards.h"

#include "subscription_message_helper.h"
#include "callback_queue_interface.h"

#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/mutex.hpp>
#include <list>

namespace ros
{

class MessageDeserializer;
typedef boost::shared_ptr<MessageDeserializer> MessageDeserializerPtr;

class SubscriptionQueue
{
private:
  struct Item
  {
    SubscriptionMessageHelperPtr helper;
    MessageDeserializerPtr deserializer;

    bool has_tracked_object;
    VoidWPtr tracked_object;

    uint64_t id;
  };
  typedef std::list<Item> L_Item;

public:
  SubscriptionQueue(const std::string& topic, int32_t queue_size);
  uint64_t push(const SubscriptionMessageHelperPtr& helper, const MessageDeserializerPtr& deserializer, bool has_tracked_object, const VoidWPtr& tracked_object);
  void clear();
  CallbackInterface::CallResult call(uint64_t id);
  bool ready(uint64_t id);
  bool full();
  void remove(uint64_t id);

private:
  bool fullNoLock();
  std::string topic_;
  int32_t size_;
  bool full_;
  uint64_t id_counter_;

  boost::mutex queue_mutex_;
  L_Item queue_;
  uint32_t queue_size_;

  boost::recursive_mutex callback_mutex_;
};

}

#endif // ROSCPP_SUBSCRIPTION_QUEUE_H
