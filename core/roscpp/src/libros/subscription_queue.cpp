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


#include "ros/subscription_queue.h"
#include "ros/message_deserializer.h"

namespace ros
{

SubscriptionQueue::SubscriptionQueue(const std::string& topic, int32_t queue_size)
: topic_(topic)
, size_(queue_size)
, full_(false)
, id_counter_(0)
, queue_size_(0)
{}

uint64_t SubscriptionQueue::push(const SubscriptionMessageHelperPtr& helper, const MessageDeserializerPtr& deserializer, bool has_tracked_object, const VoidWPtr& tracked_object)
{
  boost::mutex::scoped_lock lock(queue_mutex_);

  if(fullNoLock())
  {
    queue_.pop_front();
    --queue_size_;

    if (!full_)
    {
      ROS_DEBUG("Incoming queue full for topic \"%s\".  Discarding oldest message (current queue size [%d])", topic_.c_str(), (int)queue_.size());
    }

    full_ = true;
  }
  else
  {
    full_ = false;
  }

  uint64_t count = id_counter_++;

  Item i;
  i.helper = helper;
  i.deserializer = deserializer;
  i.has_tracked_object = has_tracked_object;
  i.tracked_object = tracked_object;
  i.id = count;
  queue_.push_back(i);
  ++queue_size_;

  return count;
}

void SubscriptionQueue::remove(uint64_t id)
{
  boost::mutex::scoped_lock lock(queue_mutex_);
  if (!queue_.empty())
  {
    if (id < queue_.front().id)
    {
      return;
    }
  }

  L_Item::iterator it = queue_.begin();
  L_Item::iterator end = queue_.end();
  for (; it != end; ++it)
  {
    const Item& i = *it;
    if (i.id == id)
    {
      queue_.erase(it);
      return;
    }
  }
}

void SubscriptionQueue::clear()
{
  boost::recursive_mutex::scoped_lock cb_lock(callback_mutex_);
  boost::mutex::scoped_lock queue_lock(queue_mutex_);

  queue_.clear();
  queue_size_ = 0;
}

CallbackInterface::CallResult SubscriptionQueue::call(uint64_t id)
{
  // The callback may result in our own destruction.  Therefore, we may need to keep a reference to ourselves
  // that outlasts the scoped_try_lock
  boost::shared_ptr<SubscriptionQueue> self;
  boost::recursive_mutex::scoped_try_lock lock(callback_mutex_);
  if (!lock.owns_lock())
  {
    return CallbackInterface::TryAgain;
  }

  VoidPtr tracker;
  Item i;

  {
    boost::mutex::scoped_lock lock(queue_mutex_);

    if (queue_.empty())
    {
      return CallbackInterface::Invalid;
    }

    i = queue_.front();

    if (id < i.id)
    {
      return CallbackInterface::Invalid;
    }

    if (id > i.id)
    {
      return CallbackInterface::TryAgain;
    }

    if (queue_.empty())
    {
      return CallbackInterface::Invalid;
    }

    if (i.has_tracked_object)
    {
      tracker = i.tracked_object.lock();

      if (!tracker)
      {
        return CallbackInterface::Invalid;
      }
    }

    queue_.pop_front();
    --queue_size_;
  }

  MessagePtr msg = i.deserializer->deserialize();

  // msg can be null here if deserialization failed
  if (msg)
  {
    self = shared_from_this();
    i.helper->call(msg);
  }

  return CallbackInterface::Success;
}

bool SubscriptionQueue::ready(uint64_t id)
{
  boost::mutex::scoped_lock lock(queue_mutex_);
  if (queue_.empty())
  {
    return true;
  }

  return id <= queue_.front().id;
}

bool SubscriptionQueue::full()
{
  boost::mutex::scoped_lock lock(queue_mutex_);
  return fullNoLock();
}

bool SubscriptionQueue::fullNoLock()
{
  return (size_ > 0) && (queue_size_ >= (uint32_t)size_);
}

}

