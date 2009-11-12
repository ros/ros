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
, queue_counter_(0)
{}

uint64_t SubscriptionQueue::push(const SubscriptionMessageHelperPtr& helper, const MessageDeserializerPtr& deserializer, bool has_tracked_object, const VoidWPtr& tracked_object)
{
  boost::mutex::scoped_lock lock(queue_mutex_);

  if(full())
  {
    queue_.pop();
    ++queue_counter_;

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

  Item i;
  i.helper = helper;
  i.deserializer = deserializer;
  i.has_tracked_object = has_tracked_object;
  i.tracked_object = tracked_object;
  queue_.push(i);

  uint64_t count = id_counter_++;
  return count;
}

void SubscriptionQueue::clear()
{
  boost::recursive_mutex::scoped_lock cb_lock(callback_mutex_);
  boost::mutex::scoped_lock queue_lock(queue_mutex_);

  while (!queue_.empty())
  {
    queue_.pop();
    ++queue_counter_;
  }

  ROS_ASSERT(queue_counter_ == id_counter_);
}

CallbackInterface::CallResult SubscriptionQueue::call(uint64_t id)
{
  if (id > queue_counter_)
  {
    return CallbackInterface::TryAgain;
  }

  boost::recursive_mutex::scoped_try_lock lock(callback_mutex_);
  if (!lock.owns_lock())
  {
    return CallbackInterface::TryAgain;
  }

  VoidPtr tracker;
  Item i;

  {
    boost::mutex::scoped_lock lock(queue_mutex_);

    if (id < queue_counter_)
    {
      return CallbackInterface::Invalid;
    }

    if (id > queue_counter_)
    {
      return CallbackInterface::TryAgain;
    }

    if (queue_.empty())
    {
      return CallbackInterface::Invalid;
    }

    i = queue_.front();

    if (i.has_tracked_object)
    {
      tracker = i.tracked_object.lock();

      if (!tracker)
      {
        return CallbackInterface::Invalid;
      }
    }

    queue_.pop();
    ++queue_counter_;
  }

  MessagePtr msg = i.deserializer->deserialize();
  i.helper->call(msg);

  return CallbackInterface::Success;
}

bool SubscriptionQueue::ready(uint64_t id)
{
  return id <= queue_counter_;
}

bool SubscriptionQueue::full()
{
  return (size_ > 0) && (queue_.size() >= (uint32_t)size_);
}

}

