/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/callback_queue.h"
#include "ros/assert.h"

namespace ros
{

CallbackQueue::CallbackQueue(bool enabled)
: enabled_(enabled)
{
}

CallbackQueue::~CallbackQueue()
{
  disable();
}

void CallbackQueue::enable()
{
  boost::mutex::scoped_lock lock(mutex_);
  enabled_ = true;

  condition_.notify_all();
}

void CallbackQueue::disable()
{
  boost::mutex::scoped_lock lock(mutex_);
  enabled_ = false;

  condition_.notify_all();
}

void CallbackQueue::clear()
{
  boost::mutex::scoped_lock lock(mutex_);

  callbacks_.clear();
}

bool CallbackQueue::isEmpty()
{
  boost::mutex::scoped_lock lock(mutex_);

  return callbacks_.empty();
}

bool CallbackQueue::isEnabled()
{
  boost::mutex::scoped_lock lock(mutex_);

  return enabled_;
}

void CallbackQueue::setupTLS()
{
  if (!calling_in_this_thread_.get())
  {
    calling_in_this_thread_.reset(new bool);
    *calling_in_this_thread_ = false;
  }
}

void CallbackQueue::addCallback(const CallbackInterfacePtr& callback, uint64_t removal_id)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (!enabled_)
  {
    return;
  }

  setupTLS();

  CallbackInfo info;
  info.callback = callback;
  info.removal_id = removal_id;
  callbacks_.push_back(info);

  condition_.notify_one();
}

void CallbackQueue::removeByID(uint64_t removal_id)
{
  setupTLS();

  // If we're being called from inside a callOne()/callAvailable() method,
  // remove the read reference for this thread.  We'll add it back later.
  if (*calling_in_this_thread_)
  {
    calling_rw_mutex_.readUnlock();
  }

  {
    RWMutex::ScopedWriteLock rw_lock(calling_rw_mutex_);
    boost::mutex::scoped_lock lock(mutex_);
    L_CallbackInfo::iterator it = callbacks_.begin();
    L_CallbackInfo::iterator end = callbacks_.end();
    for (; it != end;)
    {
      CallbackInfo& info = *it;
      if (info.removal_id == removal_id)
      {
        it = callbacks_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  if (*calling_in_this_thread_)
  {
    calling_rw_mutex_.readLock();
  }
}

void CallbackQueue::callOne(ros::WallDuration timeout)
{
  setupTLS();

  CallbackInfo cb_info;

  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!enabled_)
    {
      return;
    }

    if (callbacks_.empty() || !enabled_)
    {
      condition_.timed_wait(lock, boost::posix_time::milliseconds(timeout.toSec() * 1000.0f));

      if (callbacks_.empty())
      {
        return;
      }
    }

    L_CallbackInfo::iterator it = callbacks_.begin();
    L_CallbackInfo::iterator end = callbacks_.end();
    for (; it != end;)
    {
      CallbackInfo& info = *it;
      if (info.callback->ready())
      {
        cb_info = info;
        it = callbacks_.erase(it);
        break;
      }

      ++it;
    }

    if (!cb_info.callback)
    {
      return;
    }
  }

  {
    RWMutex::ScopedReadLock rw_lock(calling_rw_mutex_);
    *calling_in_this_thread_ = true;

    CallbackInterface::CallResult result = cb_info.callback->call();

    *calling_in_this_thread_ = false;

    if (result == CallbackInterface::TryAgain)
    {
      boost::mutex::scoped_lock lock(mutex_);
      callbacks_.push_front(cb_info);
    }
  }
}

void CallbackQueue::callAvailable(ros::WallDuration timeout)
{
  setupTLS();

  L_CallbackInfo local_callbacks;

  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!enabled_)
    {
      return;
    }

    if (callbacks_.empty())
    {
      condition_.timed_wait(lock, boost::posix_time::milliseconds(timeout.toSec() * 1000.0f));

      if (callbacks_.empty() || !enabled_)
      {
        return;
      }
    }

    local_callbacks.swap(callbacks_);
  }

  {
    RWMutex::ScopedReadLock rw_lock(calling_rw_mutex_);
    *calling_in_this_thread_ = true;

    L_CallbackInfo::iterator it = local_callbacks.begin();
    L_CallbackInfo::iterator end = local_callbacks.end();
    for (; it != end;)
    {
      CallbackInfo& info = *it;
      CallbackInterfacePtr& cb = info.callback;

      CallbackInterface::CallResult result = cb->call();
      if (result == CallbackInterface::Success)
      {
        it = local_callbacks.erase(it);
      }
      else if (result == CallbackInterface::Invalid)
      {
        it = local_callbacks.erase(it);
      }
      else if (result == CallbackInterface::TryAgain)
      {
        ++it;
      }
    }

    *calling_in_this_thread_ = false;

    // If we had some callbacks that returned TryAgain, push them to the front of the shared queue
    if (!local_callbacks.empty())
    {
      boost::mutex::scoped_lock lock(mutex_);
      callbacks_.insert(callbacks_.begin(), local_callbacks.begin(), local_callbacks.end());
    }
  }
}

}
