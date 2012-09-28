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

#ifndef ROSCPP_TIMER_MANAGER_H
#define ROSCPP_TIMER_MANAGER_H

#include "ros/forwards.h"
#include "ros/time.h"
#include "ros/file_log.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "ros/assert.h"
#include "ros/callback_queue_interface.h"

#include <vector>
#include <list>

namespace ros
{

template<class T, class D, class E>
class TimerManager
{
private:
  struct TimerInfo
  {
    int32_t handle;
    D period;

    boost::function<void(const E&)> callback;
    CallbackQueueInterface* callback_queue;

    WallDuration last_cb_duration;

    T last_expected;
    T next_expected;

    T last_real;

    bool removed;

    VoidConstWPtr tracked_object;
    bool has_tracked_object;

    // TODO: atomicize
    boost::mutex waiting_mutex;
    uint32_t waiting_callbacks;

    bool oneshot;

    // debugging info
    uint32_t total_calls;
  };
  typedef boost::shared_ptr<TimerInfo> TimerInfoPtr;
  typedef boost::weak_ptr<TimerInfo> TimerInfoWPtr;
  typedef std::vector<TimerInfoPtr> V_TimerInfo;

  typedef std::list<int32_t> L_int32;

public:
  TimerManager();
  ~TimerManager();

  int32_t add(const D& period, const boost::function<void(const E&)>& callback, CallbackQueueInterface* callback_queue, const VoidConstPtr& tracked_object, bool oneshot);
  void remove(int32_t handle);

  bool hasPending(int32_t handle);
  void setPeriod(int32_t handle, const D& period);

  static TimerManager& global()
  {
    static TimerManager<T, D, E> global;
    return global;
  }

private:
  void threadFunc();

  bool waitingCompare(int32_t lhs, int32_t rhs);
  TimerInfoPtr findTimer(int32_t handle);
  void schedule(const TimerInfoPtr& info);
  void updateNext(const TimerInfoPtr& info, const T& current_time);

  V_TimerInfo timers_;
  boost::mutex timers_mutex_;
  boost::condition_variable timers_cond_;
  volatile bool new_timer_;

  boost::mutex waiting_mutex_;
  L_int32 waiting_;

  uint32_t id_counter_;
  boost::mutex id_mutex_;

  bool thread_started_;

  boost::thread thread_;

  bool quit_;

  class TimerQueueCallback : public CallbackInterface
  {
  public:
    TimerQueueCallback(TimerManager<T, D, E>* parent, const TimerInfoPtr& info, T last_expected, T last_real, T current_expected)
    : parent_(parent)
    , info_(info)
    , last_expected_(last_expected)
    , last_real_(last_real)
    , current_expected_(current_expected)
    , called_(false)
    {
      boost::mutex::scoped_lock lock(info->waiting_mutex);
      ++info->waiting_callbacks;
    }

    ~TimerQueueCallback()
    {
      TimerInfoPtr info = info_.lock();
      if (info)
      {
        boost::mutex::scoped_lock lock(info->waiting_mutex);
        --info->waiting_callbacks;
      }
    }

    CallResult call()
    {
      TimerInfoPtr info = info_.lock();
      if (!info)
      {
        return Invalid;
      }

      {
        ++info->total_calls;
        called_ = true;

        VoidConstPtr tracked;
        if (info->has_tracked_object)
        {
          tracked = info->tracked_object.lock();
          if (!tracked)
          {
            return Invalid;
          }
        }

        E event;
        event.last_expected = last_expected_;
        event.last_real = last_real_;
        event.current_expected = current_expected_;
        event.current_real = T::now();
        event.profile.last_duration = info->last_cb_duration;

        WallTime cb_start = WallTime::now();
        info->callback(event);
        WallTime cb_end = WallTime::now();
        info->last_cb_duration = cb_end - cb_start;

        info->last_real = event.current_real;

        parent_->schedule(info);
      }

      return Success;
    }

  private:
    TimerManager<T, D, E>* parent_;
    TimerInfoWPtr info_;
    T last_expected_;
    T last_real_;
    T current_expected_;

    bool called_;
  };
};

template<class T, class D, class E>
TimerManager<T, D, E>::TimerManager() :
  new_timer_(false), id_counter_(0), thread_started_(false), quit_(false)
{

}

template<class T, class D, class E>
TimerManager<T, D, E>::~TimerManager()
{
  quit_ = true;
  {
    boost::mutex::scoped_lock lock(timers_mutex_);
    timers_cond_.notify_all();
  }
  if (thread_started_)
  {
    thread_.join();
  }
}

template<class T, class D, class E>
bool TimerManager<T, D, E>::waitingCompare(int32_t lhs, int32_t rhs)
{
  TimerInfoPtr infol = findTimer(lhs);
  TimerInfoPtr infor = findTimer(rhs);
  if (!infol || !infor)
  {
    return infol < infor;
  }

  return infol->next_expected < infor->next_expected;
}

template<class T, class D, class E>
typename TimerManager<T, D, E>::TimerInfoPtr TimerManager<T, D, E>::findTimer(int32_t handle)
{
  typename V_TimerInfo::iterator it = timers_.begin();
  typename V_TimerInfo::iterator end = timers_.end();
  for (; it != end; ++it)
  {
    if ((*it)->handle == handle)
    {
      return *it;
    }
  }

  return TimerInfoPtr();
}

template<class T, class D, class E>
bool TimerManager<T, D, E>::hasPending(int32_t handle)
{
  boost::mutex::scoped_lock lock(timers_mutex_);
  TimerInfoPtr info = findTimer(handle);

  if (!info)
  {
    return false;
  }

  if (info->has_tracked_object)
  {
    VoidConstPtr tracked = info->tracked_object.lock();
    if (!tracked)
    {
      return false;
    }
  }

  boost::mutex::scoped_lock lock2(info->waiting_mutex);
  return info->next_expected <= T::now() || info->waiting_callbacks != 0;
}

template<class T, class D, class E>
int32_t TimerManager<T, D, E>::add(const D& period, const boost::function<void(const E&)>& callback, CallbackQueueInterface* callback_queue,
                                   const VoidConstPtr& tracked_object, bool oneshot)
{
  TimerInfoPtr info(new TimerInfo);
  info->period = period;
  info->callback = callback;
  info->callback_queue = callback_queue;
  info->last_expected = T::now();
  info->next_expected = info->last_expected + period;
  info->removed = false;
  info->has_tracked_object = false;
  info->waiting_callbacks = 0;
  info->total_calls = 0;
  info->oneshot = oneshot;
  if (tracked_object)
  {
    info->tracked_object = tracked_object;
    info->has_tracked_object = true;
  }

  {
    boost::mutex::scoped_lock lock(id_mutex_);
    info->handle = id_counter_++;
  }

  {
    boost::mutex::scoped_lock lock(timers_mutex_);
    timers_.push_back(info);

    if (!thread_started_)
    {
      thread_ = boost::thread(boost::bind(&TimerManager::threadFunc, this));
      thread_started_ = true;
    }

    {
      boost::mutex::scoped_lock lock(waiting_mutex_);
      waiting_.push_back(info->handle);
      waiting_.sort(boost::bind(&TimerManager::waitingCompare, this, _1, _2));
    }

    new_timer_ = true;
    timers_cond_.notify_all();
  }

  return info->handle;
}

template<class T, class D, class E>
void TimerManager<T, D, E>::remove(int32_t handle)
{
  CallbackQueueInterface* callback_queue = 0;
  uint64_t remove_id = 0;

  {
    boost::mutex::scoped_lock lock(timers_mutex_);

    typename V_TimerInfo::iterator it = timers_.begin();
    typename V_TimerInfo::iterator end = timers_.end();
    for (; it != end; ++it)
    {
      const TimerInfoPtr& info = *it;
      if (info->handle == handle)
      {
        info->removed = true;
        callback_queue = info->callback_queue;
        remove_id = (uint64_t)info.get();
        timers_.erase(it);
        break;
      }
    }

    {
      boost::mutex::scoped_lock lock2(waiting_mutex_);
      // Remove from the waiting list if it's in it
      L_int32::iterator it = std::find(waiting_.begin(), waiting_.end(), handle);
      if (it != waiting_.end())
      {
        waiting_.erase(it);
      }
    }
  }

  if (callback_queue)
  {
    callback_queue->removeByID(remove_id);
  }
}

template<class T, class D, class E>
void TimerManager<T, D, E>::schedule(const TimerInfoPtr& info)
{
  boost::mutex::scoped_lock lock(timers_mutex_);

  if (info->removed)
  {
    return;
  }

  updateNext(info, T::now());
  {
    boost::mutex::scoped_lock lock(waiting_mutex_);

    waiting_.push_back(info->handle);
    // waitingCompare requires a lock on the timers_mutex_
    waiting_.sort(boost::bind(&TimerManager::waitingCompare, this, _1, _2));
  }

  new_timer_ = true;
  timers_cond_.notify_one();
}

template<class T, class D, class E>
void TimerManager<T, D, E>::updateNext(const TimerInfoPtr& info, const T& current_time)
{
  if (info->oneshot)
  {
    info->next_expected = T(INT_MAX, 999999999);
  }
  else
  {
    // Protect against someone having called setPeriod()
    // If the next expected time is already past the current time
    // don't update it
    if (info->next_expected <= current_time)
    {
      info->last_expected = info->next_expected;
      info->next_expected += info->period;
    }

    // detect time jumping forward, as well as callbacks that are too slow
    if (info->next_expected + info->period < current_time)
    {
      ROS_DEBUG("Time jumped forward by [%f] for timer of period [%f], resetting timer (current=%f, next_expected=%f)", (current_time - info->next_expected).toSec(), info->period.toSec(), current_time.toSec(), info->next_expected.toSec());
      info->next_expected = current_time;
    }
  }
}

template<class T, class D, class E>
void TimerManager<T, D, E>::setPeriod(int32_t handle, const D& period)
{
  boost::mutex::scoped_lock lock(timers_mutex_);
  TimerInfoPtr info = findTimer(handle);

  if (!info)
  {
    return;
  }

  {
    boost::mutex::scoped_lock lock(waiting_mutex_);
    info->period = period;
    info->next_expected = T::now() + period;

    waiting_.sort(boost::bind(&TimerManager::waitingCompare, this, _1, _2));
  }

  new_timer_ = true;
  timers_cond_.notify_one();
}

template<class T, class D, class E>
void TimerManager<T, D, E>::threadFunc()
{
  T current;
  while (!quit_)
  {
    T sleep_end;

    boost::mutex::scoped_lock lock(timers_mutex_);

    // detect time jumping backwards
    if (T::now() < current)
    {
      ROSCPP_LOG_DEBUG("Time jumped backward, resetting timers");

      current = T::now();

      typename V_TimerInfo::iterator it = timers_.begin();
      typename V_TimerInfo::iterator end = timers_.end();
      for (; it != end; ++it)
      {
        const TimerInfoPtr& info = *it;

        // Timer may have been added after the time jump, so also check if time has jumped past its last call time
        if (current < info->last_expected)
        {
          info->last_expected = current;
          info->next_expected = current + info->period;
        }
      }
    }

    current = T::now();

    {
      boost::mutex::scoped_lock waitlock(waiting_mutex_);

      if (waiting_.empty())
      {
        sleep_end = current + D(0.1);
      }
      else
      {
        TimerInfoPtr info = findTimer(waiting_.front());

        while (!waiting_.empty() && info && info->next_expected <= current)
        {
          current = T::now();

          //ROS_DEBUG("Scheduling timer callback for timer [%d] of period [%f], [%f] off expected", info->handle, info->period.toSec(), (current - info->next_expected).toSec());
          CallbackInterfacePtr cb(new TimerQueueCallback(this, info, info->last_expected, info->last_real, info->next_expected));
          info->callback_queue->addCallback(cb, (uint64_t)info.get());

          waiting_.pop_front();

          if (waiting_.empty())
          {
            break;
          }

          info = findTimer(waiting_.front());
        }

        if (info)
        {
          sleep_end = info->next_expected;
        }
      }
    }

    while (!new_timer_ && T::now() < sleep_end && !quit_)
    {
      // detect backwards jumps in time

      if (T::now() < current)
      {
        ROSCPP_LOG_DEBUG("Time jumped backwards, breaking out of sleep");
        break;
      }

      current = T::now();

      if (current >= sleep_end)
      {
        break;
      }

      // If we're on simulation time we need to check now() against sleep_end more often than on system time,
      // since simulation time may be running faster than real time.
      if (!T::isSystemTime())
      {
        timers_cond_.timed_wait(lock, boost::posix_time::milliseconds(1));
      }
      else
      {
        // On system time we can simply sleep for the rest of the wait time, since anything else requiring processing will
        // signal the condition variable
        int32_t remaining_time = std::max((int32_t)((sleep_end - current).toSec() * 1000.0f), 1);
        timers_cond_.timed_wait(lock, boost::posix_time::milliseconds(remaining_time));
      }
    }

    new_timer_ = false;
  }
}

}

#endif
