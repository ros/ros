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

#include "ros/spinner.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"

#include <boost/thread/thread.hpp>

namespace ros
{

void SingleThreadedSpinner::spin(CallbackQueue* queue)
{
  ros::WallDuration d(0.01f);

  if (!queue)
  {
    queue = getGlobalCallbackQueue();
  }

  ros::NodeHandle n;
  while (n.ok())
  {
    queue->callAvailable();
  }
}

MultiThreadedSpinner::MultiThreadedSpinner(uint32_t thread_count)
: thread_count_(thread_count)
{
}

void MultiThreadedSpinner::spin(CallbackQueue* queue)
{
  AsyncSpinner s(thread_count_, queue);
  s.start();

  ros::waitForShutdown();
}

class AsyncSpinnerImpl
{
public:
  AsyncSpinnerImpl(uint32_t thread_count, CallbackQueue* queue);
  ~AsyncSpinnerImpl();
  void start();
  void stop();

private:
  void threadFunc();

  boost::mutex mutex_;
  boost::thread_group threads_;

  uint32_t thread_count_;
  CallbackQueue* callback_queue_;

  volatile bool continue_;

  ros::NodeHandle nh_;
};

AsyncSpinnerImpl::AsyncSpinnerImpl(uint32_t thread_count, CallbackQueue* queue)
: thread_count_(thread_count)
, callback_queue_(queue)
, continue_(false)
{
  if (thread_count == 0)
  {
    thread_count_ = boost::thread::hardware_concurrency();

    if (thread_count_ == 0)
    {
      thread_count_ = 1;
    }
  }

  if (!queue)
  {
    callback_queue_ = getGlobalCallbackQueue();
  }
}

AsyncSpinnerImpl::~AsyncSpinnerImpl()
{
  stop();
}

void AsyncSpinnerImpl::start()
{
  boost::mutex::scoped_lock lock(mutex_);
  if (continue_)
  {
    return;
  }

  continue_ = true;

  for (uint32_t i = 0; i < thread_count_; ++i)
  {
    threads_.create_thread(boost::bind(&AsyncSpinnerImpl::threadFunc, this));
  }
}

void AsyncSpinnerImpl::stop()
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!continue_)
  {
    return;
  }

  continue_ = false;
  threads_.join_all();
}

void AsyncSpinnerImpl::threadFunc()
{
  disableAllSignalsInThisThread();

  CallbackQueue* queue = callback_queue_;
  bool use_call_available = thread_count_ == 1;
  WallDuration timeout(0.01);

  while (continue_ && nh_.ok())
  {
    if (use_call_available)
    {
      queue->callAvailable(timeout);
    }
    else
    {
      queue->callOne(timeout);
    }
  }
}

AsyncSpinner::AsyncSpinner(uint32_t thread_count)
: impl_(new AsyncSpinnerImpl(thread_count, 0))
{
}

AsyncSpinner::AsyncSpinner(uint32_t thread_count, CallbackQueue* queue)
: impl_(new AsyncSpinnerImpl(thread_count, queue))
{
}

void AsyncSpinner::start()
{
  impl_->start();
}

void AsyncSpinner::stop()
{
  impl_->stop();
}

}
