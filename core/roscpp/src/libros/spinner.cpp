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

extern CallbackQueue g_global_queue;

void SingleThreadedSpinner::spin(CallbackQueue* queue)
{
  ros::WallDuration d(0.01f);

  if (!queue)
  {
    queue = &g_global_queue;
  }

  ros::NodeHandle n;
  while (n.ok())
  {
    queue->callAvailable();
  }
}

MultiThreadedSpinner::MultiThreadedSpinner(uint32_t thread_count)
{
  if (thread_count == 0)
  {
    thread_count = boost::thread::hardware_concurrency();

    if (thread_count == 0)
    {
      thread_count = 1;
    }
  }

  thread_count_ = thread_count;
}

void spinThreadFunc(CallbackQueue* queue)
{
  disableAllSignalsInThisThread();

  ros::WallDuration d(0.01f);

  ros::NodeHandle n;
  while (n.ok())
  {
    queue->callOne();
  }
}

void MultiThreadedSpinner::spin(CallbackQueue* queue)
{
  if (!queue)
  {
    queue = &g_global_queue;
  }

  boost::thread_group tg;
  for (uint32_t i = 0; i < thread_count_; ++i)
  {
    tg.create_thread(boost::bind(spinThreadFunc, queue));
  }

  tg.join_all();
}

}
