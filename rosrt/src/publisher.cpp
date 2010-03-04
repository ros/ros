/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include <ros/rt/publisher.h>
#include <ros/rt/detail/publisher_manager.h>
#include <ros/rt/object_pool.h>
#include <ros/rt/init.h>
#include <ros/debug.h>

#include <boost/thread.hpp>

namespace ros
{
namespace rt
{
namespace detail
{

boost::thread_specific_ptr<PublisherManager> g_publisher_manager;

bool publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func)
{
  PublisherManager* man = g_publisher_manager.get();
  ROS_ASSERT_MSG(man, "ros::rt::initThread() has not been called for this thread!\n%s", ros::debug::getBacktrace().c_str());

  return man->publish(pub, msg, pub_func);
}

PublishQueue::PublishQueue(uint32_t size)
: pool_(size, PubItem())
, head_(0)
{
}

bool PublishQueue::push(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func)
{
  PubItem* i = pool_.allocateBare();
  if (!i)
  {
    return false;
  }

  i->pub = pub;
  i->msg = msg;
  i->pub_func = pub_func;
  PubItem* stale_head = head_.load(memory_order_relaxed);
  do
  {
    i->next = stale_head;
  } while(!head_.compare_exchange_weak(stale_head, i, memory_order_release));

  return true;
}

void PublishQueue::publishAll()
{
  PubItem* last = head_.exchange(0, memory_order_consume);
  PubItem* first = 0;

  // Reverse the list to get it back in push() order
  while (last)
  {
    PubItem* tmp = last;
    last = last->next;
    tmp->next = first;
    first = tmp;
  }

  PubItem* it = first;
  while (it)
  {
    it->pub_func(it->pub, it->msg);
    it->msg.reset();
    it->pub = ros::Publisher();
    PubItem* tmp = it;
    it = it->next;

    pool_.freeBare(tmp);
  }
}

PublisherManager::PublisherManager(const InitOptions& ops)
: queue_(ops.pubmanager_queue_size)
, pub_count_(0)
, running_(true)
{
  pub_thread_ = boost::thread(&PublisherManager::publishThread, this);
}

PublisherManager::~PublisherManager()
{
  running_ = false;
  cond_.notify_one();
  pub_thread_.join();
}

void PublisherManager::publishThread()
{
  while (running_)
  {
    {
      boost::mutex::scoped_lock lock(cond_mutex_);
      while (running_ && pub_count_.load() == 0)
      {
        cond_.wait(lock);
      }

      if (!running_)
      {
        return;
      }
    }

    queue_.publishAll();
    pub_count_.fetch_sub(1);
  }
}

bool PublisherManager::publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func)
{
  if (!queue_.push(pub, msg, pub_func))
  {
    return false;
  }

  pub_count_.fetch_add(1);
  cond_.notify_one();

  return true;
}

} // namespace detail
} // namespace rt
} // namespace ros
