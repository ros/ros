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
#include <ros/debug.h>

#include <boost/thread.hpp>

namespace ros
{
namespace rt
{

class PublisherManager
{
public:
  PublisherManager(const InitOptions& ops);
  ~PublisherManager();
  void publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func);

private:
  void publishThread();

  struct PubItem
  {
    ros::Publisher pub;
    VoidConstPtr msg;
    PublishFunc pub_func;
  };

  // TODO: lockfree-i-cise
  typedef std::vector<PubItem> V_PubItem;
  V_PubItem items_;
  boost::mutex items_mutex_;
  boost::condition_variable cond_;
  boost::thread pub_thread_;
  volatile bool running_;
};
boost::thread_specific_ptr<PublisherManager> g_publisher_manager;

// TODO: move this out of publisher.cpp
void initThread(const InitOptions& ops)
{
  ROS_ASSERT(!g_publisher_manager.get());
  g_publisher_manager.reset(new PublisherManager(ops));
}

void publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func)
{
  PublisherManager* man = g_publisher_manager.get();
  ROS_ASSERT_MSG(man, "ros::rt::initThread() has not been called for this thread!\n%s", ros::debug::getBacktrace().c_str());

  man->publish(pub, msg, pub_func);
}

PublisherManager::PublisherManager(const InitOptions& ops)
: running_(true)
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
    V_PubItem local_items;
    {
      boost::mutex::scoped_lock lock(items_mutex_);
      while (running_ && items_.empty())
      {
        cond_.wait(lock);
      }

      if (!running_)
      {
        return;
      }

      local_items.insert(local_items.end(), items_.begin(), items_.end());
      items_.clear();
    }

    V_PubItem::iterator it = local_items.begin();
    V_PubItem::iterator end = local_items.end();
    for (; it != end; ++it)
    {
      PubItem& i = *it;
      i.pub_func(i.pub, i.msg);
    }
  }
}

void PublisherManager::publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func)
{
  PubItem i;
  i.pub = pub;
  i.msg = msg;
  i.pub_func = pub_func;

  {
    boost::mutex::scoped_lock lock(items_mutex_);
    items_.push_back(i);
  }

  cond_.notify_one();
}

}
}
