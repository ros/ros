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

#ifndef ROSRT_PUBLISHER_MANAGER_H
#define ROSRT_PUBLISHER_MANAGER_H

#include <ros/atomic.h>
#include <ros/publisher.h>
#include <ros/rt/publisher.h>
#include <ros/rt/object_pool.h>
#include <boost/thread.hpp>

namespace ros
{

namespace rt
{
struct InitOptions;

class PublishQueue
{
public:
  struct PubItem
  {
    ros::Publisher pub;
    VoidConstPtr msg;
    PublishFunc pub_func;

    PubItem* next;
  };

  PublishQueue(uint32_t size);

  bool push(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func);
  void publishAll();

private:
  ObjectPool<PubItem> pool_;
  atomic<PubItem*> head_;
};

class PublisherManager
{
public:
  PublisherManager(const InitOptions& ops);
  ~PublisherManager();
  bool publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func);

private:
  void publishThread();

  PublishQueue queue_;
  boost::condition_variable cond_;
  boost::mutex cond_mutex_;
  boost::thread pub_thread_;
  atomic<uint32_t> pub_count_;
  volatile bool running_;
};

extern boost::thread_specific_ptr<PublisherManager> g_publisher_manager;

}
}

#endif

