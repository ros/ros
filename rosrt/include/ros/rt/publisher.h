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

#include "object_pool.h"

#include <ros/publisher.h>
#include <boost/utility.hpp>

namespace ros
{
namespace rt
{

struct InitOptions
{
  InitOptions()
  : pubmanager_queue_size(10000)
  {}

  uint32_t pubmanager_queue_size;
};
void initThread(const InitOptions& ops = InitOptions());

typedef void(*PublishFunc)(const ros::Publisher& pub, const VoidConstPtr& msg);

namespace detail
{
template<typename M>
void publishMessage(const ros::Publisher& pub, const VoidConstPtr& msg)
{
  boost::shared_ptr<M const> m = boost::static_pointer_cast<M const>(msg);
  pub.publish(m);
}
} // namespace detail

bool publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func);

template<typename M>
class Publisher : public boost::noncopyable
{
  typedef boost::shared_ptr<M const> MConstPtr;

public:
  Publisher()
  {
  }

  Publisher(const ros::Publisher& pub, uint32_t message_pool_size, const M& tmpl)
  {
    initialize(pub, message_pool_size, tmpl);
  }

  void initialize(const ros::Publisher& pub, uint32_t message_pool_size, const M& tmpl)
  {
    pub_ = pub;
    pool_.initialize(message_pool_size, tmpl);
  }

  bool publish(const MConstPtr& msg)
  {
    return ros::rt::publish(pub_, msg, detail::publishMessage<M>);
  }

  boost::shared_ptr<M> allocate()
  {
    return pool_.allocate();
  }

private:
  ros::Publisher pub_;
  ObjectPool<M> pool_;
};

}
}
