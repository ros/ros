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

#include "ros/subscriber.h"
#include "ros/node_handle.h"
#include "ros/topic_manager.h"

namespace ros
{

  Subscriber::Impl::Impl()
  : unsubscribed_(false)
  { }

  Subscriber::Impl::~Impl()
  {
    ROS_DEBUG("Subscriber on '%s' deregistering callbacks.", topic_.c_str());
    unsubscribe();
  }

  bool Subscriber::Impl::isValid() const
  {
    return !unsubscribed_;
  }

  void Subscriber::Impl::unsubscribe()
  {
    if (!unsubscribed_)
      {
	unsubscribed_ = true;
	TopicManager::instance()->unsubscribe(topic_, helper_);
	node_handle_.reset();
	helper_.reset();
      }
  }

  Subscriber::Subscriber(const std::string& topic, const NodeHandle& node_handle, 
			 const SubscriptionCallbackHelperPtr& helper)
  : impl_(new Impl)
  {
    impl_->topic_ = topic;
    impl_->node_handle_ = NodeHandlePtr(new NodeHandle(node_handle));
    impl_->helper_ = helper;
  }

  Subscriber::Subscriber(const Subscriber& rhs)
  {
    impl_ = rhs.impl_;
  }

  Subscriber::~Subscriber()
  {
  }

  void Subscriber::shutdown()
  {
    if (impl_)
      {
	impl_->unsubscribe();
      }
  }

  std::string Subscriber::getTopic() const
  {
    if (impl_)
      {
	return impl_->topic_;
      }

    return std::string();
  }

  uint32_t Subscriber::getNumPublishers() const
  {
    if (impl_ && impl_->isValid())
      {
	return TopicManager::instance()->getNumPublishers(impl_->topic_);
      }

    return 0;
  }

} // namespace ros
