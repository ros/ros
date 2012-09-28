/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "ros/service_publication.h"
#include "ros/service_client_link.h"
#include "ros/connection.h"
#include "ros/callback_queue_interface.h"

#include <boost/bind.hpp>

#include <std_msgs/String.h>

namespace ros
{

ServicePublication::ServicePublication(const std::string& name, const std::string &md5sum, const std::string& data_type, const std::string& request_data_type,
                             const std::string& response_data_type, const ServiceCallbackHelperPtr& helper, CallbackQueueInterface* callback_queue,
                             const VoidConstPtr& tracked_object)
: name_(name)
, md5sum_(md5sum)
, data_type_(data_type)
, request_data_type_(request_data_type)
, response_data_type_(response_data_type)
, helper_(helper)
, dropped_(false)
, callback_queue_(callback_queue)
, has_tracked_object_(false)
, tracked_object_(tracked_object)
{
  if (tracked_object)
  {
    has_tracked_object_ = true;
  }
}

ServicePublication::~ServicePublication()
{
  drop();
}

void ServicePublication::drop()
{
  // grab a lock here, to ensure that no subscription callback will
  // be invoked after we return
  {
    boost::mutex::scoped_lock lock(client_links_mutex_);
    dropped_ = true;
  }

  dropAllConnections();

  callback_queue_->removeByID((uint64_t)this);
}

class ServiceCallback : public CallbackInterface
{
public:
  ServiceCallback(const ServiceCallbackHelperPtr& helper, const boost::shared_array<uint8_t>& buf, size_t num_bytes, const ServiceClientLinkPtr& link, bool has_tracked_object, const VoidConstWPtr& tracked_object)
  : helper_(helper)
  , buffer_(buf)
  , num_bytes_(num_bytes)
  , link_(link)
  , has_tracked_object_(has_tracked_object)
  , tracked_object_(tracked_object)
  {
  }

  virtual CallResult call()
  {
    if (link_->getConnection()->isDropped())
    {
      return Invalid;
    }

    VoidConstPtr tracker;
    if (has_tracked_object_)
    {
      tracker = tracked_object_.lock();

      if (!tracker)
      {
        SerializedMessage res = serialization::serializeServiceResponse(false, 0);
        link_->processResponse(false, res);
        return Invalid;
      }
    }

    ServiceCallbackHelperCallParams params;
    params.request = SerializedMessage(buffer_, num_bytes_);
    params.connection_header = link_->getConnection()->getHeader().getValues();
    try
    {

      bool ok = helper_->call(params);
      link_->processResponse(ok, params.response);
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Exception thrown while processing service call: %s", e.what());
      std_msgs::String error_string;
      error_string.data = e.what();
      SerializedMessage res = serialization::serializeServiceResponse(false, error_string);
      link_->processResponse(false, res);
      return Invalid;
    }

    return Success;
  }

private:
  ServiceCallbackHelperPtr helper_;
  boost::shared_array<uint8_t> buffer_;
  uint32_t num_bytes_;
  ServiceClientLinkPtr link_;
  bool has_tracked_object_;
  VoidConstWPtr tracked_object_;
};

void ServicePublication::processRequest(boost::shared_array<uint8_t> buf, size_t num_bytes, const ServiceClientLinkPtr& link)
{
  CallbackInterfacePtr cb(new ServiceCallback(helper_, buf, num_bytes, link, has_tracked_object_, tracked_object_));
  callback_queue_->addCallback(cb, (uint64_t)this);
}

void ServicePublication::addServiceClientLink(const ServiceClientLinkPtr& link)
{
  boost::mutex::scoped_lock lock(client_links_mutex_);

  client_links_.push_back(link);
}

void ServicePublication::removeServiceClientLink(const ServiceClientLinkPtr& link)
{
  boost::mutex::scoped_lock lock(client_links_mutex_);

  V_ServiceClientLink::iterator it = std::find(client_links_.begin(), client_links_.end(), link);
  if (it != client_links_.end())
  {
    client_links_.erase(it);
  }
}

void ServicePublication::dropAllConnections()
{
  // Swap our client_links_ list with a local one so we can only lock for a short period of time, because a
  // side effect of our calling drop() on connections can be re-locking the client_links_ mutex
  V_ServiceClientLink local_links;

  {
    boost::mutex::scoped_lock lock(client_links_mutex_);

    local_links.swap(client_links_);
  }

  for (V_ServiceClientLink::iterator i = local_links.begin();
           i != local_links.end(); ++i)
  {
    (*i)->getConnection()->drop(Connection::Destructing);
  }
}

} // namespace ros
