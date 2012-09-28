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

#include "ros/service_client.h"
#include "ros/service_server_link.h"
#include "ros/connection.h"
#include "ros/service_manager.h"
#include "ros/service.h"

namespace ros
{

ServiceClient::Impl::Impl() 
  : is_shutdown_(false)
{ }

ServiceClient::Impl::~Impl()
{
  shutdown();
}

void ServiceClient::Impl::shutdown()
{
  if (!is_shutdown_)
  {
    if (!persistent_)
    {
      is_shutdown_ = true;
    }

    if (server_link_)
    {
      server_link_->getConnection()->drop(Connection::Destructing);
      server_link_.reset();
    }
  }
}

bool ServiceClient::Impl::isValid() const
{
  // Non-persistent connections are always valid
  if (!persistent_)
  {
    return true;
  }

  if (is_shutdown_)
  {
    return false;
  }

  if (!server_link_)
  {
    return false;
  }

  return server_link_->isValid();
}

ServiceClient::ServiceClient(const std::string& service_name, bool persistent, const M_string& header_values, const std::string& service_md5sum)
: impl_(new Impl)
{
  impl_->name_ = service_name;
  impl_->persistent_ = persistent;
  impl_->header_values_ = header_values;
  impl_->service_md5sum_ = service_md5sum;

  if (persistent)
  {
    impl_->server_link_ = ServiceManager::instance()->createServiceServerLink(impl_->name_, impl_->persistent_, impl_->service_md5sum_, impl_->service_md5sum_, impl_->header_values_);
  }
}

ServiceClient::ServiceClient(const ServiceClient& rhs)
{
  impl_ = rhs.impl_;
}

ServiceClient::~ServiceClient()
{

}

bool ServiceClient::call(const SerializedMessage& req, SerializedMessage& resp, const std::string& service_md5sum)
{
  if (service_md5sum != impl_->service_md5sum_)
  {
    ROS_ERROR("Call to service [%s] with md5sum [%s] does not match md5sum when the handle was created ([%s])", impl_->name_.c_str(), service_md5sum.c_str(), impl_->service_md5sum_.c_str());

    return false;
  }

  ServiceServerLinkPtr link;

  if (impl_->persistent_)
  {
    if (!impl_->server_link_)
    {
      impl_->server_link_ = ServiceManager::instance()->createServiceServerLink(impl_->name_, impl_->persistent_, service_md5sum, service_md5sum, impl_->header_values_);

      if (!impl_->server_link_)
      {
        return false;
      }
    }

    link = impl_->server_link_;
  }
  else
  {
    link = ServiceManager::instance()->createServiceServerLink(impl_->name_, impl_->persistent_, service_md5sum, service_md5sum, impl_->header_values_);

    if (!link)
    {
      return false;
    }
  }

  bool ret = link->call(req, resp);
  link.reset();

  // If we're shutting down but the node haven't finished yet, wait until we do
  while (ros::isShuttingDown() && ros::ok())
  {
    ros::WallDuration(0.001).sleep();
  }

  return ret;
}

bool ServiceClient::isValid() const
{
  if (!impl_)
  {
    return false;
  }

  return impl_->isValid();
}

void ServiceClient::shutdown()
{
  if (impl_)
  {
    impl_->shutdown();
  }
}

bool ServiceClient::waitForExistence(ros::Duration timeout)
{
  if (impl_)
  {
    return service::waitForService(impl_->name_, timeout);
  }

  return false;
}

bool ServiceClient::exists()
{
  if (impl_)
  {
    return service::exists(impl_->name_, false);
  }

  return false;
}

std::string ServiceClient::getService()
{
  if (impl_)
  {
    return impl_->name_;
  }

  return "";
}

}
