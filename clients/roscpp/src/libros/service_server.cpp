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

#include "ros/service_server.h"
#include "ros/node_handle.h"
#include "ros/service_manager.h"

namespace ros
{

ServiceServer::Impl::Impl() : unadvertised_(false) { }

ServiceServer::Impl::~Impl()
{
  ROS_DEBUG("ServiceServer on '%s' deregistering callbacks.", service_.c_str());
  unadvertise();
}

bool ServiceServer::Impl::isValid() const
{
  return !unadvertised_;
}

void ServiceServer::Impl::unadvertise()
{
  if (!unadvertised_)
  {
    unadvertised_ = true;
    ServiceManager::instance()->unadvertiseService(service_);
    node_handle_.reset();
  }
}

ServiceServer::ServiceServer(const std::string& service, const NodeHandle& node_handle)
: impl_(new Impl)
{
  impl_->service_ = service;
  impl_->node_handle_ = NodeHandlePtr(new NodeHandle(node_handle));
}

ServiceServer::ServiceServer(const ServiceServer& rhs)
{
  impl_ = rhs.impl_;
}

ServiceServer::~ServiceServer()
{
}

void ServiceServer::shutdown()
{
  if (impl_)
  {
    impl_->unadvertise();
  }
}

std::string ServiceServer::getService() const
{
  if (impl_ && impl_->isValid())
  {
    return impl_->service_;
  }

  return std::string();
}

} // namespace ros
