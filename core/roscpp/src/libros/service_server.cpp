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

#include "ros/service_server.h"
#include "ros/service_link.h"
#include "ros/connection.h"
#include "ros/node.h"

#include <boost/bind.hpp>

namespace ros
{

ServiceServer::ServiceServer(const std::string& name, const std::string &md5sum, const std::string& request_data_type,
                             const std::string& response_data_type, AbstractServiceFunctor* callback, int thread_pool_size)
: name_(name)
, md5sum_(md5sum)
, request_data_type_(request_data_type)
, response_data_type_(response_data_type)
, service_callback_(callback)
, thread_pool_size_(thread_pool_size)
, dropped_(false)

{
  for (int i = 0; i < thread_pool_size; ++i)
  {
    threads_.push_back(thread_group_.create_thread(boost::bind(&ServiceServer::threadFunc, this)));
  }
}

ServiceServer::~ServiceServer()
{
  drop();

  delete service_callback_;
}

void ServiceServer::drop()
{
  // grab a lock here, to ensure that no subscription callback will
  // be invoked after we return
  {
    boost::mutex::scoped_lock lock(request_queue_mutex_);
    dropped_ = true;
  }

  new_request_.notify_all();

  dropAllConnections();

  bool found_self = false;
  V_threadpointer::iterator it = threads_.begin();
  V_threadpointer::iterator end = threads_.end();
  for (; it != end; ++it)
  {
    boost::thread* thread = *it;
    if (thread->get_id() == boost::this_thread::get_id())
    {
      found_self = true;
      break;
    }
  }

  if (!found_self)
  {
    thread_group_.join_all();
  }
}

void ServiceServer::processRequest(boost::shared_array<uint8_t> buf, size_t num_bytes, const ServiceLinkPtr& link)
{
  if (thread_pool_size_ != 0)
  {
    enqueueRequest(buf, num_bytes, link);
  }
  else
  {
    callCallback(buf, num_bytes, link);
  }
}

void ServiceServer::callCallback(boost::shared_array<uint8_t> buf, size_t num_bytes, const ServiceLinkPtr& link)
{
  Message* req = service_callback_->createRequest();
  Message* resp = service_callback_->createResponse();

  req->__connection_header = link->getConnection()->getHeader().getValues();

  req->__serialized_length = num_bytes;
  req->deserialize(buf.get());
  bool ok = service_callback_->call(req, resp);

  delete req;
  if (!ok)
  {
    delete resp;
    resp = NULL;
  }

  link->processResponse(ok, resp);
}

void ServiceServer::enqueueRequest(boost::shared_array<uint8_t> buf, size_t num_bytes, const ServiceLinkPtr& link)
{
  RequestInfoPtr info(new RequestInfo);
  info->buf_ = buf;
  info->num_bytes_ = num_bytes;
  info->link_ = link;

  {
    boost::mutex::scoped_lock lock(request_queue_mutex_);
    request_queue_.push(info);
  }

  new_request_.notify_one();
}

void ServiceServer::addServiceLink(const ServiceLinkPtr& link)
{
  {
    boost::mutex::scoped_lock lock(service_links_mutex_);

    service_links_.push_back(link);

    if (thread_pool_size_ == -1)
    {
      if (thread_group_.size() < service_links_.size())
      {
        threads_.push_back(thread_group_.create_thread(boost::bind(&ServiceServer::threadFunc, this)));

        ROS_ASSERT(thread_group_.size() == service_links_.size());
      }
    }
  }
}

void ServiceServer::removeServiceLink(const ServiceLinkPtr& link)
{
  boost::mutex::scoped_lock lock(service_links_mutex_);

  V_ServiceLink::iterator it = std::find(service_links_.begin(), service_links_.end(), link);
  if (it != service_links_.end())
  {
    service_links_.erase(it);
  }
}

void ServiceServer::dropAllConnections()
{
  // Swap our service_links_ list with a local one so we can only lock for a short period of time, because a
  // side effect of our calling drop() on connections can be re-locking the service_links_ mutex
  V_ServiceLink local_links;

  {
    boost::mutex::scoped_lock lock(service_links_mutex_);

    local_links.swap(service_links_);
  }

  for (V_ServiceLink::iterator i = local_links.begin();
           i != local_links.end(); ++i)
  {
    (*i)->getConnection()->drop();
  }
}

void ServiceServer::threadFunc()
{
  disableAllSignalsInThisThread();

  ServiceServerPtr self;

  while (!dropped_)
  {
    RequestInfoPtr info;

    {
      boost::mutex::scoped_lock lock(request_queue_mutex_);

      while (!dropped_ && request_queue_.empty())
      {
        new_request_.wait(lock);
      }

      if (dropped_)
      {
        break;
      }

      if (request_queue_.empty())
      {
        continue;
      }

      info = request_queue_.front();
      request_queue_.pop();

      // Keep a shared pointer to ourselves so we don't get deleted while in a callback
      // Fixes the case of unadvertising from within a callback
      if (!self)
      {
        self = shared_from_this();
      }
    }

    if (!dropped_)
    {
      ROS_ASSERT(info);
      callCallback(info->buf_, info->num_bytes_, info->link_);
    }
  }
}

} // namespace ros
