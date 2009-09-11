/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#include "ros/service.h"
#include "ros/connection.h"
#include "ros/service_client.h"
#include "ros/node.h"
#include "ros/transport/transport_tcp.h"

#include <boost/thread/thread_time.hpp>

using namespace ros;

bool service::callImpl(const std::string& service_name, Message* req, Message* resp, const std::string& service_md5sum)
{
  ServiceClientPtr client = g_node->createServiceClient(service_name, false, service_md5sum, service_md5sum, M_string());
  if (!client)
  {
    return false;
  }

  if (client->getRequestMD5Sum() != service_md5sum || client->getResponseMD5Sum() != service_md5sum)
  {
    ROS_ERROR("Tried to use a persistent service connection with a different message md5sum for service [%s]", service_name.c_str());

    return false;
  }

  return client->call(req, resp);
}

service::ServiceHandlePtr service::createHandleImpl(const std::string& service_name, bool persistent, const M_string& header_values, const std::string& service_md5sum)
{
  ServiceHandlePtr handle(new ServiceHandle(g_node->mapName(service_name), persistent, header_values, service_md5sum));

  return handle;
}

bool service::exists(const std::string& service_name, bool print_failure_reason)
{
  std::string mapped_name = g_node->mapName(service_name);

  std::string host;
  int port;

  if (g_node->lookupService(mapped_name, host, port))
  {
    TransportTCPPtr transport(new TransportTCP(&g_node->getPollSet()));

    if (transport->connect(host, port))
    {
      transport->close();

      return true;
    }
    else
    {
      if (print_failure_reason)
      {
        ROS_INFO("waitForService: Service [%s] could not connect to host [%s:%d], waiting...", mapped_name.c_str(), host.c_str(), port);
      }
    }
  }
  else
  {
    if (print_failure_reason)
    {
      ROS_INFO("waitForService: Service [%s] has not been advertised, waiting...", mapped_name.c_str());
    }
  }

  return false;
}

bool service::waitForService(const std::string& service_name, int32_t timeout)
{
  std::string mapped_name = g_node->mapName(service_name);

  boost::system_time start_time = boost::get_system_time();

  bool printed = false;
  bool result = false;
  while (g_node->ok())
  {
    if (exists(service_name, !printed))
    {
      result = true;
      break;
    }
    else
    {
      printed = true;

      if (timeout >= 0)
      {
        boost::system_time current_time = boost::get_system_time();

        if ((current_time - start_time) >= boost::posix_time::milliseconds(timeout))
        {
          return false;
        }
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
  }

  if (printed && g_node->ok())
  {
    ROS_INFO("waitForService: Service [%s] is now available.", mapped_name.c_str());
  }

  return result;
}

service::ServiceHandle::ServiceHandle(const std::string& service_name, bool persistent, const M_string& header_values, const std::string& service_md5sum)
: name_(service_name)
, persistent_(persistent)
, header_values_(header_values)
, service_md5sum_(service_md5sum)
{
  if (persistent_)
  {
    client_ = g_node->createServiceClient(name_, persistent_, service_md5sum, service_md5sum, header_values_);
  }
}

service::ServiceHandle::~ServiceHandle()
{
  if (client_)
  {
    client_->getConnection()->drop();
  }
}

bool service::ServiceHandle::callImpl(Message* req, Message* resp, const std::string& service_md5sum)
{
  if (service_md5sum != service_md5sum_)
  {
    ROS_ERROR("Call to service [%s] with md5sum [%s] does not match md5sum when the handle was created ([%s])", name_.c_str(), service_md5sum.c_str(), service_md5sum_.c_str());

    return false;
  }

  if (!client_)
  {
    client_ = g_node->createServiceClient(name_, persistent_, service_md5sum, service_md5sum, header_values_);

    if (!client_)
    {
      return false;
    }
  }

  bool ret = client_->call(req, resp);

  if (!persistent_)
  {
    client_ = ServiceClientPtr();
  }

  return ret;
}

bool service::ServiceHandle::valid()
{
  // non-persistent service handles are always valid
  if (!persistent_)
  {
    return true;
  }

  return client_->isValid();
}
