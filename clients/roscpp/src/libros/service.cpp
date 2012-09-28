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
#include "ros/service_server_link.h"
#include "ros/service_manager.h"
#include "ros/transport/transport_tcp.h"
#include "ros/poll_manager.h"
#include "ros/init.h"
#include "ros/names.h"
#include "ros/this_node.h"
#include "ros/header.h"

using namespace ros;

bool service::exists(const std::string& service_name, bool print_failure_reason)
{
  std::string mapped_name = names::resolve(service_name);

  std::string host;
  uint32_t port;

  if (ServiceManager::instance()->lookupService(mapped_name, host, port))
  {
    TransportTCPPtr transport(new TransportTCP(0, TransportTCP::SYNCHRONOUS));

    if (transport->connect(host, port))
    {
      M_string m;
      m["probe"] = "1";
      m["md5sum"] = "*";
      m["callerid"] = this_node::getName();
      m["service"] = mapped_name;
      boost::shared_array<uint8_t> buffer;
      uint32_t size = 0;;
      Header::write(m, buffer, size);
      transport->write((uint8_t*)&size, sizeof(size));
      transport->write(buffer.get(), size);
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

bool service::waitForService(const std::string& service_name, ros::Duration timeout)
{
  std::string mapped_name = names::resolve(service_name);

  Time start_time = Time::now();

  bool printed = false;
  bool result = false;
  while (ros::ok())
  {
    if (exists(service_name, !printed))
    {
      result = true;
      break;
    }
    else
    {
      printed = true;

      if (timeout >= Duration(0))
      {
        Time current_time = Time::now();

        if ((current_time - start_time) >= timeout)
        {
          return false;
        }
      }

      Duration(0.02).sleep();
    }
  }

  if (printed && ros::ok())
  {
    ROS_INFO("waitForService: Service [%s] is now available.", mapped_name.c_str());
  }

  return result;
}

bool service::waitForService(const std::string& service_name, int32_t timeout)
{
  return waitForService(service_name, ros::Duration(timeout / 1000.0));
}
