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

#include "ros/service_server_link.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/service_manager.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/file_log.h"

#include <boost/bind.hpp>

#include <sstream>

namespace ros
{

ServiceServerLink::ServiceServerLink(const std::string& service_name, bool persistent, const std::string& request_md5sum,
                             const std::string& response_md5sum, const M_string& header_values)
: service_name_(service_name)
, persistent_(persistent)
, request_md5sum_(request_md5sum)
, response_md5sum_(response_md5sum)
, extra_outgoing_header_values_(header_values)
, header_written_(false)
, header_read_(false)
, dropped_(false)
{
}

ServiceServerLink::~ServiceServerLink()
{
  ROS_ASSERT(connection_->isDropped());

  clearCalls();
}

void ServiceServerLink::cancelCall(const CallInfoPtr& info)
{
  CallInfoPtr local = info;
  {
    boost::mutex::scoped_lock lock(local->finished_mutex_);
    local->finished_ = true;
    local->finished_condition_.notify_all();
  }

  if (boost::this_thread::get_id() != info->caller_thread_id_)
  {
    while (!local->call_finished_)
    {
      boost::this_thread::yield();
    }
  }
}

void ServiceServerLink::clearCalls()
{
  CallInfoPtr local_current;

  {
    boost::mutex::scoped_lock lock(call_queue_mutex_);
    local_current = current_call_;
  }

  if (local_current)
  {
    cancelCall(local_current);
  }

  boost::mutex::scoped_lock lock(call_queue_mutex_);

  while (!call_queue_.empty())
  {
    CallInfoPtr info = call_queue_.front();

    cancelCall(info);

    call_queue_.pop();
  }
}

bool ServiceServerLink::initialize(const ConnectionPtr& connection)
{
  connection_ = connection;
  connection_->addDropListener(boost::bind(&ServiceServerLink::onConnectionDropped, this, _1));
  connection_->setHeaderReceivedCallback(boost::bind(&ServiceServerLink::onHeaderReceived, this, _1, _2));

  M_string header;
  header["service"] = service_name_;
  header["md5sum"] = request_md5sum_;
  header["callerid"] = this_node::getName();
  header["persistent"] = persistent_ ? "1" : "0";
  header.insert(extra_outgoing_header_values_.begin(), extra_outgoing_header_values_.end());

  connection_->writeHeader(header, boost::bind(&ServiceServerLink::onHeaderWritten, this, _1));

  return true;
}

void ServiceServerLink::onHeaderWritten(const ConnectionPtr& conn)
{
  header_written_ = true;
}

bool ServiceServerLink::onHeaderReceived(const ConnectionPtr& conn, const Header& header)
{
  std::string md5sum, type;
  if (!header.getValue("md5sum", md5sum))
  {
    ROS_ERROR("TCPROS header from service server did not have required element: md5sum");
    return false;
  }

  bool empty = false;
  {
    boost::mutex::scoped_lock lock(call_queue_mutex_);
    empty = call_queue_.empty();

    if (empty)
    {
      header_read_ = true;
    }
  }

  if (!empty)
  {
    processNextCall();

    header_read_ = true;
  }

  return true;
}

void ServiceServerLink::onConnectionDropped(const ConnectionPtr& conn)
{
  ROS_ASSERT(conn == connection_);
  ROSCPP_LOG_DEBUG("Service client from [%s] for [%s] dropped", conn->getRemoteString().c_str(), service_name_.c_str());

  dropped_ = true;
  clearCalls();

  ServiceManager::instance()->removeServiceServerLink(shared_from_this());
}

void ServiceServerLink::onRequestWritten(const ConnectionPtr& conn)
{
  //ros::WallDuration(0.1).sleep();
  connection_->read(5, boost::bind(&ServiceServerLink::onResponseOkAndLength, this, _1, _2, _3, _4));
}

void ServiceServerLink::onResponseOkAndLength(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  ROS_ASSERT(conn == connection_);
  ROS_ASSERT(size == 5);

  if (!success)
    return;

  uint8_t ok = buffer[0];
  uint32_t len = *((uint32_t*)(buffer.get() + 1));

  if (len > 1000000000)
  {
    ROS_ERROR("a message of over a gigabyte was " \
                "predicted in tcpros. that seems highly " \
                "unlikely, so I'll assume protocol " \
                "synchronization is lost.");
    conn->drop(Connection::Destructing);

    return;
  }

  {
    boost::mutex::scoped_lock lock(call_queue_mutex_);
    if ( ok != 0 ) {
    	current_call_->success_ = true;
    } else {
    	current_call_->success_ = false;
    }
  }

  if (len > 0)
  {
    connection_->read(len, boost::bind(&ServiceServerLink::onResponse, this, _1, _2, _3, _4));
  }
  else
  {
    onResponse(conn, boost::shared_array<uint8_t>(), 0, true);
  }
}

void ServiceServerLink::onResponse(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  ROS_ASSERT(conn == connection_);

  if (!success)
    return;

  {
    boost::mutex::scoped_lock queue_lock(call_queue_mutex_);

    if (current_call_->success_)
    {
      *current_call_->resp_ = SerializedMessage(buffer, size);
    }
    else
    {
      current_call_->exception_string_ = std::string(reinterpret_cast<char*>(buffer.get()), size);
    }
  }

  callFinished();
}

void ServiceServerLink::callFinished()
{
  CallInfoPtr saved_call;
  ServiceServerLinkPtr self;
  {
    boost::mutex::scoped_lock queue_lock(call_queue_mutex_);
    boost::mutex::scoped_lock finished_lock(current_call_->finished_mutex_);

    ROS_DEBUG_NAMED("superdebug", "Client to service [%s] call finished with success=[%s]", service_name_.c_str(), current_call_->success_ ? "true" : "false");

    current_call_->finished_ = true;
    current_call_->finished_condition_.notify_all();

    saved_call = current_call_;
    current_call_ = CallInfoPtr();

    // If the call queue is empty here, we may be deleted as soon as we release these locks, so keep a shared pointer to ourselves until we return
    // ugly
    // jfaust TODO there's got to be a better way
    self = shared_from_this();
  }

  saved_call = CallInfoPtr();

  processNextCall();
}

void ServiceServerLink::processNextCall()
{
  bool empty = false;
  {
    boost::mutex::scoped_lock lock(call_queue_mutex_);

    if (current_call_)
    {
      return;
    }

    if (!call_queue_.empty())
    {
      ROS_DEBUG_NAMED("superdebug", "[%s] Client to service [%s] processing next service call", persistent_ ? "persistent" : "non-persistent", service_name_.c_str());

      current_call_ = call_queue_.front();
      call_queue_.pop();
    }
    else
    {
      empty = true;
    }
  }

  if (empty)
  {
    if (!persistent_)
    {
      ROS_DEBUG_NAMED("superdebug", "Dropping non-persistent client to service [%s]", service_name_.c_str());
      connection_->drop(Connection::Destructing);
    }
    else
    {
      ROS_DEBUG_NAMED("superdebug", "Keeping persistent client to service [%s]", service_name_.c_str());
    }
  }
  else
  {
    SerializedMessage request;

    {
      boost::mutex::scoped_lock lock(call_queue_mutex_);
      request = current_call_->req_;
    }

    connection_->write(request.buf, request.num_bytes, boost::bind(&ServiceServerLink::onRequestWritten, this, _1));
  }
}

bool ServiceServerLink::call(const SerializedMessage& req, SerializedMessage& resp)
{
  CallInfoPtr info(new CallInfo);
  info->req_ = req;
  info->resp_ = &resp;
  info->success_ = false;
  info->finished_ = false;
  info->call_finished_ = false;
  info->caller_thread_id_ = boost::this_thread::get_id();

  //ros::WallDuration(0.1).sleep();

  bool immediate = false;
  {
    boost::mutex::scoped_lock lock(call_queue_mutex_);

    if (connection_->isDropped())
    {
      ROSCPP_LOG_DEBUG("ServiceServerLink::call called on dropped connection for service [%s]", service_name_.c_str());
      info->call_finished_ = true;
      return false;
    }

    if (call_queue_.empty() && header_written_ && header_read_)
    {
      immediate = true;
    }

    call_queue_.push(info);
  }

  if (immediate)
  {
    processNextCall();
  }

  {
    boost::mutex::scoped_lock lock(info->finished_mutex_);

    while (!info->finished_)
    {
      info->finished_condition_.wait(lock);
    }
  }

  info->call_finished_ = true;

  if (info->exception_string_.length() > 0)
  {
    ROS_ERROR("Service call failed: service [%s] responded with an error: %s", service_name_.c_str(), info->exception_string_.c_str());
  }

  return info->success_;
}

bool ServiceServerLink::isValid() const
{
  return !dropped_;
}

} // namespace ros

