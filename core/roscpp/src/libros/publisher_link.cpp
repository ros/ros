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

#include "ros/publisher_link.h"
#include "ros/subscription.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/connection_manager.h"

#include <boost/bind.hpp>

#include <sstream>

namespace ros
{

PublisherLink::PublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const TransportHints& transport_hints)
: parent_(parent)
, publisher_xmlrpc_uri_(xmlrpc_uri)
, transport_hints_(transport_hints)
, latched_(false)
{
}

PublisherLink::~PublisherLink()
{
  ROS_ASSERT(connection_->isDropped());
}

bool PublisherLink::initialize(const ConnectionPtr& connection)
{
  connection_ = connection;
  connection_->addDropListener(boost::bind(&PublisherLink::onConnectionDropped, this, _1));

  if (connection_->getTransport()->requiresHeader())
    connection_->setHeaderReceivedCallback(boost::bind(&PublisherLink::onHeaderReceived, this, _1, _2));
  else
    connection_->read(4, boost::bind(&PublisherLink::onMessageLength, this, _1, _2, _3, _4));

  SubscriptionPtr parent = parent_.lock();

  M_string header;
  header["topic"] = parent->getName();
  header["md5sum"] = parent->md5sum();
  header["callerid"] = this_node::getName();
  header["type"] = parent->datatype();
  header["tcp_nodelay"] = transport_hints_.getTCPNoDelay() ? "1" : "0";
  connection_->writeHeader(header, boost::bind(&PublisherLink::onHeaderWritten, this, _1));

  return true;
}

void PublisherLink::onHeaderWritten(const ConnectionPtr& conn)
{
  // Do nothing
}

bool PublisherLink::onHeaderReceived(const ConnectionPtr& conn, const Header& header)
{
  std::string md5sum, type, latched_str;
  if (!header.getValue("md5sum", md5sum))
  {
    ROS_ERROR("Publisher TCPROS header did not have required element: md5sum");
    return false;
  }

  if (!header.getValue("type", type))
  {
    ROS_ERROR("Publisher TCPROS header did not have required element: type");
    return false;
  }

  latched_ = false;
  if (header.getValue("latching", latched_str))
  {
    if (latched_str == "1")
    {
      latched_ = true;
    }
  }



  connection_id_ = ConnectionManager::instance()->getNewConnectionID();

  connection_->read(4, boost::bind(&PublisherLink::onMessageLength, this, _1, _2, _3, _4));

  return true;
}

void PublisherLink::onMessageLength(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  if (!success)
  {
    if (connection_)
      connection_->read(4, boost::bind(&PublisherLink::onMessageLength, this, _1, _2, _3, _4));
    return;
  }

  ROS_ASSERT(conn == connection_);
  ROS_ASSERT(size == 4);

  uint32_t len = *((uint32_t*)buffer.get());

  if (len > 1000000000)
  {
    ROS_ERROR("woah! a message of over a gigabyte was " \
                "predicted in tcpros. that seems highly " \
                "unlikely, so I'll assume protocol " \
                "synchronization is lost... it's over.");
    conn->drop();
  }

  connection_->read(len, boost::bind(&PublisherLink::onMessage, this, _1, _2, _3, _4));
}

void PublisherLink::onMessage(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  if (!success && !conn)
    return;

  ROS_ASSERT(conn == connection_);

  if (success)
    handleMessage(buffer, size);

  if (success || !connection_->getTransport()->requiresHeader())
  {
    connection_->read(4, boost::bind(&PublisherLink::onMessageLength, this, _1, _2, _3, _4));
  }
}

void PublisherLink::onConnectionDropped(const ConnectionPtr& conn)
{
  ROS_ASSERT(conn == connection_);

  if (SubscriptionPtr parent = parent_.lock())
  {
    ROS_DEBUG("Connection to publisher [%s] to topic [%s] dropped", connection_->getTransport()->getTransportInfo().c_str(), parent->getName().c_str());

    parent->removePublisherLink(shared_from_this());
  }
}

void PublisherLink::handleMessage(const boost::shared_array<uint8_t>& buffer, size_t num_bytes)
{
  stats_.bytes_received_ += num_bytes;
  stats_.messages_received_++;

  SubscriptionPtr parent = parent_.lock();

  if (parent)
  {
    stats_.drops_ += parent->handleMessage(buffer, num_bytes, getConnection()->getHeader().getValues(), shared_from_this());
  }
}

std::string PublisherLink::getTransportType()
{
  return connection_->getTransport()->getType();
}

const std::string& PublisherLink::getPublisherXMLRPCURI()
{
  return publisher_xmlrpc_uri_;
}

} // namespace ros

