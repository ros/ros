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

#ifndef ROSCPP_SERVICE_CLIENT_LINK_H
#define ROSCPP_SERVICE_CLIENT_LINK_H

#include "ros/common.h"

#include <boost/thread/mutex.hpp>
#include <boost/shared_array.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/signals/connection.hpp>

#include <queue>

namespace ros
{
class Header;
class ServicePublication;
typedef boost::weak_ptr<ServicePublication> ServicePublicationWPtr;
typedef boost::shared_ptr<ServicePublication> ServicePublicationPtr;
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;

/**
 * \brief Handles a connection to a single incoming service client.
 */
class ROSCPP_DECL ServiceClientLink : public boost::enable_shared_from_this<ServiceClientLink>
{
public:
  ServiceClientLink();
  virtual ~ServiceClientLink();

  //
  bool initialize(const ConnectionPtr& connection);
  bool handleHeader(const Header& header);

  /**
   * \brief Writes a response to the current request.
   * \param ok Whether the callback was successful or not
   * \param resp The message response.  ServiceClientLink will delete this
   */
  void processResponse(bool ok, const SerializedMessage& res);

  const ConnectionPtr& getConnection() { return connection_; }

private:
  void onConnectionDropped(const ConnectionPtr& conn);

  void onHeaderWritten(const ConnectionPtr& conn);
  void onRequestLength(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);
  void onRequest(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);
  void onResponseWritten(const ConnectionPtr& conn);

  ConnectionPtr connection_;
  ServicePublicationWPtr parent_;
  bool persistent_;
  boost::signals::connection dropped_conn_;
};
typedef boost::shared_ptr<ServiceClientLink> ServiceClientLinkPtr;

} // namespace ros

#endif // ROSCPP_PUBLISHER_DATA_HANDLER_H



