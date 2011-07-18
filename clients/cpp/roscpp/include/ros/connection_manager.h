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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#include "forwards.h"
#include "connection.h"
#include "common.h"

#include <boost/thread/mutex.hpp>
#include <boost/signals/connection.hpp>

namespace ros
{

class PollManager;
typedef boost::shared_ptr<PollManager> PollManagerPtr;

class ConnectionManager;
typedef boost::shared_ptr<ConnectionManager> ConnectionManagerPtr;

class ROSCPP_DECL ConnectionManager
{
public:
  static const ConnectionManagerPtr& instance();

  ConnectionManager();
  ~ConnectionManager();

  /** @brief Get a new connection ID
   */
  uint32_t getNewConnectionID();

  /** @brief Add a connection to be tracked by the node.  Will automatically remove them if they've been dropped, but from inside the ros thread
   *
   * @param The connection to add
   */
  void addConnection(const ConnectionPtr& connection);

  void clear(Connection::DropReason reason);

  uint32_t getTCPPort();
  uint32_t getUDPPort();

  const TransportTCPPtr& getTCPServerTransport() { return tcpserver_transport_; }
  const TransportUDPPtr& getUDPServerTransport() { return udpserver_transport_; }

  void udprosIncomingConnection(const TransportUDPPtr& transport, Header& header);

  void start();
  void shutdown();

private:
  void onConnectionDropped(const ConnectionPtr& conn);
  // Remove any dropped connections from our list, causing them to be destroyed
  // They can't just be removed immediately when they're dropped because the ros
  // thread may still be using them (or more likely their transport)
  void removeDroppedConnections();

  bool onConnectionHeaderReceived(const ConnectionPtr& conn, const Header& header);
  void tcprosAcceptConnection(const TransportTCPPtr& transport);

  PollManagerPtr poll_manager_;

  S_Connection connections_;
  V_Connection dropped_connections_;
  boost::mutex connections_mutex_;
  boost::mutex dropped_connections_mutex_;

  // The connection ID counter, used to assign unique ID to each inbound or
  // outbound connection.  Access via getNewConnectionID()
  uint32_t connection_id_counter_;
  boost::mutex connection_id_counter_mutex_;

  boost::signals::connection poll_conn_;

  TransportTCPPtr tcpserver_transport_;
  TransportUDPPtr udpserver_transport_;

  const static int MAX_TCPROS_CONN_QUEUE = 100; // magic
};

}

