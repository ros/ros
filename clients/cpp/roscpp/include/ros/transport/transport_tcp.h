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

#ifndef ROSCPP_TRANSPORT_TCP_H
#define ROSCPP_TRANSPORT_TCP_H

#include <ros/types.h>
#include <ros/transport/transport.h>

#include <boost/thread/recursive_mutex.hpp>

#include <netinet/in.h>

namespace ros
{

class TransportTCP;
typedef boost::shared_ptr<TransportTCP> TransportTCPPtr;

class PollSet;

/**
 * \brief TCPROS transport
 */
class TransportTCP : public Transport
{
public:
  static bool s_use_keepalive_;

public:
  enum Flags
  {
    SYNCHRONOUS = 1<<0,
  };

  TransportTCP(PollSet* poll_set, int flags = 0);
  virtual ~TransportTCP();

  /**
   * \brief Connect to a remote host.
   * \param host The hostname/IP to connect to
   * \param port The port to connect to
   * \return Whether or not the connection was successful
   */
  bool connect(const std::string& host, int port);

  /**
   * \brief Returns the URI of the remote host
   */
  std::string getClientURI();

  typedef boost::function<void(const TransportTCPPtr&)> AcceptCallback;
  /**
   * \brief Start a server socket and listen on a port
   * \param port The port to listen on
   * \param backlog defines the maximum length for the queue of pending connections.  Identical to the backlog parameter to the ::listen function
   * \param accept_cb The function to call when a client socket has connected
   */
  bool listen(int port, int backlog, const AcceptCallback& accept_cb);
  /**
   * \brief Accept a connection on a server socket.  Blocks until a connection is available
   */
  TransportTCPPtr accept();
  /**
   * \brief Returns the port this transport is listening on
   */
  int getServerPort() { return server_port_; }

  void setNoDelay(bool nodelay);
  void setKeepAlive(bool use, uint32_t idle, uint32_t interval, uint32_t count);

  const std::string& getConnectedHost() { return connected_host_; }
  int getConnectedPort() { return connected_port_; }

  // overrides from Transport
  virtual int32_t read(uint8_t* buffer, uint32_t size);
  virtual int32_t write(uint8_t* buffer, uint32_t size);

  virtual void enableWrite();
  virtual void disableWrite();
  virtual void enableRead();
  virtual void disableRead();

  virtual void close();

  virtual std::string getTransportInfo();

  virtual void parseHeader(const Header& header);

  virtual const char* getType() { return "TCPROS"; }

private:
  /**
   * \brief Initializes the assigned socket -- sets it to non-blocking and enables reading
   */
  bool initializeSocket();

  bool setNonBlocking();

  /**
   * \brief Set the socket to be used by this transport
   * \param sock A valid TCP socket
   * \return Whether setting the socket was successful
   */
  bool setSocket(int sock);

  void socketUpdate(int events);

  int sock_;
  bool closed_;
  boost::recursive_mutex close_mutex_;

  bool expecting_read_;
  bool expecting_write_;

  bool is_server_;
  sockaddr_in server_address_;
  int server_port_;
  AcceptCallback accept_cb_;

  std::string cached_remote_host_;

  PollSet* poll_set_;
  int flags_;

  std::string connected_host_;
  int connected_port_;
};

}

#endif // ROSCPP_TRANSPORT_TCP_H

