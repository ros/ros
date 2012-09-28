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

#ifndef ROSCPP_TRANSPORT_H
#define ROSCPP_TRANSPORT_H

#include <ros/types.h>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace ros
{

class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;

class Header;

/**
 * \brief Abstract base class that allows abstraction of the transport type, eg. TCP, shared memory, UDP...
 */
class Transport : public boost::enable_shared_from_this<Transport>
{
public:
  Transport() {}
  virtual ~Transport() {}

  /**
   * \brief Read a number of bytes into the supplied buffer.  Not guaranteed to actually read that number of bytes.
   * \param buffer Buffer to read from
   * \param size Size, in bytes, to read
   * \return The number of bytes actually read, or -1 if there was an error
   */
  virtual int32_t read(uint8_t* buffer, uint32_t size) = 0;
  /**
   * \brief Write a number of bytes from the supplied buffer.  Not guaranteed to actually write that number of bytes.
   * \param buffer Buffer to write from
   * \param size Size, in bytes, to write
   * \return The number of bytes actually written, or -1 if there was an error
   */
  virtual int32_t write(uint8_t* buffer, uint32_t size) = 0;

  /**
   * \brief Enable writing on this transport.  Allows derived classes to, for example, enable write polling for asynchronous sockets
   */
  virtual void enableWrite() = 0;
  /**
   * \brief Disable writing on this transport.  Allows derived classes to, for example, disable write polling for asynchronous sockets
   */
  virtual void disableWrite() = 0;

  /**
   * \brief Enable reading on this transport.  Allows derived classes to, for example, enable read polling for asynchronous sockets
   */
  virtual void enableRead() = 0;
  /**
   * \brief Disable reading on this transport.  Allows derived classes to, for example, disable read polling for asynchronous sockets
   */
  virtual void disableRead() = 0;

  /**
   * \brief Close this transport.  Once this call has returned, writing on this transport should always return an error.
   */
  virtual void close() = 0;

  /**
   * \brief Return a string that details the type of transport (Eg. TCPROS)
   * \return The stringified transport type
   */
  virtual const char* getType() = 0;

  typedef boost::function<void(const TransportPtr&)> Callback;
  /**
   * \brief Set the function to call when this transport has disconnected, either through a call to close(). Or a disconnect from the remote host.
   */
  void setDisconnectCallback(const Callback& cb) { disconnect_cb_ = cb; }
  /**
   * \brief Set the function to call when there is data available to be read by this transport
   */
  void setReadCallback(const Callback& cb) { read_cb_ = cb; }
  /**
   * \brief Set the function to call when there is space available to write on this transport
   */
  void setWriteCallback(const Callback& cb) { write_cb_ = cb; }

  /**
   * \brief Returns a string description of both the type of transport and who the transport is connected to
   */
  virtual std::string getTransportInfo() = 0;

  /**
   * \brief Returns a boolean to indicate if the transport mechanism is reliable or not
   */
  virtual bool requiresHeader() {return true;}

  /**
   * \brief Provides an opportunity for transport-specific options to come in through the header
   */
  virtual void parseHeader(const Header& header) { }

protected:
  Callback disconnect_cb_;
  Callback read_cb_;
  Callback write_cb_;
};

}

#endif // ROSCPP_TRANSPORT_H
