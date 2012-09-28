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

#ifndef ROSCPP_CONNECTION_H
#define ROSCPP_CONNECTION_H

#include "ros/header.h"
#include "common.h"

#include <boost/signals.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#define READ_BUFFER_SIZE (1024*64)

namespace ros
{

class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;
typedef boost::function<void(const ConnectionPtr&, const boost::shared_array<uint8_t>&, uint32_t, bool)> ReadFinishedFunc;
typedef boost::function<void(const ConnectionPtr&)> WriteFinishedFunc;

typedef boost::function<bool(const ConnectionPtr&, const Header&)> HeaderReceivedFunc;

/**
 * \brief Encapsulates a connection to a remote host, independent of the transport type
 *
 * Connection provides automatic header negotiation, as well as easy ways of reading and writing
 * arbitrary amounts of data without having to set up your own state machines.
 */
class ROSCPP_DECL Connection : public boost::enable_shared_from_this<Connection>
{
public:
  enum DropReason
  {
    TransportDisconnect,
    HeaderError,
    Destructing,
  };

  Connection();
  ~Connection();

  /**
   * \brief Initialize this connection.
   */
  void initialize(const TransportPtr& transport, bool is_server, const HeaderReceivedFunc& header_func);
  /**
   * \brief Drop this connection.  Anything added as a drop listener through addDropListener will get called back when this connection has
   * been dropped.
   */
  void drop(DropReason reason);

  /**
   * \brief Returns whether or not this connection has been dropped
   */
  bool isDropped();

  /**
   * \brief Returns true if we're currently sending a header error (and will be automatically dropped when it's finished)
   */
  bool isSendingHeaderError() { return sending_header_error_; }

  /**
   * \brief Send a header error message, of the form "error=<message>".  Drops the connection once the data has written successfully (or fails to write)
   * \param error_message The error message
   */
  void sendHeaderError(const std::string& error_message);
  /**
   * \brief Send a list of string key/value pairs as a header message.
   * \param key_vals The values to send.  Neither keys nor values can have any newlines in them
   * \param finished_callback The function to call when the header has finished writing
   */
  void writeHeader(const M_string& key_vals, const WriteFinishedFunc& finished_callback);

  /**
   * \brief Read a number of bytes, calling a callback when finished
   *
   * read() will not queue up multiple reads.  Once read() has been called, it is not valid to call it again until the
   * finished callback has been called.  It \b is valid to call read() from within the finished callback.
   *
   * The finished callback is of the form void(const ConnectionPtr&, const boost::shared_array<uint8_t>&, uint32_t)
   *
   * \note The finished callback may be called from within this call to read() if the data has already arrived
   *
   * \param size The size, in bytes, of data to read
   * \param finished_callback The function to call when this read is finished
   */
  void read(uint32_t size, const ReadFinishedFunc& finished_callback);
  /**
   * \brief Write a buffer of bytes, calling a callback when finished
   *
   * write() will not queue up multiple writes.  Once write() has been called, it is not valid to call it again until
   * the finished callback has been called.  It \b is valid to call write() from within the finished callback.
   *
   * * The finished callback is of the form void(const ConnectionPtr&)
   *
   * \note The finished callback may be called from within this call to write() if the data can be written immediately
   *
   * \param buffer The buffer of data to write
   * \param size The size of the buffer, in bytes
   * \param finished_callback The function to call when the write has finished
   * \param immediate Whether to immediately try to write as much data as possible to the socket or to pass
   * the data off to the server thread
   */
  void write(const boost::shared_array<uint8_t>& buffer, uint32_t size, const WriteFinishedFunc& finished_callback, bool immedate = true);

  typedef boost::signal<void(const ConnectionPtr&, DropReason reason)> DropSignal;
  typedef boost::function<void(const ConnectionPtr&, DropReason reason)> DropFunc;
  /**
   * \brief Add a callback to be called when this connection has dropped
   */
  boost::signals::connection addDropListener(const DropFunc& slot);
  void removeDropListener(const boost::signals::connection& c);

  /**
   * \brief Set the header receipt callback
   */
  void setHeaderReceivedCallback(const HeaderReceivedFunc& func);

  /**
   * \brief Get the Transport associated with this connection
   */
  const TransportPtr& getTransport() { return transport_; }
  /**
   * \brief Get the Header associated with this connection
   */
  Header& getHeader() { return header_; }

  /**
   * \brief Set the Header associated with this connection (used with UDPROS, 
   *        which receives the connection during XMLRPC negotiation).
   */
  void setHeader(const Header& header) { header_ = header; }

  std::string getCallerId();
  std::string getRemoteString();

private:
  /**
   * \brief Called by the Transport when there is data available to be read
   */
  void onReadable(const TransportPtr& transport);
  /**
   * \brief Called by the Transport when it is possible to write data
   */
  void onWriteable(const TransportPtr& transport);
  /**
   * \brief Called by the Transport when it has been disconnected, either through a call to close()
   * or through an error in the connection (such as a remote disconnect)
   */
  void onDisconnect(const TransportPtr& transport);


  void onHeaderWritten(const ConnectionPtr& conn);
  void onErrorHeaderWritten(const ConnectionPtr& conn);
  void onHeaderLengthRead(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);
  void onHeaderRead(const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);

  /**
   * \brief Read data off our transport.  Also manages calling the read callback.  If there is any data to be read,
   * read() will read it until the fixed read buffer is filled.
   */
  void readTransport();
  /**
   * \brief Write data to our transport.  Also manages calling the write callback.
   */
  void writeTransport();

  /// Are we a server?  Servers wait for clients to send a header and then send a header in response.
  bool is_server_;
  /// Have we dropped?
  bool dropped_;
  /// Incoming header
  Header header_;
  /// Transport associated with us
  TransportPtr transport_;
  /// Function that handles the incoming header
  HeaderReceivedFunc header_func_;

  /// Read buffer that ends up being passed to the read callback
  boost::shared_array<uint8_t> read_buffer_;
  /// Amount of data currently in the read buffer, in bytes
  uint32_t read_filled_;
  /// Size of the read buffer, in bytes
  uint32_t read_size_;
  /// Function to call when the read is finished
  ReadFinishedFunc read_callback_;
  /// Mutex used for protecting reading.  Recursive because a read can immediately cause another read through the callback.
  boost::recursive_mutex read_mutex_;
  /// Flag telling us if we're in the middle of a read (mostly to avoid recursive deadlocking)
  bool reading_;
  /// flag telling us if there is a read callback
  /// 32-bit loads and stores are atomic on x86 and PPC... TODO: use a cross-platform atomic operations library
  /// to ensure this is done atomically
  volatile uint32_t has_read_callback_;

  /// Buffer to write from
  boost::shared_array<uint8_t> write_buffer_;
  /// Amount of data we've written from the write buffer
  uint32_t write_sent_;
  /// Size of the write buffer
  uint32_t write_size_;
  /// Function to call when the current write is finished
  WriteFinishedFunc write_callback_;
  boost::mutex write_callback_mutex_;
  /// Mutex used for protecting writing.  Recursive because a write can immediately cause another write through the callback
  boost::recursive_mutex write_mutex_;
  /// Flag telling us if we're in the middle of a write (mostly used to avoid recursive deadlocking)
  bool writing_;
  /// flag telling us if there is a write callback
  /// 32-bit loads and stores are atomic on x86 and PPC... TODO: use a cross-platform atomic operations library
  /// to ensure this is done atomically
  volatile uint32_t has_write_callback_;

  /// Function to call when the outgoing header has finished writing
  WriteFinishedFunc header_written_callback_;

  /// Signal raised when this connection is dropped
  DropSignal drop_signal_;

  /// Synchronizes drop() calls
  boost::recursive_mutex drop_mutex_;

  /// If we're sending a header error we disable most other calls
  bool sending_header_error_;
};
typedef boost::shared_ptr<Connection> ConnectionPtr;

} // namespace ros

#endif // ROSCPP_CONNECTION_H
