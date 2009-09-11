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

#ifndef ROSCPP_TRANSPORT_HINTS_H
#define ROSCPP_TRANSPORT_HINTS_H

#include "ros/forwards.h"

#include <boost/lexical_cast.hpp>

namespace ros
{

class TransportHints
{
public:
  TransportHints& reliable()
  {
    tcp();

    return *this;
  }

  TransportHints& tcp()
  {
    transports_.push_back("TCP");
    return *this;
  }

  TransportHints& tcpNoDelay(bool nodelay = true)
  {
    options_["tcp_nodelay"] = nodelay ? "true" : "false";
    return *this;
  }

  bool getTCPNoDelay()
  {
    M_string::iterator it = options_.find("tcp_nodelay");
    if (it == options_.end())
    {
      return false;
    }

    const std::string& val = it->second;
    if (val == "true")
    {
      return true;
    }

    return false;
  }

  TransportHints& maxDatagramSize(int size)
  {
    options_["max_datagram_size"] = boost::lexical_cast<std::string>(size);
    return *this;
  }

  int getMaxDatagramSize()
  {
    M_string::iterator it = options_.find("max_datagram_size");
    if (it == options_.end())
    {
      return 0;
    }

    return boost::lexical_cast<int>(it->second);
  }

  TransportHints& unreliable()
  {
    udp();

    return *this;
  }

  TransportHints& udp()
  {
    transports_.push_back("UDP");
    return *this;
  }

  const V_string& getTransports() { return transports_; }
  const M_string& getOptions() { return options_; }

private:
  V_string transports_;
  M_string options_;
};

}

#endif
