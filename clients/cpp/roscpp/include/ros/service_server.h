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

#ifndef ROSCPP_SERVICE_HANDLE_H
#define ROSCPP_SERVICE_HANDLE_H

#include "ros/forwards.h"
#include "common.h"

namespace ros
{

/**
 * \brief Manages an service advertisement.
 *
 * A ServiceServer should always be created through a call to NodeHandle::advertiseService(), or copied from
 * one that was.  Once all copies of a specific
 * ServiceServer go out of scope, the service associated with it will be unadvertised and the service callback
 * will stop being called.
 */
class ROSCPP_DECL ServiceServer
{
public:
  ServiceServer() {}
  ServiceServer(const ServiceServer& rhs);
  ~ServiceServer();

  /**
   * \brief Unadvertise the service associated with this ServiceServer
   *
   * This method usually does not need to be explicitly called, as automatic shutdown happens when
   * all copies of this ServiceServer go out of scope
   *
   * This method overrides the automatic reference counted unadvertise, and immediately
   * unadvertises the service associated with this ServiceServer
   */
  void shutdown();

  std::string getService() const;

  operator void*() const { return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0; }

  bool operator<(const ServiceServer& rhs) const
  {
    return impl_ < rhs.impl_;
  }

  bool operator==(const ServiceServer& rhs) const
  {
    return impl_ == rhs.impl_;
  }

  bool operator!=(const ServiceServer& rhs) const
  {
    return impl_ != rhs.impl_;
  }

private:
  ServiceServer(const std::string& service, const NodeHandle& node_handle);

  class Impl
  {
  public:
    Impl();
    ~Impl();

    void unadvertise();
    bool isValid() const;

    std::string service_;
    NodeHandlePtr node_handle_;
    bool unadvertised_;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;

  friend class NodeHandle;
  friend class NodeHandleBackingCollection;
};
typedef std::vector<ServiceServer> V_ServiceServer;

}

#endif // ROSCPP_SERVICE_HANDLE_H


