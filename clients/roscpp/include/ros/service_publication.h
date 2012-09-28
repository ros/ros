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

#ifndef ROSCPP_SERVICE_PUBLICATION_H
#define ROSCPP_SERVICE_PUBLICATION_H

#include "ros/service_callback_helper.h"
#include "common.h"
#include "XmlRpc.h"

#include <boost/thread/mutex.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/thread.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <vector>
#include <queue>

namespace ros
{

class ServiceClientLink;
typedef boost::shared_ptr<ServiceClientLink> ServiceClientLinkPtr;
typedef std::vector<ServiceClientLinkPtr> V_ServiceClientLink;
class CallbackQueueInterface;

class Message;

/**
 * \brief Manages an advertised service.
 *
 * ServicePublication manages all incoming service requests.  If its thread pool size is not 0, it will queue the requests
 * into a number of threads, calling the callback from within those threads.  Otherwise it immediately calls the callback
 */
class ROSCPP_DECL ServicePublication : public boost::enable_shared_from_this<ServicePublication>
{
public:
  ServicePublication(const std::string& name, const std::string &md5sum, const std::string& data_type, const std::string& request_data_type,
                const std::string& response_data_type, const ServiceCallbackHelperPtr& helper, CallbackQueueInterface* queue,
                const VoidConstPtr& tracked_object);
  ~ServicePublication();

  /**
   * \brief Adds a request to the queue if our thread pool size is not 0, otherwise immediately calls the callback
   */
  void processRequest(boost::shared_array<uint8_t> buf, size_t num_bytes, const ServiceClientLinkPtr& link);

  /**
   * \brief Adds a service link for us to manage
   */
  void addServiceClientLink(const ServiceClientLinkPtr& link);
  /**
   * \brief Removes a service link from our list
   */
  void removeServiceClientLink(const ServiceClientLinkPtr& link);

  /**
   * \brief Terminate this service server
   */
  void drop();
  /**
   * \brief Returns whether or not this service server is valid
   */
  bool isDropped() { return dropped_; }

  const std::string& getMD5Sum() { return md5sum_; }
  const std::string& getRequestDataType() { return request_data_type_; }
  const std::string& getResponseDataType() { return response_data_type_; }
  const std::string& getDataType() { return data_type_; }
  const std::string& getName() { return name_; }

private:
  void dropAllConnections();

  std::string name_;
  std::string md5sum_;
  std::string data_type_;
  std::string request_data_type_;
  std::string response_data_type_;
  ServiceCallbackHelperPtr helper_;

  V_ServiceClientLink client_links_;
  boost::mutex client_links_mutex_;

  bool dropped_;

  CallbackQueueInterface* callback_queue_;
  bool has_tracked_object_;
  VoidConstWPtr tracked_object_;
};
typedef boost::shared_ptr<ServicePublication> ServicePublicationPtr;

}

#endif // ROSCPP_SERVICE_PUBLICATION_H
