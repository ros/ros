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

#ifndef ROSCPP_SERVICE_CLIENT_OPTIONS_H
#define ROSCPP_SERVICE_CLIENT_OPTIONS_H

#include "ros/forwards.h"
#include "common.h"
#include "ros/service_traits.h"

namespace ros
{

/**
 * \brief Encapsulates all options available for creating a ServiceClient
 */
struct ROSCPP_DECL ServiceClientOptions
{
  ServiceClientOptions()
  {
  }

  /*
   * \brief Constructor
   * \param _service Name of the service to connect to
   * \param _md5sum md5sum of the service
   * \param _persistent Whether or not to keep the connection open to the service for future calls
   * \param _header Any extra values to be passed along in the connection header
   */
  ServiceClientOptions(const std::string& _service, const std::string& _md5sum, bool _persistent, const M_string& _header)
  : service(_service)
  , md5sum(_md5sum)
  , persistent(_persistent)
  , header(_header)
  {
  }

  /*
   * \brief Templated helper method, preventing you from needing to manually get the service md5sum
   * \param MReq [template] Request message type
   * \param MRes [template] Response message type
   * \param _service Name of the service to connect to
   * \param _persistent Whether or not to keep the connection open to the service for future calls
   * \param _header Any extra values to be passed along in the connection header
   */
  template <class MReq, class MRes>
  void init(const std::string& _service, bool _persistent, const M_string& _header)
  {
    namespace st = service_traits;

    service = _service;
    md5sum = st::md5sum<MReq>();
    persistent = _persistent;
    header = _header;
  }

  /*
   * \brief Templated helper method, preventing you from needing to manually get the service md5sum
   * \param Service [template] Service type
   * \param _service Name of the service to connect to
   * \param _persistent Whether or not to keep the connection open to the service for future calls
   * \param _header Any extra values to be passed along in the connection header
   */
  template <class Service>
  void init(const std::string& _service, bool _persistent, const M_string& _header)
  {
    namespace st = service_traits;

    service = _service;
    md5sum = st::md5sum<Service>();
    persistent = _persistent;
    header = _header;
  }

  std::string service;                                                      ///< Service to connect to
  std::string md5sum;                                                       ///< Service md5sum
  bool persistent;                                                          ///< Whether or not the connection should persist
  M_string header;                                                          ///< Extra key/value pairs to add to the connection header
};


}

#endif
