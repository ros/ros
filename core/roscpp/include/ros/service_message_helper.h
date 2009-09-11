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

#ifndef ROSCPP_SERVICE_MESSAGE_HELPER_H
#define ROSCPP_SERVICE_MESSAGE_HELPER_H

#include "ros/forwards.h"
#include "ros/message.h"

namespace ros
{

class ServiceMessageHelper
{
public:
  virtual ~ServiceMessageHelper() {}
  virtual MessagePtr createRequest() = 0;
  virtual MessagePtr createResponse() = 0;

  virtual bool call(const MessagePtr& req, const MessagePtr& res) = 0;

  virtual std::string getMD5Sum() = 0;
  virtual std::string getDataType() = 0;
  virtual std::string getRequestDataType() = 0;
  virtual std::string getResponseDataType() = 0;
};
typedef boost::shared_ptr<ServiceMessageHelper> ServiceMessageHelperPtr;

template<class MReq, class MRes>
class ServiceMessageHelperT : public ServiceMessageHelper
{
public:
  typedef boost::shared_ptr<MReq> MReqPtr;
  typedef boost::shared_ptr<MRes> MResPtr;
  typedef boost::function<bool(MReq&, MRes&)> Callback;

  ServiceMessageHelperT(const Callback& callback)
  : callback_(callback)
  , md5sum_(MReq::__s_getServerMD5Sum())
  , data_type_(MReq::__s_getServiceDataType())
  , req_data_type_(MReq::__s_getDataType())
  , res_data_type_(MRes::__s_getDataType())
  {}

  ServiceMessageHelperT(const Callback& callback, const std::string& md5sum, const std::string& data_type, const std::string& req_data_type, const std::string& res_data_type)
  : callback_(callback)
  , md5sum_(md5sum)
  , data_type_(data_type)
  , req_data_type_(req_data_type)
  , res_data_type_(res_data_type)
  {}


  virtual MessagePtr createRequest() { return MessagePtr(new MReq); }
  virtual MessagePtr createResponse() { return MessagePtr(new MRes); }
  virtual bool call(const MessagePtr& req, const MessagePtr& res)
  {
    MReqPtr casted_req = boost::static_pointer_cast<MReq>(req);
    MResPtr casted_res = boost::static_pointer_cast<MRes>(res);
    return callback_(*casted_req, *casted_res);
  }

  virtual std::string getMD5Sum() { return md5sum_; }
  virtual std::string getDataType() { return data_type_; }
  virtual std::string getRequestDataType() { return req_data_type_; }
  virtual std::string getResponseDataType() { return res_data_type_; }

private:
  Callback callback_;
  std::string md5sum_;
  std::string data_type_;
  std::string req_data_type_;
  std::string res_data_type_;
};

}

#endif // ROSCPP_SERVICE_MESSAGE_HELPER_H
