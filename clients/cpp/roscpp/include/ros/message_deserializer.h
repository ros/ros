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

#ifndef ROSCPP_MESSAGE_DESERIALIZER_H
#define ROSCPP_MESSAGE_DESERIALIZER_H

#include "forwards.h"
#include "common.h"

#include <ros/serialized_message.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_array.hpp>

namespace ros
{

class SubscriptionCallbackHelper;
typedef boost::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

class ROSCPP_DECL MessageDeserializer
{
public:
  MessageDeserializer(const SubscriptionCallbackHelperPtr& helper, const SerializedMessage& m, const boost::shared_ptr<M_string>& connection_header);

  VoidConstPtr deserialize();
  const boost::shared_ptr<M_string>& getConnectionHeader() { return connection_header_; }

private:
  SubscriptionCallbackHelperPtr helper_;
  SerializedMessage serialized_message_;
  boost::shared_ptr<M_string> connection_header_;

  boost::mutex mutex_;
  VoidConstPtr msg_;
};
typedef boost::shared_ptr<MessageDeserializer> MessageDeserializerPtr;

}

#endif // ROSCPP_MESSAGE_DESERIALIZER_H

