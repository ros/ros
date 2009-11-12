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


#include "ros/message_deserializer.h"
#include <ros/console.h>

namespace ros
{

MessageDeserializer::MessageDeserializer(const SubscriptionMessageHelperPtr& helper, const boost::shared_array<uint8_t>& buffer, size_t num_bytes, const boost::shared_ptr<M_string>& connection_header)
: helper_(helper)
, buffer_(buffer)
, num_bytes_(num_bytes)
, connection_header_(connection_header)
{

}

MessagePtr MessageDeserializer::deserialize()
{
  boost::mutex::scoped_lock lock(mutex_);

  if (msg_)
  {
    return msg_;
  }

  if (!buffer_)
  {
    // If the buffer has been reset it means we tried to deserialize and failed
    return MessagePtr();
  }

  msg_ = helper_->create();
  msg_->__serialized_length = num_bytes_;
  msg_->__connection_header = connection_header_;

  try
  {
    msg_->deserialize(buffer_.get());
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Exception thrown when deserializing message of length [%d] from [%s]: %s", num_bytes_, (*connection_header_)["callerid"].c_str(), e.what());
  }

  buffer_.reset();

  return msg_;
}

}
