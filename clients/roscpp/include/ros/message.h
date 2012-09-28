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

#ifndef ROSCPP_MESSAGE_H
#define ROSCPP_MESSAGE_H

// #warning You should not be using this file

#include "ros/macros.h"
#include "ros/assert.h"
#include <string>
#include <string.h>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <stdint.h>


#define ROSCPP_MESSAGE_HAS_DEFINITION

namespace ros
{

typedef std::map<std::string, std::string> M_string;

/**
 * \deprecated This base-class is deprecated in favor of a template-based serialization and traits system
 */
#if 0
class Message
{
public:
  typedef boost::shared_ptr<Message> Ptr;
  typedef boost::shared_ptr<Message const> ConstPtr;
  Message()
  {
  }
  virtual ~Message()
  {
  }
  virtual const std::string __getDataType() const = 0;
  virtual const std::string __getMD5Sum() const = 0;
  virtual const std::string __getMessageDefinition() const = 0;
  inline static std::string __s_getDataType() { ROS_BREAK(); return std::string(""); }
  inline static std::string __s_getMD5Sum() { ROS_BREAK(); return std::string(""); }
  inline static std::string __s_getMessageDefinition() { ROS_BREAK(); return std::string(""); }
  virtual uint32_t serializationLength() const = 0;
  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const = 0;
  virtual uint8_t *deserialize(uint8_t *read_ptr) = 0;
  uint32_t __serialized_length;

  boost::shared_ptr<M_string> __connection_header;
};

typedef boost::shared_ptr<Message> MessagePtr;
typedef boost::shared_ptr<Message const> MessageConstPtr;
#endif

#define SROS_SERIALIZE_PRIMITIVE(ptr, data) { memcpy(ptr, &data, sizeof(data)); ptr += sizeof(data); }
#define SROS_SERIALIZE_BUFFER(ptr, data, data_size) { if (data_size > 0) { memcpy(ptr, data, data_size); ptr += data_size; } }
#define SROS_DESERIALIZE_PRIMITIVE(ptr, data) { memcpy(&data, ptr, sizeof(data)); ptr += sizeof(data); }
#define SROS_DESERIALIZE_BUFFER(ptr, data, data_size) { if (data_size > 0) { memcpy(data, ptr, data_size); ptr += data_size; } }

}

#endif

