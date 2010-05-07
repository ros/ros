/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
********************************************************************/

#ifndef TOPIC_TOOLS_SHAPE_SHIFTER_H
#define TOPIC_TOOLS_SHAPE_SHIFTER_H

#include "ros/ros.h"
#include "ros/console.h"
#include "ros/assert.h"
#include <vector>
#include <string>
#include <string.h>

#include <ros/message_traits.h>

namespace topic_tools
{
 
class ShapeShifterException : public ros::Exception
{
public:
  ShapeShifterException(const std::string& msg)
    : ros::Exception(msg)  {}
};


class ShapeShifter : public ros::Message
{
public:
  typedef boost::shared_ptr<ShapeShifter> Ptr;
  typedef boost::shared_ptr<ShapeShifter const> ConstPtr;

  static bool uses_old_API_;

  // Constructor and destructor
  ShapeShifter();
  virtual ~ShapeShifter();

  // Helpers for inspecting shapeshifter
  std::string const& getDataType()          const;
  std::string const& getMD5Sum()            const;
  std::string const& getMessageDefinition() const;

  void morph(const std::string& md5sum, const std::string& datatype, const std::string& msg_def);

  // Helper for advertising
  ros::Publisher advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_, bool latch=false) const;

  //! Call to try instantiating as a particular type
  template<class M> 
  boost::shared_ptr<M> instantiate() const;

  //! Write serialized message contents out to a stream
  template<typename Stream>
  void write(Stream& stream) const;

  template<typename Stream>
  void read(Stream& stream);

  //! Return the size of the serialized message
  uint32_t size() const;

  // Deprecated old-style API
  ROSCPP_DEPRECATED virtual const std::string __getDataType() const;
  ROSCPP_DEPRECATED virtual const std::string __getMD5Sum() const;
  ROSCPP_DEPRECATED virtual const std::string __getMessageDefinition() const;
  
  ROSCPP_DEPRECATED static const std::string __s_getDataType();
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum();
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition();

  ROSCPP_DEPRECATED uint32_t serializationLength() const { return msgBufUsed; }
  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *writePtr, uint32_t) const;
  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *readPtr);

private:

  std::string md5, datatype, msg_def;
  bool typed;

  uint8_t *msgBuf;
  uint32_t msgBufUsed;
  uint32_t msgBufAlloc;
  
};
  
}


// Message traits allow shape shifter to work with the new serialization API
namespace ros {
namespace message_traits {

template<>
struct MD5Sum<topic_tools::ShapeShifter>
{
  static const char* value(const topic_tools::ShapeShifter& m) { return m.getMD5Sum().c_str(); }

  // Used statically, a shapeshifter appears to be of any type
  static const char* value() { return "*"; }
};

template<>
struct DataType<topic_tools::ShapeShifter>
{
  static const char* value(const topic_tools::ShapeShifter& m) { return m.getDataType().c_str(); }

  // Used statically, a shapeshifter appears to be of any type
  static const char* value() { return "*"; }
};

template<>
struct Definition<topic_tools::ShapeShifter>
{
  static const char* value(const topic_tools::ShapeShifter& m) { return m.getMessageDefinition().c_str(); }
};

} // namespace message_traits


namespace serialization
{

template<>
struct Serializer<topic_tools::ShapeShifter>
{
  template<typename Stream>
  inline static void write(Stream& stream, const topic_tools::ShapeShifter& m) {
    m.write(stream);
  }

  template<typename Stream>
  inline static void read(Stream& stream, topic_tools::ShapeShifter& m)
  {
    m.read(stream);
  }

  inline static uint32_t serializedLength(const topic_tools::ShapeShifter& m) {
    return m.size();
  }
};


template<>
struct PreDeserialize<topic_tools::ShapeShifter>
{
  static void notify(const PreDeserializeParams<topic_tools::ShapeShifter>& params)
  {
    std::string md5      = (*params.connection_header)["md5sum"];
    std::string datatype = (*params.connection_header)["type"];
    std::string msg_def  = (*params.connection_header)["message_definition"];

    params.message->morph(md5, datatype, msg_def);
  }
};

} // namespace serialization

} //namespace ros



// Template implementations:

namespace topic_tools
{

template<class M> 
boost::shared_ptr<M> ShapeShifter::instantiate() const
{
  if (!typed)
    throw ShapeShifterException("Tried to instantiate message from an untyped shapeshifter.");
  
  if (ros::message_traits::datatype<M>() != getDataType())
    throw ShapeShifterException("Tried to instantiate message without matching datatype.");
  
  if (ros::message_traits::md5sum<M>() != getMD5Sum())
    throw ShapeShifterException("Tried to instantiate message without matching md5sum.");
  
  boost::shared_ptr<M> p(new M());

  ros::assignSubscriptionConnectionHeader<M>(p.get(), __connection_header);

  ros::serialization::IStream s(msgBuf, msgBufUsed);
  ros::serialization::deserialize(s, *p);

  return p;
}


template<typename Stream>
void ShapeShifter::write(Stream& stream) const {
  if (msgBufUsed > 0)
    memcpy(stream.advance(msgBufUsed), msgBuf, msgBufUsed);
}

template<typename Stream>
void ShapeShifter::read(Stream& stream)
{
  stream.getLength();
  stream.getData();
    
  // stash this message in our buffer
  if (stream.getLength() > msgBufAlloc)
  {
    delete[] msgBuf;
    msgBuf = new uint8_t[stream.getLength()];
    msgBufAlloc = stream.getLength();
  }
  msgBufUsed = stream.getLength();
  memcpy(msgBuf, stream.getData(), stream.getLength());
}

} // namespace topic_tools


#endif

