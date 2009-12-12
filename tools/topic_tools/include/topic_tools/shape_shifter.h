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

namespace topic_tools
{
 
  class ShapeShifterException : public ros::Exception
  {
  public:
    ShapeShifterException(const std::string& msg)
      : ros::Exception(msg)  {}
  };


// as on star trek, you've always got to be on the lookout for shape shifters
class ShapeShifter : public ros::Message
{
public:
  typedef boost::shared_ptr<ShapeShifter> Ptr;
  typedef boost::shared_ptr<ShapeShifter const> ConstPtr;

  std::string md5, datatype, msg_def;
  bool typed;

  uint8_t *msgBuf;
  uint32_t msgBufUsed, msgBufAlloc;
  std::string topic;
  
  ShapeShifter()
    :  ros::Message(), typed(false), msgBuf(NULL), msgBufUsed(0), msgBufAlloc(0) { }
  virtual ~ShapeShifter() { if (msgBuf) delete[] msgBuf;
                            msgBuf = NULL; msgBufAlloc = 0; }

  virtual const std::string __getDataType() const { return datatype; }
  virtual const std::string __getMD5Sum()   const { return md5; }
  virtual const std::string __getMessageDefinition()   const { return msg_def; }
  
  // You should never use a static method on a shape shifter
  static const std::string __s_getDataType() { ROS_ASSERT_MSG(0, "Tried to get static datatype of a ShapeShifter."); return "";}
  static const std::string __s_getMD5Sum()   { ROS_ASSERT_MSG(0, "Tried to get static md5sum of a ShapeShifter."); return "";}
  static const std::string __s_getMessageDefinition()   { ROS_ASSERT_MSG(0, "Tried to get static message definition of a ShapeShifter."); return "";}

  ros::Publisher advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_, bool latch=false) const;

  uint32_t serializationLength() const { return msgBufUsed; }

  virtual uint8_t *serialize(uint8_t *writePtr, uint32_t) const;

  virtual uint8_t *deserialize(uint8_t *readPtr);

  template<class M> 
  boost::shared_ptr<M> instantiate() const
  {
    if (!typed)
      throw ShapeShifterException("Tried to instantiate message from an untyped shapeshifter.");

    if (M::__s_getDataType() != __getDataType())
      throw ShapeShifterException("Tried to instantiate message without matching datatype.");

    if (M::__s_getMD5Sum() != __getMD5Sum())
      throw ShapeShifterException("Tried to instantiate message without matching md5sum.");
      
    boost::shared_ptr<M> p(new M());
    p->__serialized_length = msgBufUsed;
    p->deserialize(msgBuf);
    return p;
  }
  
};


  class ShapeShifterSubscriptionMessageHelper : public ros::SubscriptionMessageHelper
  {
  public:
    typedef boost::shared_ptr<ShapeShifter> MPtr;
    typedef boost::function<void (const MPtr&)> Callback;
    ShapeShifterSubscriptionMessageHelper(const Callback& callback)
      : callback_(callback)
    {}
    
    virtual ros::MessagePtr create()
    {
      typedef boost::remove_const<ShapeShifter>::type NonConstType;
      NonConstType* msg = new NonConstType;
      return ros::MessagePtr(msg);
    }
    
    virtual void call(const ros::MessagePtr& msg)
    {
      MPtr casted_msg = boost::static_pointer_cast<ShapeShifter>(msg);
      callback_(casted_msg);
    }
    
    virtual std::string getMD5Sum() { return "*"; }
    virtual std::string getDataType() { return "*"; }
    
  private:
    Callback callback_;
  };
  
}

namespace ros{

// Template specialization of SubscribeOptions for ShapeShifter
template<>
void SubscribeOptions::init<topic_tools::ShapeShifter>(const std::string& _topic, uint32_t _queue_size,
                                                       const boost::function<void (const boost::shared_ptr<topic_tools::ShapeShifter>&)>& _callback)
{
  topic = _topic;
  queue_size = _queue_size;
  md5sum = "*";
  datatype = "*";
  helper = SubscriptionMessageHelperPtr(new topic_tools::ShapeShifterSubscriptionMessageHelper(_callback));
}

}

#endif

