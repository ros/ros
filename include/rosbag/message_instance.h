/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*********************************************************************/

#ifndef ROSBAG_MESSAGE_INSTANCE_H
#define ROSBAG_MESSAGE_INSTANCE_H

#include <ros/message.h>
#include <ros/message_traits.h>
#include <ros/ros.h>
#include <ros/time.h>

#include "rosbag/bag.h"

namespace rosbag {

class MessageInstance : public ros::Message
{
    friend class Bag;

public:
    typedef boost::shared_ptr<MessageInstance> Ptr;
    typedef boost::shared_ptr<MessageInstance const> ConstPtr;

    MessageInstance(const TopicInfo& info, const IndexEntry& index, Bag& bag);
    virtual ~MessageInstance();

    const std::string& getTopic()    const;
    const std::string& getDatatype() const;
    const std::string& getMd5sum()   const;
    const std::string& getDef()      const;
    const ros::Time&   getTime()     const;

    template<class T>
    boost::shared_ptr<T const> instantiate() const;

    void instantiateMessage();

    // Message implementation

    virtual const std::string __getDataType()          const { return getDatatype(); }
    virtual const std::string __getMD5Sum()            const { return getMd5sum();   }
    virtual const std::string __getMessageDefinition() const { return getDef();      }

    static const std::string __s_getDataType()          { return "*"; }
    static const std::string __s_getMD5Sum()            { return "*"; }
    static const std::string __s_getMessageDefinition() { ROS_ASSERT_MSG(0, "Tried to get static message definition of a MessageInstance."); return "";}

    uint32_t serializationLength() const { return msg_buf_used_; }
    virtual uint8_t* serialize(uint8_t* writePtr, uint32_t) const;
    virtual uint8_t* deserialize(uint8_t* readPtr);

private:
    const TopicInfo&  info_;
    const IndexEntry& index_;
    Bag&              bag_;

    uint8_t* msg_buf_;
    uint32_t msg_buf_used_;
    uint32_t msg_buf_alloc_;
};

//! Comparator to sort MessageInstances by timestamp
struct MessageInstanceCompare
{
    bool operator()(const MessageInstance& a, const MessageInstance& b) const {
        return a.getTime() < b.getTime();
    }
};

template<class T>
boost::shared_ptr<T const> MessageInstance::instantiate() const {
    if (ros::message_traits::MD5Sum<T>::value() != getMd5sum())
        return boost::shared_ptr<T const>();

    if (bag_.version_ == 103)
    	bag_.readMessageDataRecord103(info_.topic, index_.chunk_pos, index_.offset);
    else if (bag_.version_ == 102)
    	bag_.readMessageDataRecord102(info_.topic, index_.chunk_pos);
    else
    	ROS_FATAL("Unhandled version: %d", bag_.version_);

    return bag_.instantiateBuffer<T>();
}

}

#endif
