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

namespace rosbag {

class Bag;
class MessageInfo;

class MessageInstance : public ros::Message
{
    friend class Bag;

public:
    typedef boost::shared_ptr<MessageInstance> Ptr;
    typedef boost::shared_ptr<MessageInstance const> ConstPtr;

    MessageInstance(MessageInfo const& msg_info);
    virtual ~MessageInstance();

    // Message implementation

    virtual const std::string __getDataType()          const;
    virtual const std::string __getMD5Sum()            const;
    virtual const std::string __getMessageDefinition() const;

    static const std::string __s_getDataType();
    static const std::string __s_getMD5Sum();
    static const std::string __s_getMessageDefinition();

    uint32_t serializationLength() const;
    virtual uint8_t* serialize(uint8_t* writePtr, uint32_t) const;
    virtual uint8_t* deserialize(uint8_t* readPtr);

private:
    MessageInfo const* info_;
};

}

#endif
