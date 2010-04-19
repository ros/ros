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

#ifndef ROSBAG_MESSAGE_INFO_H
#define ROSBAG_MESSAGE_INFO_H

#include <ros/message.h>
#include <ros/message_traits.h>
#include <ros/ros.h>
#include <ros/time.h>

#include "rosbag/bag.h"

namespace rosbag {

class MessageInstance;

class MessageInfo
{
    friend class Bag;
    friend class MessageInstance;

public:
    MessageInfo(TopicInfo const* info, IndexEntry const& index, Bag& bag);

    std::string const& getTopic()             const;
    std::string const& getDataType()          const;
    std::string const& getMD5Sum()            const;
    std::string const& getMessageDefinition() const;
    ros::Time const&   getTime()              const;

    template<class T>
    boost::shared_ptr<T const> instantiate() const;

    boost::shared_ptr<MessageInstance> instantiateInstance() const;

private:
    TopicInfo const* topic_info_;
    IndexEntry const index_entry_;
    Bag*             bag_;
};

template<class T>
boost::shared_ptr<T const> MessageInfo::instantiate() const {
    if (ros::message_traits::MD5Sum<T>::value() != getMD5Sum())
        return boost::shared_ptr<T const>();

    switch (bag_->version_) {
    case 200: bag_->readMessageDataRecord200(topic_info_->topic, index_entry_.chunk_pos, index_entry_.offset); break;
    case 102: bag_->readMessageDataRecord102(topic_info_->topic, index_entry_.chunk_pos); break;
    default:  ROS_FATAL("Unhandled version: %d", bag_->version_);
    }

    return bag_->instantiateBuffer<T>();
}

}

#endif
