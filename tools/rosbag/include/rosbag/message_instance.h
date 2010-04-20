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

#include "rosbag/structures.h"

namespace rosbag {

class Bag;

class MessageInstance
{
    friend class Bag;
  
public:
    MessageInstance(TopicInfo const* info, IndexEntry const& index, Bag& bag);
  
    std::string const& getTopic()             const;
    std::string const& getDataType()          const;
    std::string const& getMD5Sum()            const;
    std::string const& getMessageDefinition() const;
    ros::Time const&   getTime()              const;

    bool        getLatching() const;
    std::string getCallerId() const;
  
    template<class T>
    boost::shared_ptr<T const> instantiate() const;
  
    template<typename Stream>
    void write(Stream& stream) const;

    uint32_t size() const;

private:
    TopicInfo const* topic_info_;
    IndexEntry const index_entry_;
    Bag*             bag_;
};

ros::AdvertiseOptions createAdvertiseOptions(MessageInstance const&, uint32_t queue_size);

} // namespace rosbag

namespace ros
{
namespace message_traits
{

template<>
struct MD5Sum<rosbag::MessageInstance>
{
    static const char* value(const rosbag::MessageInstance& m) { return m.getMD5Sum().c_str(); }
};

template<>
struct DataType<rosbag::MessageInstance>
{
    static const char* value(const rosbag::MessageInstance& m) { return m.getDataType().c_str(); }
};

template<>
struct Definition<rosbag::MessageInstance>
{
    static const char* value(const rosbag::MessageInstance& m) { return m.getMessageDefinition().c_str(); }
};

} // namespace message_traits

namespace serialization
{

template<>
struct Serializer<rosbag::MessageInstance>
{
    template<typename Stream> inline static void write(Stream& stream, const rosbag::MessageInstance& m) {
        m.write(stream);
    }

    inline static uint32_t serializedLength(const rosbag::MessageInstance& m) {
        return m.size();
    }
};

} // namespace serialization

} //namespace ros

// I really don't like having to do this
#include "rosbag/bag.h"

template<class T>
boost::shared_ptr<T const> rosbag::MessageInstance::instantiate() const {
    if (ros::message_traits::MD5Sum<T>::value() != getMD5Sum())
        return boost::shared_ptr<T const>();

    return bag_->instantiateBuffer<T>(index_entry_);
}

template<typename Stream>
void rosbag::MessageInstance::write(Stream& stream) const {
    bag_->readMessageDataIntoStream(index_entry_, stream);
}

#endif
