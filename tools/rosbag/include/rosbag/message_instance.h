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

//! A class pointing into a bag file
/*!
 *  The MessageInstance class itself is fairly light weight.  It
 *  simply contains a pointer to a bag-file and the index_entry
 *  necessary to get access to the corresponding data.
 *
 *  It adheres to the necessary ros::message_traits to be directly
 *  serializable.
 */
class MessageInstance
{
    friend class View;
  
public:
    std::string const& getTopic()             const;
    std::string const& getDataType()          const;
    std::string const& getMD5Sum()            const;
    std::string const& getMessageDefinition() const;
    ros::Time   const& getTime()              const;

    // Additional useful informatio from header
    bool               getLatching()          const;
    std::string        getCallerId()          const;
  

    //! Templated call to instantiate a message
    /*!
     * returns NULL pointer if incompatible
     */
    template<class T>
    boost::shared_ptr<T const> instantiate()  const;
  
    //! Write serialized message contents out to a stream
    template<typename Stream>
    void write(Stream& stream) const;

    //! Size of serialized message
    uint32_t size() const;

private:
    MessageInstance(TopicInfo const* info, IndexEntry const& index, Bag& bag);
 
    TopicInfo const* topic_info_;
    IndexEntry const index_entry_;
    Bag*             bag_;
};


//! Helper function to create AdvertiseOptions from a MessageInstance
/*!
 *  param msg         The Message instance for which to generate adveritse options
 *  param queue_size  The size of the outgoing queue
 */
ros::AdvertiseOptions createAdvertiseOptions(MessageInstance const& msg, uint32_t queue_size);

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
    template<typename Stream>
    inline static void write(Stream& stream, const rosbag::MessageInstance& m) {
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
