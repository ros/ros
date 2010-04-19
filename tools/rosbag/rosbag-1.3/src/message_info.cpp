// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rosbag/message_info.h"
#include "rosbag/message_instance.h"

using std::string;
using ros::Time;

namespace rosbag {

MessageInfo::MessageInfo(TopicInfo const* topic_info, IndexEntry const& index_entry, Bag& bag) :
	topic_info_(topic_info), index_entry_(index_entry), bag_(&bag) { }

string const& MessageInfo::getTopic()             const { return topic_info_->topic;    }
string const& MessageInfo::getDataType()          const { return topic_info_->datatype; }
string const& MessageInfo::getMD5Sum()            const { return topic_info_->md5sum;   }
string const& MessageInfo::getMessageDefinition() const { return topic_info_->msg_def;  }
Time const&   MessageInfo::getTime()              const { return index_entry_.time;    }

MessageInstance::Ptr MessageInfo::instantiateInstance() const {
	// Read message into the bag record buffer
    switch (bag_->version_) {
        case 103: bag_->readMessageDataRecord103(topic_info_->topic, index_entry_.chunk_pos, index_entry_.offset); break;
        case 102: bag_->readMessageDataRecord102(topic_info_->topic, index_entry_.chunk_pos);                break;
        default:  ROS_FATAL("Unhandled version: %d", bag_->version_); return boost::shared_ptr<MessageInstance>();
    }

    return MessageInstance::Ptr(new MessageInstance(*this));
}

}
