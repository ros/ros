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

#include "rosbag/message_instance.h"

using std::string;
using ros::Time;

namespace rosbag {

MessageInstance::MessageInstance(TopicInfo const* topic_info, IndexEntry const& index_entry, Bag& bag) :
	topic_info_(topic_info), index_entry_(index_entry), bag_(&bag) { }

string const& MessageInstance::getTopic()             const { return topic_info_->topic;    }
string const& MessageInstance::getDataType()          const { return topic_info_->datatype; }
string const& MessageInstance::getMD5Sum()            const { return topic_info_->md5sum;   }
string const& MessageInstance::getMessageDefinition() const { return topic_info_->msg_def;  }
Time const&   MessageInstance::getTime()              const { return index_entry_.time;     }

// @todo: this should cache the header
bool MessageInstance::getLatching() const {
    ros::Header header = bag_->readMessageDataHeader(index_entry_);
    ros::M_string& fields = *header.getValues();
    
    ros::M_string::iterator latch_iter = fields.find(string("latching"));
    if (latch_iter != fields.end() && latch_iter->second != string("0"))
        return true;
    else
        return false;
}

std::string MessageInstance::getCallerId() const {
    ros::Header header = bag_->readMessageDataHeader(index_entry_);
    ros::M_string& fields = *header.getValues();

    ros::M_string::iterator callerid_iter = fields.find(string("callerid"));
    if (callerid_iter != fields.end())
        return callerid_iter->second;
    else
        return std::string("");
}

uint32_t MessageInstance::size() const {
    return bag_->readMessageDataSize(index_entry_);
}

ros::AdvertiseOptions createAdvertiseOptions(MessageInstance const& m, uint32_t queue_size) {
    return ros::AdvertiseOptions(m.getTopic(), queue_size, m.getMD5Sum(), m.getDataType(), m.getMessageDefinition());
}

}
