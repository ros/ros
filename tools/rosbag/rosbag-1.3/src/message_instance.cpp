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
#include "rosbag/message_info.h"
#include "rosbag/bag.h"

using std::string;
using ros::Time;

namespace rosbag {

MessageInstance::MessageInstance(MessageInfo const& info) : ros::Message(), info_(&info) {
    __serialized_length = info_->bag_->record_buffer_.getSize();
    deserialize(info_->bag_->record_buffer_.getData());
}

MessageInstance::~MessageInstance() { }

const string MessageInstance::__getDataType()          const { return info_->getDataType();          }
const string MessageInstance::__getMD5Sum()            const { return info_->getMD5Sum();            }
const string MessageInstance::__getMessageDefinition() const { return info_->getMessageDefinition(); }

const string MessageInstance::__s_getDataType()          { return "*"; }
const string MessageInstance::__s_getMD5Sum()            { return "*"; }
const string MessageInstance::__s_getMessageDefinition() { ROS_ASSERT_MSG(0, "Tried to get static message definition of a MessageInstance."); return "";}

uint32_t MessageInstance::serializationLength() const { return info_->bag_->instantiate_buffer_.getSize(); }

uint8_t* MessageInstance::serialize(uint8_t* write_ptr, uint32_t) const {
	memcpy(write_ptr, info_->bag_->instantiate_buffer_.getData(), info_->bag_->instantiate_buffer_.getSize());
	return write_ptr + info_->bag_->instantiate_buffer_.getSize();
}

uint8_t* MessageInstance::deserialize(uint8_t* read_ptr) {
	info_->bag_->instantiate_buffer_.setSize(__serialized_length);
	memcpy(info_->bag_->instantiate_buffer_.getData(), read_ptr, __serialized_length);
	return NULL;
}

}
