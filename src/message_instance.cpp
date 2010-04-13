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

MessageInstance::MessageInstance(TopicInfo const& info, IndexEntry const& index, Bag& bag) :
	ros::Message(), info_(info), index_(index), bag_(bag), msg_buf_(NULL), msg_buf_used_(0), msg_buf_alloc_(0) { }

MessageInstance::~MessageInstance() {
	delete[] msg_buf_;
}

string const& MessageInstance::getTopic()    const { return info_.topic;    }
string const& MessageInstance::getDatatype() const { return info_.datatype; }
string const& MessageInstance::getMd5sum()   const { return info_.md5sum;   }
string const& MessageInstance::getDef()      const { return info_.msg_def;  }
Time const&   MessageInstance::getTime()     const { return index_.time;    }

uint8_t* MessageInstance::serialize(uint8_t* write_ptr, uint32_t) const {
	memcpy(write_ptr, msg_buf_, msg_buf_used_);

	return write_ptr + msg_buf_used_;
}

uint8_t* MessageInstance::deserialize(uint8_t* read_ptr) {
	if (__serialized_length > msg_buf_alloc_) {
		delete[] msg_buf_;
		msg_buf_ = new uint8_t[__serialized_length];
		msg_buf_alloc_ = __serialized_length;
	}

	msg_buf_used_ = __serialized_length;
	memcpy(msg_buf_, read_ptr, __serialized_length);

	return NULL;
}

void MessageInstance::instantiateMessage() {
    switch (bag_.version_) {
        case 103: bag_.readMessageDataRecord103(info_.topic, index_.chunk_pos, index_.offset); break;
        case 102: bag_.readMessageDataRecord102(info_.topic, index_.chunk_pos);                break;
        default:  ROS_FATAL("Unhandled version: %d", bag_.version_); return;
    }

    __serialized_length = bag_.record_buffer_.getSize();
    deserialize(bag_.record_buffer_.getData());
}

}
