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
********************************************************************/

#include "rosbag/chunked_file.h"

#include <iostream>

#include <ros/ros.h>

using std::string;
using ros::Exception;

namespace rosbag {

UncompressedStream::UncompressedStream() { }

size_t UncompressedStream::write(void* ptr, size_t size) {
    size_t result = fwrite(ptr, 1, size, file_);
    if (result != size)
        return false;

    offset_ += size;
    return true;
}

size_t UncompressedStream::read(void* ptr, size_t size) {
    if (nUnused_ > 0) {
        // We have unused data from the last compressed read
        if ((size_t) nUnused_ == size) {
            // Copy the unused data into the buffer
            memcpy(ptr, unused_, nUnused_);

            clearUnusedBZ2();
        }
        else if ((size_t) nUnused_ < size) {
            // Copy the unused data into the buffer
            memcpy(ptr, unused_, nUnused_);

            // Still have data to read
            size -= nUnused_;

            // Read the remaining data from the file
            int result = fread((char*) ptr + nUnused_, 1, size, file_);
            if ((size_t) result != size) {
                ROS_ERROR("Error reading from file: wanted %zd bytes, read %d bytes", size, result);
                return nUnused_ + result;
            }

            offset_ += size;

            clearUnusedBZ2();
        }
        else {
            // nUnused_ > size
            memcpy(ptr, unused_, size);

            unused_ += size;
            nUnused_ -= size;
        }

        return size;
    }

    // No unused data - read from stream
    int result = fread(ptr, 1, size, file_);
    if ((size_t) result != size)
        ROS_ERROR("Error reading from file: wanted %zd bytes, read %d bytes", size, result);

    offset_ += size;

    return result;
}

void UncompressedStream::decompress(uint8_t* dest, unsigned int dest_len, uint8_t* source, unsigned int source_len) {
    if (dest_len < source_len)
        throw Exception("dest_len not large enough");

    memcpy(dest, source, source_len);
}

}
