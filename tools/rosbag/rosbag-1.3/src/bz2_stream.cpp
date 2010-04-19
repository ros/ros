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

BZ2Stream::BZ2Stream(ChunkedFile* file) :
    Stream(file),
    verbosity_(0),
    block_size_100k_(9),
    work_factor_(30),
    bzfile_(NULL),
    bzerror_(0)
{ }

CompressionType BZ2Stream::getCompressionType() const {
    return compression::BZ2;
}

void BZ2Stream::startWrite() {
    bzfile_ = BZ2_bzWriteOpen(&bzerror_, getFilePointer(), block_size_100k_, verbosity_, work_factor_);

    switch (bzerror_) {
        case BZ_OK: break;
        default: {
            BZ2_bzWriteClose(&bzerror_, bzfile_, 0, NULL, NULL);
            throw Exception("Error opening file for writing compressed stream");
        }
    }

    setCompressedIn(0);
}

size_t BZ2Stream::write(void* ptr, size_t size) {
    BZ2_bzWrite(&bzerror_, bzfile_, ptr, size);

    switch (bzerror_) {
    case BZ_IO_ERROR: ROS_ERROR("BZ_IO_ERROR: error writing the compressed file"); return false;
    }

    setCompressedIn(getCompressedIn() + size);
    return true;
}

void BZ2Stream::stopWrite() {
    unsigned int nbytes_in;
    unsigned int nbytes_out;
    BZ2_bzWriteClose(&bzerror_, bzfile_, 0, &nbytes_in, &nbytes_out);

    switch (bzerror_) {
        case BZ_IO_ERROR: throw Exception("BZ_IO_ERROR");
    }

    advanceOffset(nbytes_out);
    setCompressedIn(0);
}

void BZ2Stream::startRead() {
    bzfile_ = BZ2_bzReadOpen(&bzerror_, getFilePointer(), verbosity_, 0, getUnused(), getUnusedLength());

    switch (bzerror_) {
        case BZ_OK: break;
        default: {
            checkError();
            BZ2_bzReadClose(&bzerror_, bzfile_);
            throw Exception("Error opening file for reading compressed stream");
        }
    }

    clearUnused();
}

size_t BZ2Stream::read(void* ptr, size_t size) {
    BZ2_bzRead(&bzerror_, bzfile_, ptr, size);

    advanceOffset(size);

    switch (bzerror_) {
    case BZ_OK:               return size;
    case BZ_STREAM_END:
        if (getUnused() || getUnusedLength() > 0)
            ROS_ERROR("unused data already available");
        else {
            char* unused;
            int nUnused;
            BZ2_bzReadGetUnused(&bzerror_, bzfile_, (void**) &unused, &nUnused);
            setUnused(unused);
            setUnusedLength(nUnused);
        }
        return size;
    case BZ_IO_ERROR:         ROS_ERROR("BZ_IO_ERROR: error reading from compressed stream");                                break;
    case BZ_UNEXPECTED_EOF:   ROS_ERROR("BZ_UNEXPECTED_EOF: compressed stream ended before logical end-of-stream detected"); break;
    case BZ_DATA_ERROR:       ROS_ERROR("BZ_DATA_ERROR: data integrity error detected in compressed stream");                break;
    case BZ_DATA_ERROR_MAGIC: ROS_ERROR("BZ_DATA_ERROR_MAGIC: stream does not begin with requisite header bytes");           break;
    case BZ_MEM_ERROR:        ROS_ERROR("BZ_MEM_ERROR: insufficient memory available");                                      break;
    }

    return 0;
}

void BZ2Stream::stopRead() {
    BZ2_bzReadClose(&bzerror_, bzfile_);

    switch (bzerror_ == BZ_IO_ERROR) {
        case BZ_IO_ERROR: throw Exception("BZ_IO_ERROR");
    }
}

void BZ2Stream::decompress(uint8_t* dest, unsigned int dest_len, uint8_t* source, unsigned int source_len) {
    int result = BZ2_bzBuffToBuffDecompress((char*) dest, &dest_len, (char*) source, source_len, 0, verbosity_);

    switch (result) {
    case BZ_OK:               break;
    case BZ_CONFIG_ERROR:     ROS_ERROR("library has been mis-compiled"); break;
    case BZ_PARAM_ERROR:      ROS_ERROR("dest is NULL or destLen is NULL or small != 0 && small != 1 or verbosity < 0 or verbosity > 4"); break;
    case BZ_MEM_ERROR:        ROS_ERROR("insufficient memory is available"); break;
    case BZ_OUTBUFF_FULL:     ROS_ERROR("size of the compressed data exceeds *destLen"); break;
    case BZ_DATA_ERROR:       ROS_ERROR("data integrity error was detected in the compressed data"); break;
    case BZ_DATA_ERROR_MAGIC: ROS_ERROR("compressed data doesn't begin with the right magic bytes"); break;
    case BZ_UNEXPECTED_EOF:   ROS_ERROR("compressed data ends unexpectedly"); break;
    }
}

void BZ2Stream::checkError() const {
    switch (bzerror_) {
    case BZ_OK:               ROS_INFO("BZ_OK");                break;
    case BZ_STREAM_END:       ROS_INFO("BZ_STREAM_END");        break;
    case BZ_PARAM_ERROR:      ROS_ERROR("BZ_PARAM_ERROR");      break;
    case BZ_IO_ERROR:         ROS_ERROR("BZ_IO_ERROR");         break;
    case BZ_UNEXPECTED_EOF:   ROS_ERROR("BZ_UNEXPECTED_EOF");   break;
    case BZ_DATA_ERROR:       ROS_ERROR("BZ_DATA_ERROR");       break;
    case BZ_DATA_ERROR_MAGIC: ROS_ERROR("BZ_DATA_ERROR_MAGIC"); break;
    case BZ_MEM_ERROR:        ROS_ERROR("BZ_MEM_ERROR");        break;
    }
}

}
