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

ZLIBStream::ZLIBStream(ChunkedFile* file) : Stream(file) { }

CompressionType ZLIBStream::getCompressionType() const {
    return compression::ZLIB;
}

size_t ZLIBStream::write(void* ptr, size_t size) {
    /*
       // compress until end of file
       do {
           strm.avail_in = fread(in, 1, CHUNK, source);
           if (ferror(source)) {
               (void)deflateEnd(&strm);
               return Z_ERRNO;
           }
           flush = feof(source) ? Z_FINISH : Z_NO_FLUSH;
           strm.next_in = in;

           // run deflate() on input until output buffer not full, finish compression if all of source has been read in
           do {
               strm.avail_out = CHUNK;
               strm.next_out = out;
               ret = deflate(&strm, flush);    // no bad return value
               assert(ret != Z_STREAM_ERROR);  // state not clobbered
               have = CHUNK - strm.avail_out;
               if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
                   (void) deflateEnd(&strm);
                   return Z_ERRNO;
               }
           }
           while (strm.avail_out == 0);
           ROS_ASSERT(strm.avail_in == 0);     // all input will be used

           // done when last data in file processed
       }
       while (flush != Z_FINISH);
       ROS_ASSERT(ret == Z_STREAM_END);    // stream will be complete

       // clean up and return
       (void) deflateEnd(&strm);
       */
    return 0;
}

size_t ZLIBStream::read(void* ptr, size_t size) {
    return 0;
}

void ZLIBStream::decompress(uint8_t* dest, unsigned int dest_len, uint8_t* source, unsigned int source_len) {
}

void ZLIBStream::startWrite() {
    /*
       int ret, flush;
       unsigned have;
       z_stream strm;
       unsigned char in[CHUNK];
       unsigned char out[CHUNK];

       strm.zalloc = Z_NULL;
       strm.zfree = Z_NULL;
       strm.opaque = Z_NULL;
       ret = deflateInit(&strm, level);
       if (ret != Z_OK)
           return ret;

       //! \todo
       */
    }

void ZLIBStream::stopWrite() {
}

void ZLIBStream::startRead() {
}

void ZLIBStream::stopRead() {
}

}
