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
using boost::shared_ptr;
using ros::Exception;

namespace rosbag {

ChunkedFile::ChunkedFile() :
    file_(NULL),
    offset_(0),
    compressed_in_(0),
    unused_(NULL),
    nUnused_(0),
    stream_factory_(new StreamFactory(this))
{
}

ChunkedFile::~ChunkedFile() {
    close();
}

bool ChunkedFile::openReadWrite(string const& filename) { return open(filename, "r+b"); }
bool ChunkedFile::openWrite    (string const& filename) { return open(filename, "wb");  }
bool ChunkedFile::openRead     (string const& filename) { return open(filename, "rb");  }

bool ChunkedFile::open(string const& filename, string const& mode) {
    ROS_INFO("Opening file '%s' with mode '%s'", filename.c_str(), mode.c_str());

    // Check if file is already open
    if (file_) {
        ROS_ERROR("File already open: %s", filename_.c_str());
        return false;
    }

    // Open the file
    if (mode == "r+b") {
        // Read + write requires file to exists.  Create a new file if it doesn't exist.
        file_ = fopen(filename.c_str(), "r");
        if (file_ == NULL)
            file_ = fopen(filename.c_str(), "w+b");
        else {
            fclose(file_);
            file_ = fopen(filename.c_str(), "r+b");
        }
    }
    else
        file_ = fopen(filename.c_str(), mode.c_str());

    if (!file_) {
        ROS_FATAL("Error opening file: %s", filename.c_str());
        return false;
    }

    read_stream_  = shared_ptr<Stream>(new UncompressedStream(this));
    write_stream_ = shared_ptr<Stream>(new UncompressedStream(this));
    filename_     = filename;
    offset_       = ftell(file_);

    return true;
}

bool ChunkedFile::good() const {
    return feof(file_) == 0 && ferror(file_) == 0;
}

bool   ChunkedFile::isOpen()      const { return file_ != NULL; }
string ChunkedFile::getFileName() const { return filename_;     }

bool ChunkedFile::close() {
    if (!file_)
        return true;

    // Close any compressed stream by changing to uncompressed mode
    setWriteMode(compression::None);

    // Close the file
    int success = fclose(file_);
    if (success != 0) {
        ROS_FATAL("Error closing file: %s", filename_.c_str());
        return false;
    }

    file_ = NULL;
    filename_.clear();
    
    clearUnused();

    return true;
}

// Read/write modes

bool ChunkedFile::setWriteMode(CompressionType type) {
    if (!file_) {
        ROS_ERROR("Can't set compression mode before opening a file");
        return false;
    }

    if (type == write_stream_->getCompressionType())
        return true;

    try {
        write_stream_->stopWrite();
    }
    catch (Exception const& ex) {
        ROS_ERROR("Error stopping write mode: %s", ex.what());
        return false;
    }

    shared_ptr<Stream> stream = stream_factory_->getStream(type);

    try {
        stream->startWrite();
    }
    catch (Exception const& ex) {
        ROS_ERROR("Error starting write mode: %s", ex.what());
        return false;
    }

    write_stream_ = stream;

    return true;
}

bool ChunkedFile::setReadMode(CompressionType type) {
    if (!file_) {
        ROS_ERROR("Can't set compression mode before opening a file");
        return false;
    }

    if (type == read_stream_->getCompressionType())
        return true;

    try {
        read_stream_->stopRead();
    }
    catch (Exception const& ex) {
        ROS_ERROR("Error stopping read mode: %s", ex.what());
        return false;
    }

    shared_ptr<Stream> stream = stream_factory_->getStream(type);

    try {
        stream->startRead();
    }
    catch (Exception const& ex) {
        ROS_ERROR("Error starting read mode: %s", ex.what());
        return false;
    }

    read_stream_ = stream;

    return true;
}

bool ChunkedFile::seek(uint64_t offset, int origin) {
    if (!file_) {
        ROS_ERROR("Can't seek - file not open");
        return false;
    }

    setReadMode(compression::None);

    int success = fseek(file_, offset, origin);
    if (success != 0) {
        ROS_ERROR("Error seeking");
        return false;
    }

    offset_ = ftell(file_);

    return true;
}

uint64_t ChunkedFile::getOffset()            const { return offset_;        }
uint32_t ChunkedFile::getCompressedBytesIn() const { return compressed_in_; }

size_t ChunkedFile::write(string const& s)        { return write((void*) s.c_str(), s.size()); }
size_t ChunkedFile::write(void* ptr, size_t size) { return write_stream_->write(ptr, size);    }
size_t ChunkedFile::read(void* ptr, size_t size)  { return read_stream_->read(ptr, size);      }

bool ChunkedFile::truncate(uint64_t length) {
    int fd = fileno(file_);
    return ftruncate(fd, length) == 0;
}

//! \todo add error handling
string ChunkedFile::getline() {
    char buffer[1024];
    fgets(buffer, 1024, file_);
    string s(buffer);
    offset_ += s.size();

    return s;
}

void ChunkedFile::decompress(CompressionType compression, uint8_t* dest, unsigned int dest_len, uint8_t* source, unsigned int source_len) {
    stream_factory_->getStream(compression)->decompress(dest, dest_len, source, source_len);
}

void ChunkedFile::clearUnused() {
    unused_ = NULL;
    nUnused_ = 0;
}

}
