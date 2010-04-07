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

ChunkedFile::ChunkedFile() :
    verbosity_(0), 
    blockSize100k_(9),
    workFactor_(30),
    file_(NULL), 
    bzfile_(NULL), 
    unused_(NULL),
    nUnused_(0),
    offset_(0),
    compressed_in_(0)
{
}

ChunkedFile::~ChunkedFile() {
    close();
}

bool ChunkedFile::openReadWrite(const string& filename) {
    writing_ = true;
    return open(filename, "r+b");
}

bool ChunkedFile::openWrite(const string& filename) {
    writing_ = true;
    return open(filename, "wb");
}

bool ChunkedFile::openRead(const string& filename) {
    writing_ = false;
    return open(filename, "rb");
}

bool ChunkedFile::open(const string& filename, const string& mode) {
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

    read_compression_  = compression::None;
    write_compression_ = compression::None;
    filename_          = filename;
    offset_            = ftell(file_);

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

    if (type == write_compression_)
        return true;

    try {
        switch (type) {
            case compression::None: setWriteModeUncompressed(); break;
            case compression::BZ2:  setWriteModeBZ2();          break;
            case compression::GZ:   setWriteModeGZ();           break;
        }
    }
    catch (const Exception& ex) {
        ROS_ERROR("Error setting write mode: %s", ex.what());
        return false;
    }

    write_compression_ = type;

    return true;
}

bool ChunkedFile::setReadMode(CompressionType type) {
    if (!file_) {
        ROS_ERROR("Can't set compression mode before opening a file");
        return false;
    }

    if (type == read_compression_)
        return true;

    try {
        switch (type) {
            case compression::None: setReadModeUncompressed(); break;
            case compression::BZ2:  setReadModeBZ2();          break;
            case compression::GZ:   setReadModeGZ();           break;
        }
    }
    catch (const Exception& ex) {
        ROS_ERROR("Error setting read mode: %s", ex.what());
        return false;
    }

    read_compression_ = type;

    return true;
}

void ChunkedFile::setWriteModeUncompressed() {
    unsigned int nbytes_in;
    unsigned int nbytes_out;
    BZ2_bzWriteClose(&bzerror_, bzfile_, 0, &nbytes_in, &nbytes_out);

    switch (bzerror_) {
        case BZ_IO_ERROR: throw Exception("BZ_IO_ERROR");
    }

    offset_ += nbytes_out;
    compressed_in_ = 0;
}

void ChunkedFile::setReadModeUncompressed() {
    BZ2_bzReadClose(&bzerror_, bzfile_);

    switch (bzerror_ == BZ_IO_ERROR) {
        case BZ_IO_ERROR: throw Exception("BZ_IO_ERROR");
    }
}

void ChunkedFile::setWriteModeBZ2() {
    bzfile_ = BZ2_bzWriteOpen(&bzerror_, file_, blockSize100k_, verbosity_, workFactor_);

    switch (bzerror_) {
        case BZ_OK: break;
        default: {
            BZ2_bzWriteClose(&bzerror_, bzfile_, 0, NULL, NULL);
            throw Exception("Error opening file for writing compressed stream");
        }
    }

    compressed_in_ = 0;
}

void ChunkedFile::setReadModeBZ2() {
    bzfile_ = BZ2_bzReadOpen(&bzerror_, file_, verbosity_, 0, unused_, nUnused_);

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

void ChunkedFile::setWriteModeGZ() {
    //! \todo
}

void ChunkedFile::setReadModeGZ() {
    //! \todo
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

bool ChunkedFile::truncate(uint64_t length) {
    int fd = fileno(file_);
    return ftruncate(fd, length) == 0;
}

size_t ChunkedFile::write(const string& s) { return write((void*) s.c_str(), s.size()); }

size_t ChunkedFile::write(void* ptr, size_t size) {
    switch (write_compression_) {
        case compression::None: {
            size_t result = fwrite(ptr, 1, size, file_);
            if (result != size)
                return false;

            offset_ += size;
            break;
        }
        case compression::BZ2: {
            BZ2_bzWrite(&bzerror_, bzfile_, ptr, size);

            switch (bzerror_) {
            case BZ_IO_ERROR: ROS_ERROR("BZ_IO_ERROR: error writing the compressed file"); return false;
            }

            compressed_in_ += size;
            break;
        }
        case compression::GZ: {
            //! \todo
            break;
        }
    }

    return true;
}

//! \todo add error handling
string ChunkedFile::getline() {
	char buffer[1024];
	fgets(buffer, 1024, file_);
	string s(buffer);
	offset_ += s.size();

    return s;
}

size_t ChunkedFile::read(void* ptr, size_t size) {
    switch (read_compression_) {
        case compression::None: {
            if (nUnused_ > 0) {
                // We have unused data from the last compressed read
                if ((size_t) nUnused_ == size) {
                    // Copy the unused data into the buffer
                    memcpy(ptr, unused_, nUnused_);

                    clearUnused();
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

                    clearUnused();
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
        case compression::BZ2: {
            // Reading a compressed stream

            BZ2_bzRead(&bzerror_, bzfile_, ptr, size);

            offset_ += size;

            switch (bzerror_) {
            case BZ_OK:               return size;
            case BZ_STREAM_END:
                if (unused_ || nUnused_ > 0)
                    ROS_ERROR("unused data already available");
                else
                    BZ2_bzReadGetUnused(&bzerror_, bzfile_, (void**) &unused_, &nUnused_);
                return size;
            case BZ_IO_ERROR:         ROS_ERROR("BZ_IO_ERROR: error reading from compressed stream");                                break;
            case BZ_UNEXPECTED_EOF:   ROS_ERROR("BZ_UNEXPECTED_EOF: compressed stream ended before logical end-of-stream detected"); break;
            case BZ_DATA_ERROR:       ROS_ERROR("BZ_DATA_ERROR: data integrity error detected in compressed stream");                break;
            case BZ_DATA_ERROR_MAGIC: ROS_ERROR("BZ_DATA_ERROR_MAGIC: stream does not begin with requisite header bytes");           break;
            case BZ_MEM_ERROR:        ROS_ERROR("BZ_MEM_ERROR: insufficient memory available");                                      break;
            }

            return 0;
        }
        case compression::GZ: {
            //! \todo
            return 0;
        }
    }
}

void ChunkedFile::decompress(uint8_t* dest, unsigned int destLen, uint8_t* source, unsigned int sourceLen) {
    int result = BZ2_bzBuffToBuffDecompress((char*) dest, &destLen, (char*) source, sourceLen, 0, verbosity_);

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

void ChunkedFile::checkError() const {
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

void ChunkedFile::clearUnused() {
    unused_ = NULL;
    nUnused_ = 0;
}

}
