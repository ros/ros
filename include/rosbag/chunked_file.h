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

#ifndef ROSBAG_CHUNKED_FILE_H
#define ROSBAG_CHUNKED_FILE_H

#include <ios>
#include <stdint.h>
#include <string>

#include <bzlib.h>
#include <zlib.h>

namespace rosbag
{

namespace compression
{
    enum CompressionType
    {
        None = 0,
        BZ2  = 1,
        ZLIB = 2
    };
}
typedef compression::CompressionType CompressionType;

//! ChunkedFile reads and writes files which contain interleaved chunks of compressed and uncompressed data.
/*!
 * ChunkedFile uses libbzip2 (http://www.bzip.org) for reading/writing compressed data.
 */
class ChunkedFile
{
public:
    ChunkedFile();
    ~ChunkedFile();

    bool openWrite    (const std::string& filename);            //!< open file for writing
    bool openRead     (const std::string& filename);            //!< open file for reading
    bool openReadWrite(const std::string& filename);            //!< open file for reading & writing

    bool close();                                               //!< close the file

    std::string getFileName()          const;                   //!< return path of currently open file
    uint64_t    getOffset()            const;                   //!< return current offset from the beginning of the file
    uint32_t    getCompressedBytesIn() const;                   //!< return the number of bytes written to current compressed stream
    bool        isOpen()               const;                   //!< return true if file is open for reading or writing
    bool        good()                 const;                   //!< return true if hasn't reached end-of-file and no error

    bool        setReadMode(CompressionType type);
    bool        setWriteMode(CompressionType type);

    // File I/O
    size_t      write(const std::string& s);
    size_t      write(void* ptr, size_t size);                          //!< write size bytes from ptr to the file
    size_t      read(void* ptr, size_t size);                           //!< read size bytes from the file into ptr
    std::string getline();
    bool        truncate(uint64_t length);
    bool        seek(uint64_t offset, int origin = std::ios_base::beg); //!< seek to given offset from origin

    void decompress(CompressionType compression, uint8_t* dest, unsigned int destLen, uint8_t* source, unsigned int sourceLen);

private:
    bool open(const std::string& filename, const std::string& mode);

    void clearUnusedBZ2();
    void checkErrorBZ2() const;
    void startWriteBZ2();
    void stopWriteBZ2();
    void startReadBZ2();
    void stopReadBZ2();

    void startWriteZLIB();
    void stopWriteZLIB();
    void startReadZLIB();
    void stopReadZLIB();

private:
    std::string     filename_;           //!< path to file
    FILE*           file_;               //!< file pointer

    bool            writing_;            //!< true iff file is opened for writing
    CompressionType read_compression_;   //!< current compression mode for reading data
    CompressionType write_compression_;  //!< current compression mode for writing data

    uint64_t        offset_;             //!< current position in the file
    uint64_t        compressed_in_;      //!< number of bytes written to current compressed stream

    // BZ2 parameters
    int             verbosity_;          //!< level of debugging output (0-4; 0 default). 0 is silent, 4 is max verbose debugging output
    int             blockSize100k_;      //!< compression block size (1-9; 9 default). 9 is best compression, most memory
    int             workFactor_;         //!< compression behavior for worst case, highly repetitive data (0-250; 30 default)

    // BZ2 state
    BZFILE*         bzfile_;             //!< bzlib compressed file stream
    int             bzerror_;            //!< last error from bzlib
    char*           unused_;             //!< extra data read by bzlib
    int             nUnused_;            //!< number of bytes of extra data read by bzlib
};

}

#endif
