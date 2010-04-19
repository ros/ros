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
*********************************************************************/

#ifndef ROSBAG_BAG_H
#define ROSBAG_BAG_H

#include "rosbag/buffer.h"
#include "rosbag/chunked_file.h"
#include "rosbag/constants.h"
#include "rosbag/exceptions.h"

#include "ros/header.h"
#include "ros/time.h"
#include "ros/message.h"
#include "ros/message_traits.h"
#include "ros/ros.h"

#include <ios>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>

#include <boost/iterator/iterator_facade.hpp>
#include <boost/thread/mutex.hpp>

namespace rosbag {

namespace bagmode
{
    enum BagMode
    {
        Write   = 0,
        Read    = 1,
        Append  = 2,
        Default = Read
    };
}
typedef bagmode::BagMode BagMode;

struct TopicInfo
{
    std::string topic;
    std::string datatype;
    std::string md5sum;
    std::string msg_def;
};

struct ChunkInfo
{
    ros::Time   start_time;    //! earliest timestamp of a message in the chunk
    ros::Time   end_time;      //! latest timestamp of a message in the chunk
    uint64_t    pos;           //! absolute byte offset of chunk record in bag file

    std::map<std::string, uint32_t> topic_counts;   //! number of messages in each topic stored in the chunk
};

struct ChunkHeader
{
    std::string compression;          //! chunk compression type, e.g. "none" or "bz2" (see constants.h)
    uint32_t    compressed_size;      //! compressed size of the chunk in bytes
    uint32_t    uncompressed_size;    //! uncompressed size of the chunk in bytes
};

struct IndexEntry
{
    ros::Time time;            //! timestamp of the message
    uint64_t  chunk_pos;       //! absolute byte offset of the chunk record containing the message
    uint32_t  offset;          //! relative byte offset of the message record (either definition or data) in the chunk
};

struct IndexEntryCompare
{
    bool operator()(ros::Time const& a, IndexEntry const& b) const { return a < b.time; }
    bool operator()(IndexEntry const& a, ros::Time const& b) const { return a.time < b; }
};

class MessageInfo;
class MessageInstance;
class View;
class Query;

class Bag
{
    friend class MessageInfo;
	friend class MessageInstance;
	friend class View;

public:
    Bag();
    ~Bag();

    //! Open a bag file
    bool open(std::string const& filename, BagMode mode = bagmode::Default);

    //! Fix a bag file
    bool rewrite(std::string const& src_filename, std::string const& dest_filename);

    BagMode  getMode()         const;
    int      getVersion()      const;
    int      getMajorVersion() const;
    int      getMinorVersion() const;

    uint64_t getOffset()       const;   //! \todo replace with getSize?

    // Version 1.3 options
    void            setCompression(CompressionType compression);  //!< Set the compression method to use for writing chunks
    CompressionType getCompression() const;                       //!< Get the compression method to use for writing chunks
    void            setChunkThreshold(uint32_t chunk_threshold);
    uint32_t        getChunkThreshold() const;

    //! Close the bag file
    /*!
     * Ensure file is written out to disk, index is appended and file descriptor is closed.
     */
    void close();
    
    //! Write a message into the bag file
    /*!
     * \param topic The topic name
     * \param time  Timestamp of the message
     * \param msg   A pointer to the message to be added
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(std::string const& topic, ros::Time const& time, ros::Message::ConstPtr msg);

    //! Write a message into the bag file
    /*!
     * \param topic The topic name
     * \param time  Timestamp of the message
     * \param msg   A const reference to a message
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(std::string const& topic, ros::Time const& time, ros::Message const& msg);

    //! Write a message into the bag file
    /*!
     * \param topic The topic name
     * \param time  Timestamp of the message
     * \param msg   A MessageInstance as returned by reader
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(std::string const& topic, ros::Time const& time, MessageInstance* msg);

    void dump();

private:
    bool openRead  (std::string const& filename);
    bool openWrite (std::string const& filename);
    bool openAppend(std::string const& filename);

    void closeWrite();

    template<class T>
    boost::shared_ptr<T const> instantiateBuffer();  //!< deserializes the message held in record_buffer_

    void startWritingVersion103();
    void stopWritingVersion103();

    bool startReadingVersion101();
    bool startReadingVersion102();
    bool startReadingVersion103();

    bool checkLogging();
    bool scheduledCheckDisk();
    bool checkDisk();

    // Writing
    
    void writeVersion();
    void writeFileHeaderRecord();
    void writeMessageDefinitionRecord(TopicInfo const* topic_info);
    void writeMessageDataRecord(std::string const& topic, ros::Time const& time, bool latching, std::string const& callerid, ros::Message const& msg);
    void writeTopicIndexRecords();
    void writeMessageDefinitionRecords();
    void writeChunkInfoRecords();
    void startWritingChunk(ros::Time time);
    void writeChunkHeader(CompressionType compression, uint32_t compressed_size, uint32_t uncompressed_size);
    void stopWritingChunk();

    // Reading

    bool readVersion();
    bool readFileHeaderRecord();
    bool readMessageDefinitionRecord();
    bool readMessageDataRecord102(std::string const& topic, uint64_t offset);
    bool readMessageDataRecord103(std::string const& topic, uint64_t chunk_pos, uint32_t offset);
    bool readChunkHeader(ChunkHeader& chunk_header);
    bool readTopicIndexRecord();
    bool readTopicIndexDataVersion0(uint32_t data_size, uint32_t count, std::string const& topic);
    bool readTopicIndexDataVersion1(uint32_t data_size, uint32_t count, std::string const& topic);
    bool readChunkInfoRecord();

    bool     decompressChunk(uint64_t chunk_pos);
    uint32_t getChunkOffset() const;

    // Record header I/O

    void writeHeader(ros::M_string const& fields, uint32_t data_len);
    bool readHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read);
    bool readHeader(ros::Header& header, uint32_t& data_size);
    bool isOp(ros::M_string& fields, uint8_t reqOp);

    // Header fields

    template<typename T>
    std::string toHeaderString(T const* field);

    std::string toHeaderString(ros::Time const* field);

    template<typename T>
    bool readField(ros::M_string const& fields, std::string const& field_name, bool required, T* data);

    bool readField(ros::M_string const& fields, std::string const& field_name, unsigned int min_len, unsigned int max_len, bool required, std::string& data);
    bool readField(ros::M_string const& fields, std::string const& field_name, bool required, std::string& data);

    bool readField(ros::M_string const& fields, std::string const& field_name, bool required, ros::Time& data);

    ros::M_string::const_iterator checkField(ros::M_string const& fields, std::string const& field,
                                             unsigned int min_len, unsigned int max_len, bool required) const;

    // Low-level I/O

    void write(char const* s, std::streamsize n);
    void write(std::string const& s);
    void read(char* b, std::streamsize n);
    void seek(uint64_t pos, int origin = std::ios_base::beg);

private:
    BagMode         mode_;
    ChunkedFile     file_;
    int             version_;
    CompressionType compression_;
    uint32_t        chunk_threshold_;

    uint64_t file_header_pos_;
    uint64_t index_data_pos_;
    uint32_t topic_count_;
    uint32_t chunk_count_;

    boost::mutex record_mutex_;
    boost::mutex topic_infos_mutex_;
    
    // Current chunk
    bool      chunk_open_;
    ChunkInfo curr_chunk_info_;
    uint64_t  curr_chunk_data_pos_;
    std::map<std::string, std::vector<IndexEntry> > curr_chunk_topic_indexes_;

    std::map<std::string, TopicInfo*>               topic_infos_;
    std::vector<ChunkInfo>                          chunk_infos_;
    std::map<std::string, std::vector<IndexEntry> > topic_indexes_;

    //

    Buffer   header_buffer_;        //!< reusable buffer in which to assemble the record header before writing to file
    Buffer   record_buffer_;        //!< reusable buffer in which to assemble the record data before writing to file
    Buffer   chunk_buffer_;         //!< reusable buffer to read chunk into
    Buffer   decompress_buffer_;    //!< reusable buffer to decompress chunks into
    uint64_t decompressed_chunk_;   //!< position of decompressed chunk
    Buffer   instantiate_buffer_;   //!< reusable buffer in which to instantiate MessageInstance messages into
    
    bool          writing_enabled_;

    boost::mutex  check_disk_mutex_;
    ros::WallTime check_disk_next_;
    ros::WallTime warn_next_;
};

// Templated method definitions

template<typename T>
std::string Bag::toHeaderString(T const* field) {
	return std::string((char*) field, sizeof(T));
}

template<typename T>
bool Bag::readField(ros::M_string const& fields, std::string const& field_name, bool required, T* data) {
    ros::M_string::const_iterator i;
    if ((i = checkField(fields, field_name, sizeof(T), sizeof(T), required)) == fields.end())
        return false;
    memcpy(data, i->second.data(), sizeof(T));
    return true;
}

template<class T>
boost::shared_ptr<T const> Bag::instantiateBuffer() {
    boost::shared_ptr<T> p = boost::shared_ptr<T>(new T());
    p->__serialized_length = record_buffer_.getSize();
    p->deserialize(record_buffer_.getData());

    return p;
}

}

#endif
