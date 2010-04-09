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

#include <ros/header.h>
#include <ros/time.h>
#include <ros/message.h>
#include <ros/message_traits.h>
#include <ros/ros.h>

#include <ios>
#include <map>
#include <set>
#include <stdexcept>

#include <boost/thread/mutex.hpp>

namespace rosbag
{

// BagMode
namespace bagmode
{
    enum BagMode
    {
        Write      = 0,
        Read       = 1,
        Append     = 2,
        ReadAppend = 3,
        Default    = Read
    };
}
typedef bagmode::BagMode BagMode;

/**
 * Contains the compression type as a string, e.g. "none" or "bz2" (see constants.h),
 * and the compressed and uncompressed size of the chunk in bytes.
 */
struct ChunkHeader
{
    std::string compression;
    uint32_t    compressed_size;
    uint32_t    uncompressed_size;
};

/**
 * Contains the earliest and latest timestamps in the chunk, the absolute offset of
 * the chunk record in the file, and the number of messages in each topic stored in
 * the chunk.
 */
struct ChunkInfo
{
    ros::Time start_time;
    ros::Time end_time;
    uint64_t  pos;

    std::map<std::string, uint32_t> topic_counts;
};

//! Topic information
struct TopicInfo
{
    std::string topic;
    std::string datatype;
    std::string md5sum;
    std::string msg_def;
};

//! Entry in a topic index
struct IndexEntry
{
    ros::Time time;
    uint64_t  chunk_pos;
    uint32_t  offset;
};

class MessageInstance;
class MessageInstanceCompare;

typedef std::multiset<MessageInstance, MessageInstanceCompare> MessageList;

class Bag
{
    friend class MessageInstance;

public:
    Bag();
    ~Bag();

    bool open(const std::string& filename, BagMode mode = bagmode::Default);           //!< Open a bag file

    bool rewrite(const std::string& src_filename, const std::string& dest_filename);   //!< Fix a bag file

    // Version 1.3 options
    void            setChunkThreshold(uint32_t chunk_threshold);
    uint32_t        getChunkThreshold() const;
    void            setCompression(CompressionType compression);  //!< Set the compression method to use for writing chunks
    CompressionType getCompression() const;                       //!< Get the compression method to use for writing chunks

    BagMode  getMode()   const;  //!< Get the mode
    uint64_t getOffset() const;

    int getVersion()      const; //!< Get the version number
    int getMajorVersion() const; //!< Get the major version number
    int getMinorVersion() const; //!< Get the minor version number

    //! Close bag file
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
    void write(const std::string& topic, ros::Time time, ros::Message::ConstPtr msg);

    //! Write a message into the bag file
    /*!
     * \param topic The topic name
     * \param time  Timestamp of the message
     * \param msg   A const reference to a message
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(const std::string& topic, ros::Time time, const ros::Message& msg);

    //! Write a message into the bag file
    /*!
     * \param topic The topic name
     * \param time  Timestamp of the message
     * \param msg   A MessageInstance as returned by reader
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(const std::string& topic, ros::Time time, MessageInstance* msg);

    //! Return a MessageList
    MessageList getMessageList(const ros::Time& start_time = ros::TIME_MIN,
                               const ros::Time& end_time = ros::TIME_MAX);

    //! Return a MessageList using a subset of topics
    MessageList getMessageListByTopic(const std::vector<std::string>& topics,
                                      const ros::Time& start_time = ros::TIME_MIN,
                                      const ros::Time& end_time = ros::TIME_MAX);

    //! Return a MessageList using a subset of types
    MessageList getMessageListByType(const std::vector<std::string>& types,
                                     const ros::Time& start_time = ros::TIME_MIN,
                                     const ros::Time& end_time = ros::TIME_MAX);

    void dump();

private:
    bool openRead  (const std::string& filename);
    bool openWrite (const std::string& filename);
    bool openAppend(const std::string& filename);

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
    void writeMessageDefinitionRecord(const TopicInfo& topic_info);
    void writeMessageDataRecord(const std::string& topic, const ros::Time& time, bool latching, const std::string& callerid, const ros::Message& msg);
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
    bool readMessageDataRecord102(const std::string& topic, uint64_t offset);
    bool readMessageDataRecord103(const std::string& topic, uint64_t chunk_pos, uint32_t offset);
    bool readChunkHeader(ChunkHeader& chunk_header);
    bool readTopicIndexRecord();
    bool readTopicIndexDataVersion0(uint32_t data_size, uint32_t count, const std::string& topic);
    bool readTopicIndexDataVersion1(uint32_t data_size, uint32_t count, const std::string& topic);
    bool readChunkInfoRecord();

    bool     decompressChunk(uint64_t chunk_pos);
    uint32_t getChunkOffset() const;

    // Record header I/O

    void writeHeader(const ros::M_string& fields, uint32_t data_len);
    bool readHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read);
    bool readHeader(ros::Header& header, uint32_t& data_size);
    bool isOp(ros::M_string& fields, uint8_t reqOp);

    // Header fields

    template<typename T>
    std::string toHeaderString(const T* field);

    std::string toHeaderString(const ros::Time* field);

    template<typename T>
    bool readField(const ros::M_string& fields, const std::string& field_name, bool required, T* data);

    bool readField(const ros::M_string& fields, const std::string& field_name, unsigned int min_len, unsigned int max_len, bool required, std::string& data);
    bool readField(const ros::M_string& fields, const std::string& field_name, bool required, std::string& data);

    bool readField(const ros::M_string& fields, const std::string& field_name, bool required, ros::Time& data);

    ros::M_string::const_iterator checkField(const ros::M_string& fields, const std::string& field,
                                             unsigned int min_len, unsigned int max_len, bool required) const;

    // Low-level I/O

    void write(const char* s, std::streamsize n);
    void write(const std::string& s);
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

    std::map<std::string, TopicInfo>                topic_infos_;    //!< topic infos
    std::vector<ChunkInfo>                          chunk_infos_;    //!< chunk infos
    std::map<std::string, std::vector<IndexEntry> > topic_indexes_;  //!< topic indexes

    //

    Buffer   header_buffer_;      //!< reusable buffer in which to assemble the record header before writing to file
    Buffer   record_buffer_;      //!< reusable buffer in which to assemble the record data before writing to file
    Buffer   chunk_buffer_;       //!< reusable buffer to read chunk into
    Buffer   decompress_buffer_;  //!< reusable buffer to decompress chunks into
    uint64_t decompressed_chunk_;
    
    bool          writing_enabled_;

    boost::mutex  check_disk_mutex_;
    ros::WallTime check_disk_next_;
    ros::WallTime warn_next_;
};

// Templated method definitions

template<typename T>
std::string Bag::toHeaderString(const T* field) {
	return std::string((char*) field, sizeof(T));
}

template<typename T>
bool Bag::readField(const ros::M_string& fields, const std::string& field_name, bool required, T* data) {
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
