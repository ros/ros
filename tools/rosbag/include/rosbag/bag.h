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
#include "rosbag/structures.h"

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

class MessageInstance;
class View;
class Query;

class Bag
{
    friend class MessageInstance;
    friend class View;

public:
    Bag();
    ~Bag();

    //! Open a bag file
    bool open(std::string const& filename, BagMode mode = bagmode::Default);

    //! Fix a bag file
    bool rewrite(std::string const& src_filename, std::string const& dest_filename);

    std::string getFileName()     const;
    BagMode     getMode()         const;
    int         getVersion()      const;
    int         getMajorVersion() const;
    int         getMinorVersion() const;

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

    template<class T>
    void write_(std::string const& topic, ros::Time const& time, T const& msg, boost::shared_ptr<ros::M_string> connection_header);
    
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
     * \param msg   A MessageInstance
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(std::string const& topic, ros::Time const& time, MessageInstance const& msg,
			   boost::shared_ptr<ros::M_string> connection_header = boost::shared_ptr<ros::M_string>());

    void dump();

private:
    bool openRead  (std::string const& filename);
    bool openWrite (std::string const& filename);
    bool openAppend(std::string const& filename);

    void closeWrite();

    template<class T>
    boost::shared_ptr<T const> instantiateBuffer(IndexEntry const& index_entry);  //!< deserializes the message held in record_buffer_

    void startWritingVersion200();
    void stopWritingVersion200();

    bool startReadingVersion102();
    bool startReadingVersion200();

    // Writing
    
    void writeVersion();
    void writeFileHeaderRecord();
    void writeMessageDefinitionRecord(TopicInfo const* topic_info);
    template<class T>
    void writeMessageDataRecord(std::string const& topic, ros::Time const& time, bool latching, std::string const& callerid, T const& msg);
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

    ros::Header readMessageDataHeader(IndexEntry const& index_entry);
    uint32_t    readMessageDataSize(IndexEntry const& index_entry);
   
    // Would be nice not to have to template this on Stream.  Also,
    // we don't need to read the header here either.  It just so
    // happens to be the most efficient way to skip it at the moment.
    template<typename Stream>
    void readMessageDataIntoStream(IndexEntry const& index_entry, Stream& stream);

    bool readChunkHeader(ChunkHeader& chunk_header);
    bool readTopicIndexRecord();
    bool readTopicIndexDataVersion0(uint32_t count, std::string const& topic);
    bool readTopicIndexDataVersion1(uint32_t count, std::string const& topic, uint64_t chunk_pos);
    bool readChunkInfoRecord();

    bool     decompressChunk(uint64_t chunk_pos);
    bool     decompressRawChunk(ChunkHeader const& chunk_header);
    bool     decompressBz2Chunk(ChunkHeader const& chunk_header);
    uint32_t getChunkOffset() const;

    // Record header I/O

    void writeHeader(ros::M_string const& fields, uint32_t data_len);
    bool readHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read);
    bool readMessageDataHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read);
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

    Buffer   header_buffer_;        //!< reusable buffer in which to assemble the record header before writing to file
    Buffer   record_buffer_;        //!< reusable buffer in which to assemble the record data before writing to file
    Buffer   chunk_buffer_;         //!< reusable buffer to read chunk into
    Buffer   decompress_buffer_;    //!< reusable buffer to decompress chunks into
    uint64_t decompressed_chunk_;   //!< position of decompressed chunk
    Buffer   instantiate_buffer_;   //!< reusable buffer in which to instantiate MessageInstance messages into
};

}

#include "rosbag/message_instance.h"

namespace rosbag {

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

template<typename Stream>
void Bag::readMessageDataIntoStream(IndexEntry const& index_entry, Stream& stream)
{
    ros::Header header;
    uint32_t data_size;
    uint32_t bytes_read;
    
    switch (version_)
    {
    case 200:
        decompressChunk(index_entry.chunk_pos);
        readMessageDataHeaderFromBuffer(decompress_buffer_, index_entry.offset, header, data_size, bytes_read);
         
        if (data_size > 0)
            memcpy(stream.advance(data_size), decompress_buffer_.getData() + index_entry.offset + bytes_read, data_size);
            
        return;
    default:
        ROS_FATAL("Unhandled version: %d", version_);
        break;
    }
}

template<class T>
boost::shared_ptr<T const> Bag::instantiateBuffer(IndexEntry const& index_entry) {
    boost::shared_ptr<T> p = boost::shared_ptr<T>(new T());

    ros::Header header;
    uint32_t data_size;
    uint32_t bytes_read;

    switch (version_)
    {
    case 200:
        decompressChunk(index_entry.chunk_pos);
        readMessageDataHeaderFromBuffer(decompress_buffer_, index_entry.offset, header, data_size, bytes_read);
        {  
            ros::serialization::IStream s(decompress_buffer_.getData() + index_entry.offset + bytes_read, data_size);
            ros::serialization::deserialize(s, *p);
        }
        break;
    default:
        ROS_FATAL("Unhandled version: %d", version_);
        break;
    }

    return p;
}

template<class T>
void Bag::write_(std::string const& topic, ros::Time const& time, T const& msg, boost::shared_ptr<ros::M_string> connection_header) {
    bool needs_def_written = false;
    TopicInfo* topic_info;
    {
        boost::mutex::scoped_lock lock(topic_infos_mutex_);
        
        std::map<std::string, TopicInfo*>::iterator key = topic_infos_.find(topic);
        if (key == topic_infos_.end()) {
            // Extract the topic info from the message
            topic_info = new TopicInfo();
            topic_info->topic    = topic;
            topic_info->msg_def  = ros::message_traits::definition(msg);
            topic_info->datatype = ros::message_traits::datatype(msg);
            topic_info->md5sum   = ros::message_traits::md5sum(msg);
            topic_infos_[topic] = topic_info;

            // Initialize the topic index
            topic_indexes_[topic] = std::vector<IndexEntry>();
            
            // Flag that we need to write a message definition
            needs_def_written = true;
        }
        else
            topic_info = key->second;
    }

    // Get information about possible latching and callerid from the connection header
    bool latching = false;
    std::string callerid("");
    if (connection_header != NULL) {
        ros::M_string::iterator latch_iter = connection_header->find(std::string("latching"));
        if (latch_iter != connection_header->end() && latch_iter->second != std::string("0"))
            latching = true;
        
        ros::M_string::iterator callerid_iter = connection_header->find(std::string("callerid"));
        if (callerid_iter != connection_header->end())
            callerid = callerid_iter->second;
    }

    {
    	//! \todo enabling this lock seems to cause a deadlock - do we even need it?
        //boost::mutex::scoped_lock lock(record_mutex_);

        // Seek to the end of the file (needed in case previous operation was a read)
        //! \todo only do when necessary
        seek(0, std::ios::end);

        // Write the chunk header if we're starting a new chunk
        if (!chunk_open_)
            startWritingChunk(time);

        // Write a message definition record, if necessary
        if (needs_def_written)
            writeMessageDefinitionRecord(topic_info);

        // Add to topic index
        IndexEntry index_entry;
        index_entry.time      = time;
        index_entry.chunk_pos = curr_chunk_info_.pos;
        index_entry.offset    = getChunkOffset();
        curr_chunk_topic_indexes_[topic].push_back(index_entry);

        // Increment the topic count
        curr_chunk_info_.topic_counts[topic]++;

        // Write the message data
        writeMessageDataRecord(topic, time, latching, callerid, msg);

        // Check if we want to stop this chunk
        uint32_t chunk_size = getChunkOffset();
        ROS_DEBUG("  curr_chunk_size=%d (threshold=%d)", chunk_size, chunk_threshold_);
        if (chunk_size > chunk_threshold_)
            stopWritingChunk();
    }
}

template<class T>
void Bag::writeMessageDataRecord(std::string const& topic, ros::Time const& time, bool latching, std::string const& callerid, T const& msg) {
    ros::M_string header;
    header[OP_FIELD_NAME]    = toHeaderString(&OP_MSG_DATA);
    header[TOPIC_FIELD_NAME] = topic;
    header[TIME_FIELD_NAME]  = toHeaderString(&time);
    if (latching) {
        header[LATCHING_FIELD_NAME] = std::string("1");
        header[CALLERID_FIELD_NAME] = callerid;
    }

    // Assemble message in memory first, because we need to write its length
    uint32_t msg_ser_len = ros::serialization::serializationLength(msg);

    record_buffer_.setSize(msg_ser_len);
    
    ros::serialization::OStream s(record_buffer_.getData(), msg_ser_len);

    // TODO: with a little work here we can serialize directly into the file -- sweet
    ros::serialization::serialize(s, msg);

    ROS_DEBUG("Writing MSG_DATA [%llu:%d]: topic=%s sec=%d nsec=%d data_len=%d",
              (unsigned long long) file_.getOffset(), getChunkOffset(), topic.c_str(), time.sec, time.nsec, msg_ser_len);

    writeHeader(header, msg_ser_len);
    write((char*) record_buffer_.getData(), msg_ser_len);

    // Update the current chunk time
    if (time > curr_chunk_info_.end_time)
    	curr_chunk_info_.end_time = time;
}

}

#endif
