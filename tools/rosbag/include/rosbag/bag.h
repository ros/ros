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

#include <boost/format.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/thread/mutex.hpp>

namespace rosbag {

namespace bagmode
{
    //! The possible modes to open a bag in
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

    void open(std::string const& filename, BagMode mode = bagmode::Default);          //!< Open a bag file
    void close();     //!< Close the bag file (write to disk, append index, etc.)

    std::string     getFileName()     const;                      //!< Get the filename of the bag
    BagMode         getMode()         const;                      //!< Get the mode the bag is in
    int             getVersion()      const;                      //!< Get the version of the open bagfile
    int             getMajorVersion() const;                      //!< Get the major-version of the open bagfile
    int             getMinorVersion() const;                      //!< Get the minor-version of the open bagfile
    uint64_t        getOffset()       const;                      //!< Get the offset into the actual file

    void            setCompression(CompressionType compression);  //!< Set the compression method to use for writing chunks
    CompressionType getCompression() const;                       //!< Get the compression method to use for writing chunks
    void            setChunkThreshold(uint32_t chunk_threshold);  //!< Set the threshold for creating new chunks
    uint32_t        getChunkThreshold() const;                    //!< Get the threshold for creating new chunks

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
     * \param connection_header  A connection header.
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(std::string const& topic, ros::Time const& time, MessageInstance const& msg,
			   boost::shared_ptr<ros::M_string> connection_header = boost::shared_ptr<ros::M_string>());

    void dump();

private:
    // This helper function actually does the write with an arbitrary serializable message
    template<class T>
    void doWrite(std::string const& topic, ros::Time const& time, T const& msg, boost::shared_ptr<ros::M_string> connection_header);

    void openRead  (std::string const& filename);
    void openWrite (std::string const& filename);
    void openAppend(std::string const& filename);

    void closeWrite();

    template<class T>
    boost::shared_ptr<T const> instantiateBuffer(IndexEntry const& index_entry);  //!< deserializes the message held in record_buffer_

    void startWriting();
    void stopWriting();

    void startReadingVersion102();
    void startReadingVersion200();

    // Writing
    
    void writeVersion();
    void writeFileHeaderRecord();
    void writeConnectionRecord(ConnectionInfo const* connection_info);
    void appendConnectionRecordToBuffer(Buffer& buf, ConnectionInfo const* connection_info);
    template<class T>
    void writeMessageDataRecord(uint32_t conn_id, ros::Time const& time, T const& msg);
    void writeIndexRecords();
    void writeConnectionRecords();
    void writeChunkInfoRecords();
    void startWritingChunk(ros::Time time);
    void writeChunkHeader(CompressionType compression, uint32_t compressed_size, uint32_t uncompressed_size);
    void stopWritingChunk();

    // Reading

    void readVersion();
    void readFileHeaderRecord();
    void readConnectionRecord();
    void readChunkHeader(ChunkHeader& chunk_header);
    void readChunkInfoRecord();
    void readConnectionIndexRecord200();

    void readTopicIndexRecord102();
    void readMessageDefinitionRecord102();

    ros::Header readMessageDataHeader(IndexEntry const& index_entry);
    uint32_t    readMessageDataSize(IndexEntry const& index_entry);

    // Would be nice not to have to template this on Stream.  Also,
    // we don't need to read the header here either.  It just so
    // happens to be the most efficient way to skip it at the moment.
    template<typename Stream>
    void readMessageDataIntoStream(IndexEntry const& index_entry, Stream& stream);

    void     decompressChunk(uint64_t chunk_pos);
    void     decompressRawChunk(ChunkHeader const& chunk_header);
    void     decompressBz2Chunk(ChunkHeader const& chunk_header);
    uint32_t getChunkOffset() const;

    // Record header I/O

    void writeHeader(ros::M_string const& fields);
    void writeDataLength(uint32_t data_len);
    void appendHeaderToBuffer(Buffer& buf, ros::M_string const& fields);
    void appendDataLengthToBuffer(Buffer& buf, uint32_t data_len);
    void readHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read);
    void readMessageDataHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read);
    bool readHeader(ros::Header& header);
    bool readDataLength(uint32_t& data_size);
    bool isOp(ros::M_string& fields, uint8_t reqOp);

    void loadMessageDataRecord102(uint64_t offset);

    // Header fields

    template<typename T>
    std::string toHeaderString(T const* field);

    std::string toHeaderString(ros::Time const* field);

    template<typename T>
    void readField(ros::M_string const& fields, std::string const& field_name, bool required, T* data);

    void readField(ros::M_string const& fields, std::string const& field_name, unsigned int min_len, unsigned int max_len, bool required, std::string& data);
    void readField(ros::M_string const& fields, std::string const& field_name, bool required, std::string& data);

    void readField(ros::M_string const& fields, std::string const& field_name, bool required, ros::Time& data);

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
    uint32_t        bag_revision_;

    uint64_t file_header_pos_;
    uint64_t index_data_pos_;
    uint32_t connection_count_;
    uint32_t chunk_count_;

    boost::mutex record_mutex_;
    
    // Current chunk
    bool      chunk_open_;
    ChunkInfo curr_chunk_info_;
    uint64_t  curr_chunk_data_pos_;
    std::map<uint32_t,    std::multiset<IndexEntry> > curr_chunk_connection_indexes_;
    std::map<std::string, std::multiset<IndexEntry> > curr_chunk_topic_indexes_;

    uint32_t                                          top_connection_id_;
    std::map<std::string, uint32_t>                   topic_connection_ids_;
    std::map<ros::M_string*, uint32_t>                header_connection_ids_;
    std::map<uint32_t, ConnectionInfo*>               connections_;

    std::vector<ChunkInfo>                            chunks_;

    std::map<uint32_t, std::multiset<IndexEntry> >    connection_indexes_;
    std::map<std::string, std::multiset<IndexEntry> > topic_indexes_;

    Buffer   header_buffer_;           //!< reusable buffer in which to assemble the record header before writing to file
    Buffer   record_buffer_;           //!< reusable buffer in which to assemble the record data before writing to file

    Buffer   chunk_buffer_;            //!< reusable buffer to read chunk into
    Buffer   decompress_buffer_;       //!< reusable buffer to decompress chunks into

    Buffer   outgoing_chunk_buffer_;   //!< reusable buffer to read chunk into

    Buffer*  current_buffer_;

    uint64_t decompressed_chunk_;      //!< position of decompressed chunk
};

} // namespace rosbag

#include "rosbag/message_instance.h"

namespace rosbag {

// Templated method definitions

template<typename T>
std::string Bag::toHeaderString(T const* field) {
    return std::string((char*) field, sizeof(T));
}

template<typename T>
void Bag::readField(ros::M_string const& fields, std::string const& field_name, bool required, T* data) {
    ros::M_string::const_iterator i = checkField(fields, field_name, sizeof(T), sizeof(T), required);
    memcpy(data, i->second.data(), sizeof(T));
}

template<typename Stream>
void Bag::readMessageDataIntoStream(IndexEntry const& index_entry, Stream& stream) {
    ros::Header header;
    uint32_t data_size;
    uint32_t bytes_read;
    
    switch (version_)
    {
    case 200:
        decompressChunk(index_entry.chunk_pos);
        readMessageDataHeaderFromBuffer(*current_buffer_, index_entry.offset, header, data_size, bytes_read);
        if (data_size > 0)
            memcpy(stream.advance(data_size), current_buffer_->getData() + index_entry.offset + bytes_read, data_size);
        break;
    case 102:
        // todo
        break;
    default:
        throw BagFormatException((boost::format("Unhandled version: %1%") % version_).str());
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
        readMessageDataHeaderFromBuffer(*current_buffer_, index_entry.offset, header, data_size, bytes_read);
        {  
            ros::serialization::IStream s(current_buffer_->getData() + index_entry.offset + bytes_read, data_size);
            ros::serialization::deserialize(s, *p);
        }
        return p;
    //case 102:
        // todo
    default:
        throw BagFormatException((boost::format("Unhandled version: %1%") % version_).str());
    }
}

template<class T>
void Bag::doWrite(std::string const& topic, ros::Time const& time, T const& msg, boost::shared_ptr<ros::M_string> connection_header) {
    // Whenever we write we increment our revision
    bag_revision_++;

    // Get ID for connection header
    ConnectionInfo* connection_info = NULL;
    uint32_t conn_id = 0;
    ros::M_string* header_address = connection_header.get();
    if (header_address == NULL) {
        // No connection header: we'll manufacture one, and store by topic

        std::map<std::string, uint32_t>::iterator topic_connection_ids_iter = topic_connection_ids_.find(topic);
        if (topic_connection_ids_iter == topic_connection_ids_.end()) {
            conn_id = top_connection_id_++;
            topic_connection_ids_[topic] = conn_id;
        }
        else {
            conn_id = topic_connection_ids_iter->second;
            connection_info = connections_[conn_id];
        }
    }
    else {
        // Store the connection info by the address of the connection header

        std::map<ros::M_string*, uint32_t>::iterator header_connection_ids_iter = header_connection_ids_.find(header_address);
        if (header_connection_ids_iter == header_connection_ids_.end()) {
            conn_id = top_connection_id_++;
            header_connection_ids_[header_address] = conn_id;
        }
        else {
            conn_id = header_connection_ids_iter->second;
            connection_info = connections_[conn_id];
        }
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

        // Write connection info record, if necessary
        if (connection_info == NULL) {
            connection_info = new ConnectionInfo();
            connection_info->id       = conn_id;
            connection_info->topic    = topic;
            connection_info->datatype = std::string(ros::message_traits::datatype(msg));
            connection_info->md5sum   = std::string(ros::message_traits::md5sum(msg));
            connection_info->msg_def  = std::string(ros::message_traits::definition(msg));

            ROS_INFO("NEW CONNECTION");
            ROS_INFO("  datatype: %s", ros::message_traits::datatype(msg));
            ROS_INFO("  md5sum: %s", ros::message_traits::md5sum(msg));
            ROS_INFO("  msg_def: %s", ros::message_traits::definition(msg));

            if (connection_header != NULL) {
                for (ros::M_string::const_iterator i = connection_header->begin(); i != connection_header->end(); i++)
                    connection_info->header[i->first] = i->second;
            }
            else {
                connection_info->header["message_definition"] = connection_info->msg_def;
                connection_info->header["datatype"]           = connection_info->datatype;
                connection_info->header["md5sum"]             = connection_info->md5sum;
            }
            connections_[conn_id] = connection_info;

            writeConnectionRecord(connection_info);
            appendConnectionRecordToBuffer(outgoing_chunk_buffer_, connection_info);
        }

        // Add to topic indexes
        IndexEntry index_entry;
        index_entry.time      = time;
        index_entry.chunk_pos = curr_chunk_info_.pos;
        index_entry.offset    = getChunkOffset();

        std::multiset<IndexEntry>& chunk_connection_index = curr_chunk_connection_indexes_[connection_info->id];
        chunk_connection_index.insert(chunk_connection_index.end(), index_entry);
        std::multiset<IndexEntry>& chunk_topic_index = curr_chunk_topic_indexes_[topic];
        chunk_topic_index.insert(chunk_topic_index.end(), index_entry);

        std::multiset<IndexEntry>& connection_index = connection_indexes_[connection_info->id];
        connection_index.insert(connection_index.end(), index_entry);
        std::multiset<IndexEntry>& index = topic_indexes_[topic];
        index.insert(index.end(), index_entry);

        // Increment the topic count
        curr_chunk_info_.topic_counts[topic]++;

        // Write the message data
        writeMessageDataRecord(conn_id, time, msg);

        // Check if we want to stop this chunk
        uint32_t chunk_size = getChunkOffset();
        ROS_INFO("  curr_chunk_size=%d (threshold=%d)", chunk_size, chunk_threshold_);
        if (chunk_size > chunk_threshold_) {
            // Empty the outgoing chunk
            stopWritingChunk();
            outgoing_chunk_buffer_.setSize(0);

            // We no longer have a valid curr_chunk_info
            curr_chunk_info_.pos = -1;
        }
    }
}

template<class T>
void Bag::writeMessageDataRecord(uint32_t conn_id, ros::Time const& time, T const& msg) {
    ros::M_string header;
    header[OP_FIELD_NAME]         = toHeaderString(&OP_MSG_DATA);
    header[CONNECTION_FIELD_NAME] = toHeaderString(&conn_id);
    header[TIME_FIELD_NAME]       = toHeaderString(&time);

    // Assemble message in memory first, because we need to write its length
    uint32_t msg_ser_len = ros::serialization::serializationLength(msg);

    record_buffer_.setSize(msg_ser_len);
    
    ros::serialization::OStream s(record_buffer_.getData(), msg_ser_len);

    // TODO: with a little work here we can serialize directly into the file -- sweet
    ros::serialization::serialize(s, msg);

    ROS_INFO("Writing MSG_DATA [%llu:%d]: conn=%d sec=%d nsec=%d data_len=%d",
              (unsigned long long) file_.getOffset(), getChunkOffset(), conn_id, time.sec, time.nsec, msg_ser_len);

    // TODO: If we are clever here, we can serialize into the
    // outgoing_chunk_buffer and get rid of the record_buffer_ all
    // together.

    writeHeader(header);
    writeDataLength(msg_ser_len);
    write((char*) record_buffer_.getData(), msg_ser_len);
    
    // TODO: Using appendHeaderToBuffer is ugly.  We need a better abstraction
    appendHeaderToBuffer(outgoing_chunk_buffer_, header);
    appendDataLengthToBuffer(outgoing_chunk_buffer_, msg_ser_len);

    uint32_t offset = outgoing_chunk_buffer_.getSize();
    outgoing_chunk_buffer_.setSize(outgoing_chunk_buffer_.getSize() + msg_ser_len);
    memcpy(outgoing_chunk_buffer_.getData() + offset, record_buffer_.getData(), msg_ser_len);
    
    // Update the current chunk time
    if (time > curr_chunk_info_.end_time)
    	curr_chunk_info_.end_time = time;
}

}

#endif
