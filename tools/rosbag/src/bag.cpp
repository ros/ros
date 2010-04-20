// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rosbag/bag.h"
#include "rosbag/message_instance.h"
#include "rosbag/query.h"
#include "rosbag/view.h"

#include <inttypes.h>
#include <signal.h>
#include <sys/statvfs.h>

#include <iomanip>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using std::map;
using std::priority_queue;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::M_string;
using ros::Time;

namespace rosbag {

Bag::Bag() :
    compression_(compression::BZ2),
    chunk_threshold_(768 * 1024),  // 768KB chunks
    file_header_pos_(0),
    index_data_pos_(0),
    topic_count_(0),
    chunk_count_(0),
    chunk_open_(false),
    curr_chunk_data_pos_(0),
    decompressed_chunk_(0)
{
}

Bag::~Bag() {
    close();

    for (map<string, TopicInfo*>::iterator i = topic_infos_.begin(); i != topic_infos_.end(); i++)
    	delete i->second;
}

bool Bag::open(string const& filename, BagMode mode) {
    mode_ = mode;

    switch (mode_) {
    case bagmode::Read:   return openRead(filename);
    case bagmode::Write:  return openWrite(filename);
    case bagmode::Append: return openAppend(filename);
    default:
    	ROS_ERROR("Unknown mode: %d", (int) mode);
    }

    return false;
}

bool Bag::openRead(string const& filename) {
    if (!file_.openRead(filename)) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return false;
    }

    if (!readVersion())
        return false;

    switch (version_) {
    case 102: return startReadingVersion102();
    case 200: return startReadingVersion200();
    default:
        ROS_ERROR("Unsupported bag file version: %d.%d", getMajorVersion(), getMinorVersion());
        return false;
    }

    return true;
}

bool Bag::openWrite(string const& filename) {
    if (!file_.openWrite(filename)) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return false;
    }

    startWritingVersion200();
    return true;
}

bool Bag::openAppend(string const& filename) {
    if (!file_.openReadWrite(filename)) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return false;
    }

    // Read in the version and file header
    if (!readVersion())
        return false;
    startReadingVersion200();

    // Truncate the file to chop off the index
    file_.truncate(index_data_pos_);
    index_data_pos_ = 0;

    // Rewrite the file header, clearing the index position (so we know if the index is invalid)
    seek(file_header_pos_);
    writeFileHeaderRecord();

    // Seek to the end of the file
    seek(0, std::ios::end);

    return true;
}

void Bag::write(std::string const& topic, ros::Time const& time, ros::Message::ConstPtr msg) {
	write_(topic, time, *msg, msg->__connection_header);
}

void Bag::write(std::string const& topic, ros::Time const& time, ros::Message const& msg) {
	write_(topic, time, msg, msg.__connection_header);
}

void Bag::write(std::string const& topic, ros::Time const& time, MessageInstance const& msg, boost::shared_ptr<ros::M_string> connection_header) {
	write_(topic, time, msg, connection_header);
}

bool Bag::rewrite(string const& src_filename, string const& dest_filename) {
    Bag in;
    if (!in.open(src_filename, bagmode::Read))
        return false;

    string target_filename = dest_filename;
    if (target_filename == src_filename)
        target_filename += ".active";

    if (!open(target_filename, bagmode::Write))
        return false;

    View view;
    view.addQuery(in, Query());

    foreach(MessageInstance const m, view)
        write(m.getTopic(), m.getTime(), m);

    in.close();
    close();

    if (target_filename != dest_filename)
        rename(target_filename.c_str(), dest_filename.c_str());

    return true;
}

void Bag::close() {
    if (!file_.isOpen())
        return;

    if (mode_ == bagmode::Write || mode_ == bagmode::Append)
    	closeWrite();
    
    // Unfortunately closing this possibly enormous file takes a while
    // (especially over NFS) and handling of a SIGINT while a file is
    // closing leads to a double free.  So, we disable signals while
    // we close the file.
    
    // Darwin doesn't have sighandler_t; I hope that sig_t on Linux does
    // the right thing.
    //sighandler_t old = signal(SIGINT, SIG_IGN);
    
    sig_t old = signal(SIGINT, SIG_IGN);
    file_.close();
    signal(SIGINT, old);
}

void Bag::closeWrite() {
    stopWritingVersion200();
}

string   Bag::getFileName() const { return file_.getFileName(); }
BagMode  Bag::getMode()     const { return mode_;               }
uint64_t Bag::getOffset()   const { return file_.getOffset();   }

void     Bag::setChunkThreshold(uint32_t chunk_threshold) { chunk_threshold_ = chunk_threshold; }
uint32_t Bag::getChunkThreshold() const                   { return chunk_threshold_;            }

void            Bag::setCompression(CompressionType compression) { compression_ = compression; }
CompressionType Bag::getCompression() const                      { return compression_;        }

// Version

void Bag::writeVersion() {
    string version = string("#ROSBAG V") + VERSION + string("\n");

    ROS_DEBUG("Writing VERSION [%llu]: %s", (unsigned long long) file_.getOffset(), version.c_str());

    write(version);
}

bool Bag::readVersion() {
    // Read the version line
    string version_line = file_.getline();

    file_header_pos_ = file_.getOffset();

    char logtypename[100];
    int version_major, version_minor;
    if (sscanf(version_line.c_str(), "#ROS%s V%d.%d", logtypename, &version_major, &version_minor) != 3) {
    	ROS_ERROR("Error reading version line");
    	return false;
    }

    // Special case
    if (version_major == 0 && version_line[0] == '#')
        version_major = 1;

    version_ = version_major * 100 + version_minor;

    ROS_DEBUG("Read VERSION: version=%d", version_);

    return true;
}

int Bag::getVersion()      const { return version_;       }
int Bag::getMajorVersion() const { return version_ / 100; }
int Bag::getMinorVersion() const { return version_ % 100; }

//

void Bag::startWritingVersion200() {
    writeVersion();
    file_header_pos_ = file_.getOffset();
    writeFileHeaderRecord();
}

void Bag::stopWritingVersion200() {
    if (chunk_open_)
        stopWritingChunk();

    index_data_pos_ = file_.getOffset();
    writeMessageDefinitionRecords();
    writeChunkInfoRecords();

    seek(file_header_pos_);
    writeFileHeaderRecord();

    topic_infos_.clear();
}

bool Bag::startReadingVersion200() {
    ROS_DEBUG("Reading in version 2.0 bag");

    // Read the file header record, which points to the end of the chunks
    if (!readFileHeaderRecord())
        return false;

    // Seek to the end of the chunks
    seek(index_data_pos_);

    // Read the message definition records (one for each topic)
    for (uint32_t i = 0; i < topic_count_; i++) {
        if (!readMessageDefinitionRecord()) {
            ROS_ERROR("Failed to read message definition record");
            return false;
        }
    }

    // Read the chunk info records
    for (uint32_t i = 0; i < chunk_count_; i++) {
        if (!readChunkInfoRecord()) {
            ROS_ERROR("Failed to read chunk info record");
            return false;
        }
    }

    // Read the topic indexes
    foreach(ChunkInfo const& chunk_info, chunk_infos_) {
        curr_chunk_info_ = chunk_info;

        seek(curr_chunk_info_.pos);

        // Skip over the chunk data
        ChunkHeader chunk_header;
        if (!readChunkHeader(chunk_header))
        	return false;
        seek(chunk_header.compressed_size, std::ios::cur);

        // Read the topic index records after the chunk
        for (unsigned int i = 0; i < chunk_info.topic_counts.size(); i++)
            if (!readTopicIndexRecord())
                return false;
    }

    return true;
}

bool Bag::startReadingVersion102() {
    // @todo: handle 1.2 bags with no index

    ROS_DEBUG("Reading in version 1.2 bag");

    // Read the file header record, which points to the start of the topic indexes
    if (!readFileHeaderRecord())
        return false;

    // Seek to the beginning of the topic index records
    seek(index_data_pos_);

    // Read the topic index records, which point to the offsets of each message in the file
    while (file_.good())
        readTopicIndexRecord();

    // Read the message definition records (which are the first entry in the topic indexes)
    for (map<string, vector<IndexEntry> >::const_iterator i = topic_indexes_.begin(); i != topic_indexes_.end(); i++) {
        vector<IndexEntry> const& topic_index = i->second;
        IndexEntry const&         first_entry = *topic_index.begin();

        ROS_DEBUG("Reading message definition for %s at %llu", i->first.c_str(), (unsigned long long) first_entry.chunk_pos);

        seek(first_entry.chunk_pos);

        if (!readMessageDefinitionRecord())
            return false;
    }

    return true;
}

// File header record

void Bag::writeFileHeaderRecord() {
    boost::mutex::scoped_lock lock(record_mutex_);

    topic_count_ = topic_infos_.size();
    chunk_count_ = chunk_infos_.size();

    ROS_DEBUG("Writing FILE_HEADER [%llu]: index_pos=%llu topic_count=%d chunk_count=%d",
              (unsigned long long) file_.getOffset(), (unsigned long long) index_data_pos_, topic_count_, chunk_count_);
    
    // Write file header record
    M_string header;
    header[OP_FIELD_NAME]          = toHeaderString(&OP_FILE_HEADER);
    header[INDEX_POS_FIELD_NAME]   = toHeaderString(&index_data_pos_);
    header[TOPIC_COUNT_FIELD_NAME] = toHeaderString(&topic_count_);
    header[CHUNK_COUNT_FIELD_NAME] = toHeaderString(&chunk_count_);

    boost::shared_array<uint8_t> header_buffer;
    uint32_t header_len;
    ros::Header::write(header, header_buffer, header_len);
    uint32_t data_len = 0;
    if (header_len < FILE_HEADER_LENGTH)
        data_len = FILE_HEADER_LENGTH - header_len;
    write((char*) &header_len, 4);
    write((char*) header_buffer.get(), header_len);
    write((char*) &data_len, 4);
    
    // Pad the file header record out
    if (data_len > 0) {
        string padding;
        padding.resize(data_len, ' ');
        write(padding);
    }
}

bool Bag::readFileHeaderRecord() {
    ros::Header header;
    uint32_t data_size;    
    if (!readHeader(header, data_size)) {
        ROS_ERROR("Error reading FILE_HEADER record");
        return false;
    }

    M_string& fields = *header.getValues();

    if (!isOp(fields, OP_FILE_HEADER)) {
        ROS_ERROR("Expected FILE_HEADER op not found");
        return false;
    }

    // Read index position
    if (!readField(fields, INDEX_POS_FIELD_NAME, true, (uint64_t*) &index_data_pos_))
        return false;

    // Read topic and chunks count
    if (version_ >= 200) {
        readField(fields, TOPIC_COUNT_FIELD_NAME, true, &topic_count_);
        readField(fields, CHUNK_COUNT_FIELD_NAME, true, &chunk_count_);
    }

    ROS_DEBUG("Read FILE_HEADER: index_pos=%llu topic_count=%d chunk_count=%d",
              (unsigned long long) index_data_pos_, topic_count_, chunk_count_);

    // Skip the data section (just padding)
    seek(data_size, std::ios::cur);

    return true;
}

uint32_t Bag::getChunkOffset() const {
    if (compression_ == compression::None)
        return file_.getOffset() - curr_chunk_data_pos_;
    else
        return file_.getCompressedBytesIn();
}

void Bag::startWritingChunk(Time time) {
    // Initialize chunk info
    curr_chunk_info_.pos        = file_.getOffset();
    curr_chunk_info_.start_time = time;
    curr_chunk_info_.end_time   = time;

    // Write the chunk header, with a place-holder for the data sizes (we'll fill in when the chunk is finished)
    writeChunkHeader(compression_, 0, 0);

    // Turn on compressed writing
    file_.setWriteMode(compression_);
    
    // Record where the data section of this chunk started
    curr_chunk_data_pos_ = file_.getOffset();

    chunk_open_ = true;
}

void Bag::stopWritingChunk() {
    // Add this chunk to the index
    chunk_infos_.push_back(curr_chunk_info_);
    for (map<string, vector<IndexEntry> >::const_iterator i = curr_chunk_topic_indexes_.begin(); i != curr_chunk_topic_indexes_.end(); i++) {
        foreach(IndexEntry const& e, i->second) {
            ROS_DEBUG("adding to topic index: %s -> %llu:%d", i->first.c_str(), (unsigned long long) e.chunk_pos, e.offset);
            topic_indexes_[i->first].push_back(e);
        }
    }

    // Get the uncompressed and compressed sizes
    uint32_t uncompressed_size = getChunkOffset();
    file_.setWriteMode(compression::None);
    uint32_t compressed_size = file_.getOffset() - curr_chunk_data_pos_;

    ROS_DEBUG("<<< END CHUNK: uncompressed = %d compressed = %d", uncompressed_size, compressed_size);

    // Rewrite the chunk header with the size of the chunk (remembering current offset)
    uint64_t end_of_chunk_pos = file_.getOffset();
    seek(curr_chunk_info_.pos);
    writeChunkHeader(compression_, compressed_size, uncompressed_size);

    // Write out the topic indexes and clear them
    seek(end_of_chunk_pos);
    writeTopicIndexRecords();
    curr_chunk_topic_indexes_.clear();
    
    // Flag that we're starting a new chunk
    chunk_open_ = false;
}

void Bag::writeChunkHeader(CompressionType compression, uint32_t compressed_size, uint32_t uncompressed_size) {
    ChunkHeader chunk_header;
    switch (compression) {
    case compression::None: chunk_header.compression = COMPRESSION_NONE; break;
    case compression::BZ2:  chunk_header.compression = COMPRESSION_BZ2;  break;
    //case compression::ZLIB: chunk_header.compression = COMPRESSION_ZLIB; break;
    }
    chunk_header.compressed_size   = compressed_size;
    chunk_header.uncompressed_size = uncompressed_size;

    ROS_DEBUG("Writing CHUNK [%llu]: compression=%s compressed=%d uncompressed=%d",
              (unsigned long long) file_.getOffset(), chunk_header.compression.c_str(), chunk_header.compressed_size, chunk_header.uncompressed_size);

    M_string header;
    header[OP_FIELD_NAME]          = toHeaderString(&OP_CHUNK);
    header[COMPRESSION_FIELD_NAME] = chunk_header.compression;
    header[SIZE_FIELD_NAME]        = toHeaderString(&chunk_header.uncompressed_size);

    writeHeader(header, chunk_header.compressed_size);
}

bool Bag::readChunkHeader(ChunkHeader& chunk_header) {
    ros::Header header;
    if (!readHeader(header, chunk_header.compressed_size))
        return false;
        
    M_string& fields = *header.getValues();
    
    if (!isOp(fields, OP_CHUNK) ||
    	!readField(fields, COMPRESSION_FIELD_NAME, true, chunk_header.compression) ||
    	!readField(fields, SIZE_FIELD_NAME,        true, &chunk_header.uncompressed_size))
        return false;

    ROS_DEBUG("Read CHUNK: compression=%s size=%d uncompressed=%d (%f)", chunk_header.compression.c_str(), chunk_header.compressed_size, chunk_header.uncompressed_size, 100 * ((double) chunk_header.compressed_size) / chunk_header.uncompressed_size);

    return true;
}

// Topic index records

void Bag::writeTopicIndexRecords() {
    boost::mutex::scoped_lock lock(record_mutex_);

    for (map<string, vector<IndexEntry> >::const_iterator i = curr_chunk_topic_indexes_.begin(); i != curr_chunk_topic_indexes_.end(); i++) {
        string const&             topic       = i->first;
        vector<IndexEntry> const& topic_index = i->second;

        // Write the index record header
        M_string header;
        header[OP_FIELD_NAME]    = toHeaderString(&OP_INDEX_DATA);
        header[TOPIC_FIELD_NAME] = topic;
        header[VER_FIELD_NAME]   = toHeaderString(&INDEX_VERSION);
        uint32_t topic_index_size = topic_index.size();
        header[COUNT_FIELD_NAME] = toHeaderString(&topic_index_size);
       
        uint32_t data_len = topic_index_size * sizeof(IndexEntry);
        writeHeader(header, data_len);

        ROS_DEBUG("Writing INDEX_DATA: topic=%s ver=%d count=%d", topic.c_str(), INDEX_VERSION, topic_index_size);
        
        // Write the index record data (pairs of timestamp and position in file)
        foreach(IndexEntry const& e, topic_index) {
            write((char*) &e.time.sec,  4);
            write((char*) &e.time.nsec, 4);
            write((char*) &e.offset,    4);

            ROS_DEBUG("  - %d.%d: %d", e.time.sec, e.time.nsec, e.offset);
        }
    }
}

bool Bag::readTopicIndexRecord() {
    ros::Header header;
    uint32_t data_size;
    if (!readHeader(header, data_size)) {
    	ROS_ERROR("Error reading INDEX_DATA header");
    	return false;
    }
    M_string& fields = *header.getValues();
    
    if (!isOp(fields, OP_INDEX_DATA)) {
    	ROS_ERROR("Expected INDEX_DATA record");
        return false;
    }
    
    uint32_t index_version;
    string topic;
    uint32_t count;
    if (!readField(fields, VER_FIELD_NAME,   true, &index_version) ||
    	!readField(fields, TOPIC_FIELD_NAME, true, topic)          ||
    	!readField(fields, COUNT_FIELD_NAME, true, &count))
    	return false;

    ROS_DEBUG("Read INDEX_DATA: ver=%d topic=%s count=%d", index_version, topic.c_str(), count);

    switch (index_version) {
    case 0:  return readTopicIndexDataVersion0(count, topic);
    case 1:  return readTopicIndexDataVersion1(count, topic, curr_chunk_info_.pos);
    default: {
    	ROS_ERROR("Unsupported INDEX_DATA version: %d", index_version);
    	return false;
    }
    }
}

//! Store the position of the message in the chunk_pos field
bool Bag::readTopicIndexDataVersion0(uint32_t count, string const& topic) {
    // Read the index entry records
    vector<IndexEntry>& topic_index = topic_indexes_[topic];
    for (uint32_t i = 0; i < count; i++) {
        IndexEntry index_entry;
        uint32_t sec;
        uint32_t nsec;
        read((char*) &sec,                   4);
        read((char*) &nsec,                  4);
        read((char*) &index_entry.chunk_pos, 8);
        index_entry.time = Time(sec, nsec);
        index_entry.offset = 0;

        ROS_DEBUG("  - %d.%d: %llu", sec, nsec, (unsigned long long) index_entry.chunk_pos);

        topic_index.push_back(index_entry);
    }

    return true;
}

bool Bag::readTopicIndexDataVersion1(uint32_t count, string const& topic, uint64_t chunk_pos) {
    // Read the index entry records
    vector<IndexEntry>& topic_index = topic_indexes_[topic];
    for (uint32_t i = 0; i < count; i++) {
        IndexEntry index_entry;
        index_entry.chunk_pos = chunk_pos;
        uint32_t sec;
        uint32_t nsec;
        read((char*) &sec,                4);
        read((char*) &nsec,               4);
        read((char*) &index_entry.offset, 4);
        index_entry.time = Time(sec, nsec);

        ROS_DEBUG("  - %d.%d: %llu+%d", sec, nsec, (unsigned long long) index_entry.chunk_pos, index_entry.offset);

        topic_index.push_back(index_entry);
    }

    return true;
}

// Message definition records

void Bag::writeMessageDefinitionRecords() {
    boost::mutex::scoped_lock lock(record_mutex_);

    for (map<string, TopicInfo*>::const_iterator i = topic_infos_.begin(); i != topic_infos_.end(); i++) {
        TopicInfo const* topic_info = i->second;
        writeMessageDefinitionRecord(topic_info);
    }
}

void Bag::writeMessageDefinitionRecord(TopicInfo const* topic_info) {
    ROS_DEBUG("Writing MSG_DEF [%llu:%d]: topic=%s md5sum=%s type=%s def=...",
              (unsigned long long) file_.getOffset(), getChunkOffset(), topic_info->topic.c_str(), topic_info->md5sum.c_str(), topic_info->datatype.c_str());

    M_string header;
    header[OP_FIELD_NAME]    = toHeaderString(&OP_MSG_DEF);
    header[TOPIC_FIELD_NAME] = topic_info->topic;
    header[MD5_FIELD_NAME]   = topic_info->md5sum;
    header[TYPE_FIELD_NAME]  = topic_info->datatype;
    header[DEF_FIELD_NAME]   = topic_info->msg_def;

    writeHeader(header, 0);
}

bool Bag::readMessageDefinitionRecord() {
    ros::Header header;
    uint32_t data_size;    
    if (!readHeader(header, data_size)) {
        ROS_ERROR("Error reading message definition header");
        return false;
    }
    M_string& fields = *header.getValues();

    if (!isOp(fields, OP_MSG_DEF)) {
        ROS_ERROR("Expected MSG_DEF op not found");
        return false;
    }

    string topic, md5sum, datatype, message_definition;
    if (!readField(fields, TOPIC_FIELD_NAME,               true, topic)      ||
        !readField(fields, MD5_FIELD_NAME,   32,       32, true, md5sum)     ||
        !readField(fields, TYPE_FIELD_NAME,                true, datatype)   ||
        !readField(fields, DEF_FIELD_NAME,    0, UINT_MAX, true, message_definition))
    	return false;

    map<string, TopicInfo*>::iterator key = topic_infos_.find(topic);
    if (key == topic_infos_.end()) {
        TopicInfo* topic_info = new TopicInfo();
        topic_info->topic    = topic;
        topic_info->msg_def  = message_definition;
        topic_info->datatype = datatype;
        topic_info->md5sum   = md5sum;
        topic_infos_[topic] = topic_info;

        ROS_DEBUG("Read MSG_DEF: topic=%s md5sum=%s datatype=%s def=...", topic.c_str(), md5sum.c_str(), datatype.c_str());
    }

    return true;
}

bool Bag::decompressChunk(uint64_t chunk_pos)
{
    if (decompressed_chunk_ == chunk_pos)
        return true;

    // Seek to the start of the chunk
    seek(chunk_pos);

    // Read the chunk header
    ChunkHeader chunk_header;
    if (!readChunkHeader(chunk_header))
        return false;

    bool ret = false;

    // Read and decompress the chunk.  These assume we are at the right place in the stream already
    if (chunk_header.compression == COMPRESSION_NONE)
    {
        ret = decompressRawChunk(chunk_header);
    } else if (chunk_header.compression == COMPRESSION_BZ2)
    {
        ret = decompressBz2Chunk(chunk_header);
    } else {
        ROS_ERROR("Unknown compression: %s", chunk_header.compression.c_str());
    }
    
    if (ret)
        decompressed_chunk_ = chunk_pos;

    return ret;
}


// Reading this into a buffer isn't completely necessary, but we do it anyways for now
bool Bag::decompressRawChunk(ChunkHeader const& chunk_header)
{
    assert(chunk_header.compression == COMPRESSION_NONE);
    assert(chunk_header.compressed_size == chunk_header.uncompressed_size);

    ROS_DEBUG("compressed_size: %d uncompressed_size: %d", chunk_header.compressed_size, chunk_header.uncompressed_size);

    decompress_buffer_.setSize(chunk_header.compressed_size);
    file_.read((char*) decompress_buffer_.getData(), chunk_header.compressed_size);

    return true;
}

bool Bag::decompressBz2Chunk(ChunkHeader const& chunk_header)
{
    assert(chunk_header.compression == COMPRESSION_BZ2);

    CompressionType compression = compression::BZ2;

    ROS_DEBUG("compressed_size: %d uncompressed_size: %d", chunk_header.compressed_size, chunk_header.uncompressed_size);

    chunk_buffer_.setSize(chunk_header.compressed_size);
    file_.read((char*) chunk_buffer_.getData(), chunk_header.compressed_size);

    decompress_buffer_.setSize(chunk_header.uncompressed_size);
    file_.decompress(compression, decompress_buffer_.getData(), decompress_buffer_.getSize(), chunk_buffer_.getData(), chunk_buffer_.getSize());

    return true;
}

ros::Header Bag::readMessageDataHeader(IndexEntry const& index_entry)
{
    ros::Header header;
    uint32_t data_size;
    uint32_t bytes_read;
    switch (version_)
    {
    case 200:
        decompressChunk(index_entry.chunk_pos);
        readMessageDataHeaderFromBuffer(decompress_buffer_, index_entry.offset, header, data_size, bytes_read);
        return header;
    default:
        ROS_FATAL("Unhandled version: %d", version_);
        break;
    }

    return ros::Header();
}


// This actually loads the header, which shouldn't be necessary.  Not
// the end of the world for now.
uint32_t  Bag::readMessageDataSize(IndexEntry const& index_entry)
{
    ros::Header header;
    uint32_t data_size;
    uint32_t bytes_read;

    switch (version_)
    {
    case 200:
        decompressChunk(index_entry.chunk_pos);
        readMessageDataHeaderFromBuffer(decompress_buffer_, index_entry.offset, header, data_size, bytes_read);

        return data_size;
    default:
        ROS_FATAL("Unhandled version: %d", version_);
        break;
    }

    return 0;
}

// Will fill this in later
/*
  bool Bag::loadMessageDataRecord102(string const& topic, uint64_t offset) {
  ROS_DEBUG("loadMessageDataRecord: offset=%llu", (unsigned long long) offset);

  seek(offset);

  ros::Header header;
  uint32_t data_size;
  uint8_t op;
  do {
  if (!readHeader(header, data_size) ||
  !readField(*header.getValues(), OP_FIELD_NAME, true, &op))
  return false;
  }
  while (op == OP_MSG_DEF);
  if (op != OP_MSG_DATA)
  return false;

  string msg_topic;
  if (!readField(*header.getValues(), TOPIC_FIELD_NAME, true, msg_topic))
  return false;
  if (topic != msg_topic)
  return false;

  record_buffer_.setSize(data_size);
  file_.read((char*) record_buffer_.getData(), data_size);

  return true;
  }
*/

void Bag::writeChunkInfoRecords() {
    boost::mutex::scoped_lock lock(record_mutex_);

    foreach(ChunkInfo const& chunk_info, chunk_infos_) {
        // Write the chunk info header
        M_string header;
        uint32_t chunk_topic_count = chunk_info.topic_counts.size();
        header[OP_FIELD_NAME]         = toHeaderString(&OP_CHUNK_INFO);
        header[VER_FIELD_NAME]        = toHeaderString(&CHUNK_INFO_VERSION);
        header[CHUNK_POS_FIELD_NAME]  = toHeaderString(&chunk_info.pos);
        header[START_TIME_FIELD_NAME] = toHeaderString(&chunk_info.start_time);
        header[END_TIME_FIELD_NAME]   = toHeaderString(&chunk_info.end_time);
        header[COUNT_FIELD_NAME]      = toHeaderString(&chunk_topic_count);

        // Measure length of data
        uint32_t data_len = 0;
        for (map<string, uint32_t>::const_iterator i = chunk_info.topic_counts.begin(); i != chunk_info.topic_counts.end(); i++) {
            string const& topic = i->first;
            data_len += 4 + topic.size() + 4;   // 4 bytes for length of topic_name + topic_name + 4 bytes for topic count
        }
        
        ROS_DEBUG("Writing CHUNK_INFO [%llu]: ver=%d pos=%llu start=%d.%d end=%d.%d data_len=%d",
                  (unsigned long long) file_.getOffset(), CHUNK_INFO_VERSION, (unsigned long long) chunk_info.pos,
                  chunk_info.start_time.sec, chunk_info.start_time.nsec,
                  chunk_info.end_time.sec, chunk_info.end_time.nsec,
                  data_len);

        writeHeader(header, data_len);

        // Write the topic names and counts
        for (map<string, uint32_t>::const_iterator i = chunk_info.topic_counts.begin(); i != chunk_info.topic_counts.end(); i++) {
            string const& topic = i->first;
            uint32_t      count = i->second;

            uint32_t topic_name_size = topic.size();

            write((char*) &topic_name_size, 4);
            write(topic);
            write((char*) &count, 4);

            ROS_DEBUG("  - %s: %d", topic.c_str(), count);
        }
    }
}

bool Bag::readChunkInfoRecord() {
    // Read a CHUNK_INFO header
    ros::Header header;
    uint32_t data_size;
    if (!readHeader(header, data_size))
        return false;
    M_string& fields = *header.getValues();
    if (!isOp(fields, OP_CHUNK_INFO))
        return false;

    // Check that the chunk info version is current
    uint32_t chunk_info_version;
    if (!readField(fields, VER_FIELD_NAME, true, &chunk_info_version))
        return false;
    assert(chunk_info_version == CHUNK_INFO_VERSION);

    // Read the chunk position, timestamp, and topic count fields
    ChunkInfo chunk_info;
    uint32_t chunk_topic_count;
    if (!readField(fields, CHUNK_POS_FIELD_NAME,  true, &chunk_info.pos)        ||
        !readField(fields, START_TIME_FIELD_NAME, true,  chunk_info.start_time) ||
        !readField(fields, END_TIME_FIELD_NAME,   true,  chunk_info.end_time)   ||
        !readField(fields, COUNT_FIELD_NAME,      true, &chunk_topic_count))
        return false;
    ROS_DEBUG("Read CHUNK_INFO: chunk_pos=%llu topic_count=%d start=%d.%d end=%d.%d",
              (unsigned long long) chunk_info.pos, chunk_topic_count,
              chunk_info.start_time.sec, chunk_info.start_time.nsec,
              chunk_info.end_time.sec, chunk_info.end_time.nsec);

    // Read the topic count entries
    char topic_name_buf[4096];
    for (uint32_t i = 0; i < chunk_topic_count; i ++) {
        uint32_t topic_name_len;
        uint32_t topic_count;
        read((char*) &topic_name_len, 4);
        read((char*) &topic_name_buf, topic_name_len);
        read((char*) &topic_count,    4);

        string topic = string(topic_name_buf, topic_name_len);

        ROS_DEBUG("  %s: %d messages", topic.c_str(), topic_count);

        chunk_info.topic_counts[topic] = topic_count;
    }

    chunk_infos_.push_back(chunk_info);

    return true;
}

// Record I/O

bool Bag::isOp(M_string& fields, uint8_t reqOp) {
    uint8_t op;
    return readField(fields, OP_FIELD_NAME, true, &op) && (op == reqOp);
}

void Bag::writeHeader(M_string const& fields, uint32_t data_len) {
    boost::shared_array<uint8_t> header_buffer;
    uint32_t header_len;
    ros::Header::write(fields, header_buffer, header_len);

    write((char*) &header_len, 4);
    write((char*) header_buffer.get(), header_len);

    write((char*) &data_len, 4);
}

//! \todo clean this up
bool Bag::readHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read) {
    ROS_ASSERT(buffer.getSize() > 8);

    uint8_t* start = (uint8_t*) buffer.getData() + offset;

    uint8_t* ptr = start;

    // Read the header length
    uint32_t header_len;
    memcpy(&header_len, ptr, 4);
    ptr += 4;

    // Parse the header
    string error_msg;
    bool parsed = header.parse(ptr, header_len, error_msg);
    if (!parsed)
        return false;
    ptr += header_len;

    // Read the data size
    memcpy(&data_size, ptr, 4);
    ptr += 4;

    bytes_read = ptr - start;

    return true;
}


bool Bag::readMessageDataHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& total_bytes_read)
{
    total_bytes_read = 0;
    uint8_t op;
    do {
        ROS_DEBUG("reading header from buffer: offset=%d", offset);
        uint32_t bytes_read;
        if (!readHeaderFromBuffer(decompress_buffer_, offset, header, data_size, bytes_read))
            return false;
        offset += bytes_read;
        total_bytes_read += bytes_read;
        
        if (!readField(*header.getValues(), OP_FIELD_NAME, true, &op))
            return false;
    }
    while (op == OP_MSG_DEF);
    
    assert(op == OP_MSG_DATA);    

    return true;
}


bool Bag::readHeader(ros::Header& header, uint32_t& data_size) {
    // Read the header length
    uint32_t header_len;
    read((char*) &header_len, 4);

    // Read the header
    header_buffer_.setSize(header_len);
    read((char*) header_buffer_.getData(), header_len);

    // Parse the header
    string error_msg;
    bool parsed = header.parse(header_buffer_.getData(), header_len, error_msg);
    if (!parsed)
        return false;

    // Read the data size
    read((char*) &data_size, 4);

    return true;
}

M_string::const_iterator Bag::checkField(M_string const& fields, string const& field, unsigned int min_len, unsigned int max_len, bool required) const {
    M_string::const_iterator fitr = fields.find(field);
    if (fitr == fields.end()) {
        if (required)
            ROS_ERROR("Required '%s' field missing", field.c_str());
    }
    else if ((fitr->second.size() < min_len) || (fitr->second.size() > max_len)) {
        ROS_ERROR("Field '%s' is wrong size (%u bytes)", field.c_str(), (uint32_t) fitr->second.size());
        return fields.end();
    }

    return fitr;
}

bool Bag::readField(M_string const& fields, string const& field_name, bool required, string& data) {
    return readField(fields, field_name, 1, UINT_MAX, true, data);
}

bool Bag::readField(M_string const& fields, string const& field_name, unsigned int min_len, unsigned int max_len, bool required, string& data) {
    M_string::const_iterator i;
    if ((i = checkField(fields, field_name, min_len, max_len, required)) == fields.end())
        return false;
    data = i->second;
    return true;
}

bool Bag::readField(M_string const& fields, string const& field_name, bool required, Time& data) {
    uint64_t packed_time;
    if (!readField(fields, field_name, required, &packed_time))
        return false;

    uint64_t bitmask = (1LL << 33) - 1;
    data.sec  = (uint32_t) (packed_time & bitmask);
    data.nsec = (uint32_t) (packed_time >> 32);
    return true;
}

std::string Bag::toHeaderString(Time const* field) {
    uint64_t packed_time = (((uint64_t) field->nsec) << 32) + field->sec;
    return toHeaderString(&packed_time);
}

// Low-level I/O

void Bag::read(char* b, std::streamsize n)        { file_.read(b, n);             }
void Bag::write(string const& s)                  { write(s.c_str(), s.length()); }
void Bag::write(char const* s, std::streamsize n) { file_.write((char*) s, n);    }
void Bag::seek(uint64_t pos, int origin)          { file_.seek(pos, origin);      }

// Debugging

void Bag::dump() {
    std::cout << "chunk_open: " << chunk_open_ << std::endl;
    std::cout << "curr_chunk_info: " << curr_chunk_info_.topic_counts.size() << " topics" << std::endl;

    std::cout << "topic_infos:" << std::endl;
    for (map<string, TopicInfo*>::const_iterator i = topic_infos_.begin(); i != topic_infos_.end(); i++)
        std::cout << "  topic: " << i->first << std::endl;

    std::cout << "chunk_infos:" << std::endl;
    for (vector<ChunkInfo>::const_iterator i = chunk_infos_.begin(); i != chunk_infos_.end(); i++)
        std::cout << "  chunk: " << (*i).topic_counts.size() << " topics" << std::endl;

    std::cout << "topic_indexes:" << std::endl;
    for (map<string, vector<IndexEntry> >::const_iterator i = topic_indexes_.begin(); i != topic_indexes_.end(); i++) {
        std::cout << "  topic: " << i->first << std::endl;
        for (vector<IndexEntry>::const_iterator j = i->second.begin(); j != i->second.end(); j++) {
            std::cout << "    - " << j->chunk_pos << ":" << j->offset << std::endl;
        }
    }
}

}
