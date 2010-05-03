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

#include <iomanip>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using std::map;
using std::priority_queue;
using std::string;
using std::vector;
using std::multiset;
using boost::format;
using boost::shared_ptr;
using ros::M_string;
using ros::Time;

namespace rosbag {

Bag::Bag() :
    compression_(compression::None),
    chunk_threshold_(768 * 1024),  // 768KB chunks
    bag_revision_(0),
    file_size_(0),
    file_header_pos_(0),
    index_data_pos_(0),
    connection_count_(0),
    chunk_count_(0),
    chunk_open_(false),
    curr_chunk_data_pos_(0),
    decompressed_chunk_(0)
{
}

Bag::Bag(string const& filename, uint32_t mode) :
    compression_(compression::None),
    chunk_threshold_(768 * 1024),  // 768KB chunks
    bag_revision_(0),
    file_size_(0),
    file_header_pos_(0),
    index_data_pos_(0),
    connection_count_(0),
    chunk_count_(0),
    chunk_open_(false),
    curr_chunk_data_pos_(0),
    decompressed_chunk_(0)
{
    open(filename, mode);
}

Bag::~Bag() {
    close();

    for (map<uint32_t, ConnectionInfo*>::iterator i = connections_.begin(); i != connections_.end(); i++)
        delete i->second;
}

void Bag::open(string const& filename, uint32_t mode) {
    mode_ = (BagMode) mode;

    switch (mode_) {
    case bagmode::Read:   openRead(filename);   break;
    case bagmode::Write:  openWrite(filename);  break;
    case bagmode::Append: openAppend(filename); break;
    default:
        throw BagException((format("Unknown mode: %1%") % (int) mode).str());
    }

    // Determine file size
    uint64_t offset = file_.getOffset();
    seek(0, std::ios::end);
    file_size_ = file_.getOffset();
    seek(offset);
}

void Bag::openRead(string const& filename) {
    if (!file_.openRead(filename))
        throw BagIOException((format("Failed to open file: %1%") % filename.c_str()).str());

    readVersion();

    switch (version_) {
    case 102: startReadingVersion102(); break;
    case 200: startReadingVersion200(); break;
    default:
        throw BagException((format("Unsupported bag file version: %1%.%2%") % getMajorVersion() % getMinorVersion()).str());
    }
}

void Bag::openWrite(string const& filename) {
    if (!file_.openWrite(filename))
        throw BagIOException("Failed to open file: " + filename);

    startWriting();
}

void Bag::openAppend(string const& filename) {
    if (!file_.openReadWrite(filename))
        throw BagIOException("Failed to open file: " + filename);

    readVersion();
    startReadingVersion200();

    // Truncate the file to chop off the index
    file_.truncate(index_data_pos_);
    index_data_pos_ = 0;

    // Rewrite the file header, clearing the index position (so we know if the index is invalid)
    seek(file_header_pos_);
    writeFileHeaderRecord();

    // Seek to the end of the file
    seek(0, std::ios::end);
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
    stopWriting();
}

string   Bag::getFileName() const { return file_.getFileName(); }
BagMode  Bag::getMode()     const { return mode_;               }
uint64_t Bag::getSize()     const { return file_size_;          }

uint32_t Bag::getChunkThreshold() const { return chunk_threshold_; }

void Bag::setChunkThreshold(uint32_t chunk_threshold) {
    if (file_.isOpen() && chunk_open_)
        stopWritingChunk();

    chunk_threshold_ = chunk_threshold;
}

CompressionType Bag::getCompression() const { return compression_; }

void Bag::setCompression(CompressionType compression) {
    if (file_.isOpen() && chunk_open_)
        stopWritingChunk();

    compression_ = compression;
}

// Version

void Bag::writeVersion() {
    string version = string("#ROSBAG V") + VERSION + string("\n");

    ROS_INFO("Writing VERSION [%llu]: %s", (unsigned long long) file_.getOffset(), version.c_str());

    version_ = 200;

    write(version);
}

void Bag::readVersion() {
    string version_line = file_.getline();

    file_header_pos_ = file_.getOffset();

    char logtypename[100];
    int version_major, version_minor;
    if (sscanf(version_line.c_str(), "#ROS%s V%d.%d", logtypename, &version_major, &version_minor) != 3)
        throw BagIOException("Error reading version line");

    // Special case
    if (version_major == 0 && version_line[0] == '#')
        version_major = 1;

    version_ = version_major * 100 + version_minor;

    ROS_INFO("Read VERSION: version=%d", version_);
}

uint32_t Bag::getMajorVersion() const { return version_ / 100; }
uint32_t Bag::getMinorVersion() const { return version_ % 100; }

//

void Bag::startWriting() {
    writeVersion();
    file_header_pos_ = file_.getOffset();
    writeFileHeaderRecord();
}

void Bag::stopWriting() {
    if (chunk_open_)
        stopWritingChunk();

    index_data_pos_ = file_.getOffset();
    writeConnectionRecords();
    writeChunkInfoRecords();

    seek(file_header_pos_);
    writeFileHeaderRecord();
}

void Bag::startReadingVersion200() {
    // Read the file header record, which points to the end of the chunks
    readFileHeaderRecord();

    // Seek to the end of the chunks
    seek(index_data_pos_);

    // Read the connection records (one for each connection)
    for (uint32_t i = 0; i < connection_count_; i++)
        readConnectionRecord();

    // Read the chunk info records
    for (uint32_t i = 0; i < chunk_count_; i++)
        readChunkInfoRecord();

    // Read the connection indexes for each chunk
    foreach(ChunkInfo const& chunk_info, chunks_) {
        curr_chunk_info_ = chunk_info;

        seek(curr_chunk_info_.pos);

        // Skip over the chunk data
        ChunkHeader chunk_header;
        readChunkHeader(chunk_header);
        seek(chunk_header.compressed_size, std::ios::cur);

        // Read the index records after the chunk
        for (unsigned int i = 0; i < chunk_info.connection_counts.size(); i++)
            readConnectionIndexRecord200();
    }

    // At this point we don't have a curr_chunk_info anymore so we reset it
    curr_chunk_info_ = ChunkInfo();
}

// @todo: handle 1.2 bags with no index
void Bag::startReadingVersion102() {
    // Read the file header record, which points to the start of the topic indexes
    readFileHeaderRecord();

    // Get the length of the file
    seek(0, std::ios::end);
    uint64_t filelength = file_.getOffset();

    // Seek to the beginning of the topic index records
    seek(index_data_pos_);

    // Read the topic index records, which point to the offsets of each message in the file
    while (file_.getOffset() < filelength)
        readTopicIndexRecord102();

    // Read the message definition records (which are the first entry in the topic indexes)
    for (map<string, multiset<IndexEntry> >::const_iterator i = topic_indexes_.begin(); i != topic_indexes_.end(); i++) {
        multiset<IndexEntry> const& topic_index = i->second;
        IndexEntry const&           first_entry = *topic_index.begin();

        ROS_INFO("Reading message definition for %s at %llu", i->first.c_str(), (unsigned long long) first_entry.chunk_pos);

        seek(first_entry.chunk_pos);

        readMessageDefinitionRecord102();
    }
}

// File header record

void Bag::writeFileHeaderRecord() {
    boost::mutex::scoped_lock lock(record_mutex_);

    connection_count_ = connections_.size();
    chunk_count_      = chunks_.size();

    ROS_INFO("Writing FILE_HEADER [%llu]: index_pos=%llu connection_count=%d chunk_count=%d",
              (unsigned long long) file_.getOffset(), (unsigned long long) index_data_pos_, connection_count_, chunk_count_);
    
    // Write file header record
    M_string header;
    header[OP_FIELD_NAME]               = toHeaderString(&OP_FILE_HEADER);
    header[INDEX_POS_FIELD_NAME]        = toHeaderString(&index_data_pos_);
    header[CONNECTION_COUNT_FIELD_NAME] = toHeaderString(&connection_count_);
    header[CHUNK_COUNT_FIELD_NAME]      = toHeaderString(&chunk_count_);

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

void Bag::readFileHeaderRecord() {
    ros::Header header;
    uint32_t data_size;
    if (!readHeader(header) || !readDataLength(data_size))
        throw BagFormatException("Error reading FILE_HEADER record");

    M_string& fields = *header.getValues();

    if (!isOp(fields, OP_FILE_HEADER))
        throw BagFormatException("Expected FILE_HEADER op not found");

    // Read index position
    readField(fields, INDEX_POS_FIELD_NAME, true, (uint64_t*) &index_data_pos_);

    // Read topic and chunks count
    if (version_ >= 200) {
        readField(fields, CONNECTION_COUNT_FIELD_NAME, true, &connection_count_);
        readField(fields, CHUNK_COUNT_FIELD_NAME,      true, &chunk_count_);
    }

    ROS_INFO("Read FILE_HEADER: index_pos=%llu connection_count=%d chunk_count=%d",
              (unsigned long long) index_data_pos_, connection_count_, chunk_count_);

    // Skip the data section (just padding)
    seek(data_size, std::ios::cur);
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
    chunks_.push_back(curr_chunk_info_);
    
    // Get the uncompressed and compressed sizes
    uint32_t uncompressed_size = getChunkOffset();
    file_.setWriteMode(compression::None);
    uint32_t compressed_size = file_.getOffset() - curr_chunk_data_pos_;

    // Rewrite the chunk header with the size of the chunk (remembering current offset)
    uint64_t end_of_chunk_pos = file_.getOffset();
    seek(curr_chunk_info_.pos);
    writeChunkHeader(compression_, compressed_size, uncompressed_size);

    // Write out the indexes and clear them
    seek(end_of_chunk_pos);
    writeIndexRecords();
    curr_chunk_connection_indexes_.clear();
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

    ROS_INFO("Writing CHUNK [%llu]: compression=%s compressed=%d uncompressed=%d",
              (unsigned long long) file_.getOffset(), chunk_header.compression.c_str(), chunk_header.compressed_size, chunk_header.uncompressed_size);

    M_string header;
    header[OP_FIELD_NAME]          = toHeaderString(&OP_CHUNK);
    header[COMPRESSION_FIELD_NAME] = chunk_header.compression;
    header[SIZE_FIELD_NAME]        = toHeaderString(&chunk_header.uncompressed_size);
    writeHeader(header);

    writeDataLength(chunk_header.compressed_size);
}

void Bag::readChunkHeader(ChunkHeader& chunk_header) const {
    ros::Header header;
    if (!readHeader(header) || !readDataLength(chunk_header.compressed_size))
        throw BagFormatException("Error reading CHUNK record");
        
    M_string& fields = *header.getValues();
    
    if (!isOp(fields, OP_CHUNK))
        throw BagFormatException("Expected CHUNK op not found");

    readField(fields, COMPRESSION_FIELD_NAME, true, chunk_header.compression);
    readField(fields, SIZE_FIELD_NAME,        true, &chunk_header.uncompressed_size);

    ROS_INFO("Read CHUNK: compression=%s size=%d uncompressed=%d (%f)", chunk_header.compression.c_str(), chunk_header.compressed_size, chunk_header.uncompressed_size, 100 * ((double) chunk_header.compressed_size) / chunk_header.uncompressed_size);
}

// Index records

void Bag::writeIndexRecords() {
    boost::mutex::scoped_lock lock(record_mutex_);

    for (map<uint32_t, multiset<IndexEntry> >::const_iterator i = curr_chunk_connection_indexes_.begin(); i != curr_chunk_connection_indexes_.end(); i++) {
        uint32_t                    connection_id = i->first;
        multiset<IndexEntry> const& index         = i->second;

        // Write the index record header
        uint32_t index_size = index.size();
        M_string header;
        header[OP_FIELD_NAME]         = toHeaderString(&OP_INDEX_DATA);
        header[CONNECTION_FIELD_NAME] = toHeaderString(&connection_id);
        header[VER_FIELD_NAME]        = toHeaderString(&INDEX_VERSION);
        header[COUNT_FIELD_NAME]      = toHeaderString(&index_size);
        writeHeader(header);

        writeDataLength(index_size * sizeof(IndexEntry));

        ROS_INFO("Writing INDEX_DATA: connection=%d ver=%d count=%d", connection_id, INDEX_VERSION, index_size);

        // Write the index record data (pairs of timestamp and position in file)
        foreach(IndexEntry const& e, index) {
            write((char*) &e.time.sec,  4);
            write((char*) &e.time.nsec, 4);
            write((char*) &e.offset,    4);

            ROS_INFO("  - %d.%d: %d", e.time.sec, e.time.nsec, e.offset);
        }
    }
}

void Bag::readTopicIndexRecord102() {
    ros::Header header;
    uint32_t data_size;
    if (!readHeader(header) || !readDataLength(data_size))
        throw BagFormatException("Error reading INDEX_DATA header");
    M_string& fields = *header.getValues();

    if (!isOp(fields, OP_INDEX_DATA))
        throw BagFormatException("Expected INDEX_DATA record");

    uint32_t index_version;
    string topic;
    uint32_t count;
    readField(fields, VER_FIELD_NAME,   true, &index_version);
    readField(fields, TOPIC_FIELD_NAME, true, topic);
    readField(fields, COUNT_FIELD_NAME, true, &count);

    ROS_INFO("Read INDEX_DATA: ver=%d topic=%s count=%d", index_version, topic.c_str(), count);

    if (index_version != 0)
        throw BagFormatException((format("Unsupported INDEX_DATA version: %1%") % index_version).str());

    uint32_t connection_id;
    map<string, uint32_t>::const_iterator topic_conn_id_iter = topic_connection_ids_.find(topic);
    if (topic_conn_id_iter == topic_connection_ids_.end()) {
    	connection_id = connections_.size();

        ROS_INFO("Creating connection: id=%d topic=%s", connection_id, topic.c_str());
        ConnectionInfo* connection_info = new ConnectionInfo();
        connection_info->id       = connection_id;
        connection_info->topic    = topic;
        connections_[connection_id] = connection_info;

        topic_connection_ids_[topic] = connection_id;
    }
    else
    	connection_id = topic_conn_id_iter->second;

    multiset<IndexEntry>& connection_index = connection_indexes_[connection_id];
    multiset<IndexEntry>& topic_index      = topic_indexes_[topic];

    for (uint32_t i = 0; i < count; i++) {
        IndexEntry index_entry;
        uint32_t sec;
        uint32_t nsec;
        read((char*) &sec,                   4);
        read((char*) &nsec,                  4);
        read((char*) &index_entry.chunk_pos, 8);   //<! store position of the message in the chunk_pos field as it's 64 bits
        index_entry.time = Time(sec, nsec);
        index_entry.offset = 0;

        ROS_INFO("  - %d.%d: %llu", sec, nsec, (unsigned long long) index_entry.chunk_pos);

        connection_index.insert(connection_index.end(), index_entry);
        topic_index.insert(topic_index.end(), index_entry);
    }
}

void Bag::readConnectionIndexRecord200() {
    ros::Header header;
    uint32_t data_size;
    if (!readHeader(header) || !readDataLength(data_size))
        throw BagFormatException("Error reading INDEX_DATA header");
    M_string& fields = *header.getValues();
    
    if (!isOp(fields, OP_INDEX_DATA))
        throw BagFormatException("Expected INDEX_DATA record");
    
    uint32_t index_version;
    uint32_t connection_id;
    uint32_t count;
    readField(fields, VER_FIELD_NAME,        true, &index_version);
    readField(fields, CONNECTION_FIELD_NAME, true, &connection_id);
    readField(fields, COUNT_FIELD_NAME,      true, &count);

    ROS_INFO("Read INDEX_DATA: ver=%d connection=%d count=%d", index_version, connection_id, count);

    if (index_version != 1)
        throw BagFormatException((format("Unsupported INDEX_DATA version: %1%") % index_version).str());

    uint64_t chunk_pos = curr_chunk_info_.pos;

    ConnectionInfo*       connection_info  = connections_[connection_id];
    multiset<IndexEntry>& connection_index = connection_indexes_[connection_id];
    string const&         topic            = connection_info->topic;
    multiset<IndexEntry>& topic_index      = topic_indexes_[topic];

    for (uint32_t i = 0; i < count; i++) {
        IndexEntry index_entry;
        index_entry.chunk_pos = chunk_pos;
        uint32_t sec;
        uint32_t nsec;
        read((char*) &sec,                4);
        read((char*) &nsec,               4);
        read((char*) &index_entry.offset, 4);
        index_entry.time = Time(sec, nsec);

        ROS_INFO("  - %d.%d: %llu+%d", sec, nsec, (unsigned long long) index_entry.chunk_pos, index_entry.offset);

        connection_index.insert(connection_index.end(), index_entry);
        topic_index.insert(topic_index.end(), index_entry);
    }
}

// Connection records

void Bag::writeConnectionRecords() {
    boost::mutex::scoped_lock lock(record_mutex_);

    for (map<uint32_t, ConnectionInfo*>::const_iterator i = connections_.begin(); i != connections_.end(); i++) {
        ConnectionInfo const* connection_info = i->second;
        writeConnectionRecord(connection_info);
    }
}

void Bag::writeConnectionRecord(ConnectionInfo const* connection_info) {
    ROS_INFO("Writing CONNECTION [%llu:%d]: topic=%s id=%d",
              (unsigned long long) file_.getOffset(), getChunkOffset(), connection_info->topic.c_str(), connection_info->id);

    M_string header;
    header[OP_FIELD_NAME]         = toHeaderString(&OP_CONNECTION);
    header[TOPIC_FIELD_NAME]      = connection_info->topic;
    header[CONNECTION_FIELD_NAME] = toHeaderString(&connection_info->id);
    writeHeader(header);

    writeHeader(*connection_info->header);
}

void Bag::appendConnectionRecordToBuffer(Buffer& buf, ConnectionInfo const* connection_info) {
    M_string header;
    header[OP_FIELD_NAME]         = toHeaderString(&OP_CONNECTION);
    header[TOPIC_FIELD_NAME]      = connection_info->topic;
    header[CONNECTION_FIELD_NAME] = toHeaderString(&connection_info->id);
    appendHeaderToBuffer(buf, header);

    appendHeaderToBuffer(buf, *connection_info->header);
}

void Bag::readConnectionRecord() {
    ros::Header header;
    if (!readHeader(header))
        throw BagFormatException("Error reading CONNECTION header");
    M_string& fields = *header.getValues();

    if (!isOp(fields, OP_CONNECTION))
        throw BagFormatException("Expected CONNECTION op not found");

    uint32_t id;
    readField(fields, CONNECTION_FIELD_NAME, true, &id);
    string topic;
    readField(fields, TOPIC_FIELD_NAME,      true, topic);

    ros::Header connection_header;
    if (!readHeader(connection_header))
        throw BagFormatException("Error reading connection header");

    // If this is a new connection, update connections
    map<uint32_t, ConnectionInfo*>::iterator key = connections_.find(id);
    if (key == connections_.end()) {
        ConnectionInfo* connection_info = new ConnectionInfo();
        connection_info->id       = id;
        connection_info->topic    = topic;
        connection_info->header = shared_ptr<M_string>(new M_string);
        for (M_string::const_iterator i = connection_header.getValues()->begin(); i != connection_header.getValues()->end(); i++)
            (*connection_info->header)[i->first] = i->second;
        connection_info->msg_def  = (*connection_info->header)["message_definition"];
        connection_info->datatype = (*connection_info->header)["type"];
        connection_info->md5sum   = (*connection_info->header)["md5sum"];
        connections_[id] = connection_info;

        ROS_INFO("Read CONNECTION: topic=%s id=%d", topic.c_str(), id);
    }
}

void Bag::readMessageDefinitionRecord102() {
    ros::Header header;
    uint32_t data_size;
    if (!readHeader(header) || !readDataLength(data_size))
        throw BagFormatException("Error reading message definition header");
    M_string& fields = *header.getValues();

    if (!isOp(fields, OP_MSG_DEF))
        throw BagFormatException("Expected MSG_DEF op not found");

    string topic, md5sum, datatype, message_definition;
    readField(fields, TOPIC_FIELD_NAME,               true, topic);
    readField(fields, MD5_FIELD_NAME,   32,       32, true, md5sum);
    readField(fields, TYPE_FIELD_NAME,                true, datatype);
    readField(fields, DEF_FIELD_NAME,    0, UINT_MAX, true, message_definition);

    map<string, uint32_t>::const_iterator topic_conn_id_iter = topic_connection_ids_.find(topic);
    if (topic_conn_id_iter == topic_connection_ids_.end()) {
    	uint32_t id = connections_.size();

        ROS_INFO("Creating connection: topic=%s md5sum=%s datatype=%s", topic.c_str(), md5sum.c_str(), datatype.c_str());
        ConnectionInfo* connection_info = new ConnectionInfo();
        connection_info->id       = id;
        connection_info->topic    = topic;
        connection_info->msg_def  = message_definition;
        connection_info->datatype = datatype;
        connection_info->md5sum   = md5sum;
        connection_info->header = boost::shared_ptr<ros::M_string>(new ros::M_string);

        // TESTING
        (*connection_info->header)["callerid"]           = string("");
        (*connection_info->header)["latching"]           = string("0");

        (*connection_info->header)["type"]               = connection_info->datatype;
        (*connection_info->header)["md5sum"]             = connection_info->md5sum;
        (*connection_info->header)["message_definition"] = connection_info->msg_def;

        connections_[id] = connection_info;

        topic_connection_ids_[topic] = id;

        ROS_INFO("Read MSG_DEF: topic=%s md5sum=%s datatype=%s", topic.c_str(), md5sum.c_str(), datatype.c_str());
    }
    else {
    	uint32_t id = topic_conn_id_iter->second;

        ROS_INFO("Updating connection: topic=%s md5sum=%s datatype=%s", topic.c_str(), md5sum.c_str(), datatype.c_str());
        ConnectionInfo* connection_info = connections_[id];
        connection_info->msg_def  = message_definition;
        connection_info->datatype = datatype;
        connection_info->md5sum   = md5sum;
        connection_info->header = boost::shared_ptr<ros::M_string>(new ros::M_string);
        (*connection_info->header)["type"]               = connection_info->datatype;
        (*connection_info->header)["md5sum"]             = connection_info->md5sum;
        (*connection_info->header)["message_definition"] = connection_info->msg_def;

        ROS_INFO("Read MSG_DEF: topic=%s md5sum=%s datatype=%s", topic.c_str(), md5sum.c_str(), datatype.c_str());
    }
}

void Bag::decompressChunk(uint64_t chunk_pos) const {
    if (curr_chunk_info_.pos == chunk_pos) {
        current_buffer_ = &outgoing_chunk_buffer_;
        return;
    }

    current_buffer_ = &decompress_buffer_;

    if (decompressed_chunk_ == chunk_pos)
        return;

    // Seek to the start of the chunk
    seek(chunk_pos);

    // Read the chunk header
    ChunkHeader chunk_header;
    readChunkHeader(chunk_header);

    // Read and decompress the chunk.  These assume we are at the right place in the stream already
    if (chunk_header.compression == COMPRESSION_NONE)
        decompressRawChunk(chunk_header);
    else if (chunk_header.compression == COMPRESSION_BZ2)
        decompressBz2Chunk(chunk_header);
    else
        throw BagFormatException("Unknown compression: " + chunk_header.compression);
    
    decompressed_chunk_ = chunk_pos;
}

void Bag::loadMessageDataRecord102(uint64_t offset, ros::Header& header) const {
    ROS_INFO("loadMessageDataRecord: offset=%llu", (unsigned long long) offset);

    seek(offset);

    uint32_t data_size;
    uint8_t op;
    do {
        if (!readHeader(header) || !readDataLength(data_size))
            throw BagFormatException("Error reading header");

        readField(*header.getValues(), OP_FIELD_NAME, true, &op);
    }
    while (op == OP_MSG_DEF);

    if (op != OP_MSG_DATA)
        throw BagFormatException((format("Expected MSG_DATA op, got %d") % op).str());

    record_buffer_.setSize(data_size);
    file_.read((char*) record_buffer_.getData(), data_size);
}

// Reading this into a buffer isn't completely necessary, but we do it anyways for now
void Bag::decompressRawChunk(ChunkHeader const& chunk_header) const {
    assert(chunk_header.compression == COMPRESSION_NONE);
    assert(chunk_header.compressed_size == chunk_header.uncompressed_size);

    ROS_INFO("compressed_size: %d uncompressed_size: %d", chunk_header.compressed_size, chunk_header.uncompressed_size);

    decompress_buffer_.setSize(chunk_header.compressed_size);
    file_.read((char*) decompress_buffer_.getData(), chunk_header.compressed_size);

    // todo check read was successful
}

void Bag::decompressBz2Chunk(ChunkHeader const& chunk_header) const {
    assert(chunk_header.compression == COMPRESSION_BZ2);

    CompressionType compression = compression::BZ2;

    ROS_INFO("compressed_size: %d uncompressed_size: %d", chunk_header.compressed_size, chunk_header.uncompressed_size);

    chunk_buffer_.setSize(chunk_header.compressed_size);
    file_.read((char*) chunk_buffer_.getData(), chunk_header.compressed_size);

    decompress_buffer_.setSize(chunk_header.uncompressed_size);
    file_.decompress(compression, decompress_buffer_.getData(), decompress_buffer_.getSize(), chunk_buffer_.getData(), chunk_buffer_.getSize());

    // todo check read was successful
}

ros::Header Bag::readMessageDataHeader(IndexEntry const& index_entry) {
    ros::Header header;
    uint32_t data_size;
    uint32_t bytes_read;
    switch (version_)
    {
    case 200:
        decompressChunk(index_entry.chunk_pos);
        readMessageDataHeaderFromBuffer(*current_buffer_, index_entry.offset, header, data_size, bytes_read);
        return header;
    case 102:
        loadMessageDataRecord102(index_entry.chunk_pos, header);
        return header;
    default:
        throw BagFormatException((format("Unhandled version: %1%") % version_).str());
    }
}

// NOTE: this loads the header, which is unnecessary
uint32_t Bag::readMessageDataSize(IndexEntry const& index_entry) const {
    ros::Header header;
    uint32_t data_size;
    uint32_t bytes_read;
    switch (version_)
    {
    case 200:
        decompressChunk(index_entry.chunk_pos);
        readMessageDataHeaderFromBuffer(*current_buffer_, index_entry.offset, header, data_size, bytes_read);
        return data_size;
    case 102:
        loadMessageDataRecord102(index_entry.chunk_pos, header);
        return data_size;
    default:
        throw BagFormatException((format("Unhandled version: %1%") % version_).str());
    }
}

void Bag::writeChunkInfoRecords() {
    boost::mutex::scoped_lock lock(record_mutex_);

    foreach(ChunkInfo const& chunk_info, chunks_) {
        // Write the chunk info header
        M_string header;
        uint32_t chunk_connection_count = chunk_info.connection_counts.size();
        header[OP_FIELD_NAME]         = toHeaderString(&OP_CHUNK_INFO);
        header[VER_FIELD_NAME]        = toHeaderString(&CHUNK_INFO_VERSION);
        header[CHUNK_POS_FIELD_NAME]  = toHeaderString(&chunk_info.pos);
        header[START_TIME_FIELD_NAME] = toHeaderString(&chunk_info.start_time);
        header[END_TIME_FIELD_NAME]   = toHeaderString(&chunk_info.end_time);
        header[COUNT_FIELD_NAME]      = toHeaderString(&chunk_connection_count);

        ROS_INFO("Writing CHUNK_INFO [%llu]: ver=%d pos=%llu start=%d.%d end=%d.%d",
                  (unsigned long long) file_.getOffset(), CHUNK_INFO_VERSION, (unsigned long long) chunk_info.pos,
                  chunk_info.start_time.sec, chunk_info.start_time.nsec,
                  chunk_info.end_time.sec, chunk_info.end_time.nsec);

        writeHeader(header);

        writeDataLength(8 * chunk_connection_count);

        // Write the topic names and counts
        for (map<uint32_t, uint32_t>::const_iterator i = chunk_info.connection_counts.begin(); i != chunk_info.connection_counts.end(); i++) {
            uint32_t connection_id = i->first;
            uint32_t count         = i->second;

            write((char*) &connection_id, 4);
            write((char*) &count, 4);

            ROS_INFO("  - %d: %d", connection_id, count);
        }
    }
}

void Bag::readChunkInfoRecord() {
    // Read a CHUNK_INFO header
    ros::Header header;
    uint32_t data_size;
    if (!readHeader(header) || !readDataLength(data_size))
        throw BagFormatException("Error reading CHUNK_INFO record header");
    M_string& fields = *header.getValues();
    if (!isOp(fields, OP_CHUNK_INFO))
        throw BagFormatException("Expected CHUNK_INFO op not found");

    // Check that the chunk info version is current
    uint32_t chunk_info_version;
    readField(fields, VER_FIELD_NAME, true, &chunk_info_version);
    if (chunk_info_version != CHUNK_INFO_VERSION)
        throw BagFormatException((format("Expected CHUNK_INFO version %1%, read %2%") % CHUNK_INFO_VERSION % chunk_info_version).str());

    // Read the chunk position, timestamp, and topic count fields
    ChunkInfo chunk_info;
    readField(fields, CHUNK_POS_FIELD_NAME,  true, &chunk_info.pos);
    readField(fields, START_TIME_FIELD_NAME, true,  chunk_info.start_time);
    readField(fields, END_TIME_FIELD_NAME,   true,  chunk_info.end_time);
    uint32_t chunk_connection_count;
    readField(fields, COUNT_FIELD_NAME,      true, &chunk_connection_count);

    ROS_INFO("Read CHUNK_INFO: chunk_pos=%llu connection_count=%d start=%d.%d end=%d.%d",
              (unsigned long long) chunk_info.pos, chunk_connection_count,
              chunk_info.start_time.sec, chunk_info.start_time.nsec,
              chunk_info.end_time.sec, chunk_info.end_time.nsec);

    // Read the topic count entries
    for (uint32_t i = 0; i < chunk_connection_count; i ++) {
        uint32_t connection_id, connection_count;
        read((char*) &connection_id,    4);
        read((char*) &connection_count, 4);

        ROS_INFO("  %d: %d messages", connection_id, connection_count);

        chunk_info.connection_counts[connection_id] = connection_count;
    }

    chunks_.push_back(chunk_info);
}

// Record I/O

bool Bag::isOp(M_string& fields, uint8_t reqOp) const {
    uint8_t op;
    readField(fields, OP_FIELD_NAME, true, &op);
    return op == reqOp;
}

void Bag::writeHeader(M_string const& fields) {
    boost::shared_array<uint8_t> header_buffer;
    uint32_t header_len;
    ros::Header::write(fields, header_buffer, header_len);
    write((char*) &header_len, 4);
    write((char*) header_buffer.get(), header_len);
}

void Bag::writeDataLength(uint32_t data_len) {
    write((char*) &data_len, 4);
}

void Bag::appendHeaderToBuffer(Buffer& buf, M_string const& fields) {
    boost::shared_array<uint8_t> header_buffer;
    uint32_t header_len;
    ros::Header::write(fields, header_buffer, header_len);

    uint32_t offset = buf.getSize();

    buf.setSize(buf.getSize() + 4 + header_len);

    memcpy(buf.getData() + offset, &header_len, 4);
    offset += 4;
    memcpy(buf.getData() + offset, header_buffer.get(), header_len);
}

void Bag::appendDataLengthToBuffer(Buffer& buf, uint32_t data_len) {
    uint32_t offset = buf.getSize();

    buf.setSize(buf.getSize() + 4);

    memcpy(buf.getData() + offset, &data_len, 4);
}

//! \todo clean this up
void Bag::readHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read) const {
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
        throw BagFormatException("Error parsing header");
    ptr += header_len;

    // Read the data size
    memcpy(&data_size, ptr, 4);
    ptr += 4;

    bytes_read = ptr - start;
}

void Bag::readMessageDataHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& total_bytes_read) const {
    total_bytes_read = 0;
    uint8_t op;
    do {
        ROS_INFO("reading header from buffer: offset=%d", offset);
        uint32_t bytes_read;
        readHeaderFromBuffer(*current_buffer_, offset, header, data_size, bytes_read);

        offset += bytes_read;
        total_bytes_read += bytes_read;
        
        readField(*header.getValues(), OP_FIELD_NAME, true, &op);
    }
    while (op == OP_MSG_DEF || op == OP_CONNECTION);

    if (op != OP_MSG_DATA)
        throw BagFormatException("Expected MSG_DATA op not found");
}

bool Bag::readHeader(ros::Header& header) const {
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

    return true;
}

bool Bag::readDataLength(uint32_t& data_size) const {
    read((char*) &data_size, 4);
    return true;
}

M_string::const_iterator Bag::checkField(M_string const& fields, string const& field, unsigned int min_len, unsigned int max_len, bool required) const {
    M_string::const_iterator fitr = fields.find(field);
    if (fitr == fields.end()) {
        if (required)
            throw BagFormatException("Required '" + field + "' field missing");
    }
    else if ((fitr->second.size() < min_len) || (fitr->second.size() > max_len))
        throw BagFormatException((format("Field '%1%' is wrong size (%2% bytes)") % field % (uint32_t) fitr->second.size()).str());

    return fitr;
}

bool Bag::readField(M_string const& fields, string const& field_name, bool required, string& data) const {
    return readField(fields, field_name, 1, UINT_MAX, required, data);
}

bool Bag::readField(M_string const& fields, string const& field_name, unsigned int min_len, unsigned int max_len, bool required, string& data) const {
    M_string::const_iterator fitr = checkField(fields, field_name, min_len, max_len, required);
    if (fitr == fields.end())
    	return false;

    data = fitr->second;
    return true;
}

bool Bag::readField(M_string const& fields, string const& field_name, bool required, Time& data) const {
    uint64_t packed_time;
    if (!readField(fields, field_name, required, &packed_time))
    	return false;

    uint64_t bitmask = (1LL << 33) - 1;
    data.sec  = (uint32_t) (packed_time & bitmask);
    data.nsec = (uint32_t) (packed_time >> 32);

    return true;
}

std::string Bag::toHeaderString(Time const* field) const {
    uint64_t packed_time = (((uint64_t) field->nsec) << 32) + field->sec;
    return toHeaderString(&packed_time);
}

// Low-level I/O

void Bag::write(string const& s)                  { write(s.c_str(), s.length()); }
void Bag::write(char const* s, std::streamsize n) { file_.write((char*) s, n);    }

void Bag::read(char* b, std::streamsize n) const  { file_.read(b, n);             }
void Bag::seek(uint64_t pos, int origin) const    { file_.seek(pos, origin);      }

// Debugging

void Bag::dump() {
    std::cout << "chunk_open: " << chunk_open_ << std::endl;
    std::cout << "curr_chunk_info: " << curr_chunk_info_.connection_counts.size() << " connections" << std::endl;

    std::cout << "connections:" << std::endl;
    for (map<uint32_t, ConnectionInfo*>::const_iterator i = connections_.begin(); i != connections_.end(); i++)
        std::cout << "  connection: " << i->first << " " << i->second->topic << std::endl;

    std::cout << "chunks:" << std::endl;
    for (vector<ChunkInfo>::const_iterator i = chunks_.begin(); i != chunks_.end(); i++) {
        std::cout << "  chunk: " << (*i).connection_counts.size() << " connections" << std::endl;
        for (map<uint32_t, uint32_t>::const_iterator j = (*i).connection_counts.begin(); j != (*i).connection_counts.end(); j++) {
            std::cout << "    - " << j->first << ": " << j->second << std::endl;
        }
    }

    std::cout << "connection_indexes:" << std::endl;
    for (map<uint32_t, multiset<IndexEntry> >::const_iterator i = connection_indexes_.begin(); i != connection_indexes_.end(); i++) {
        std::cout << "  connection: " << i->first << std::endl;
        for (multiset<IndexEntry>::const_iterator j = i->second.begin(); j != i->second.end(); j++) {
            std::cout << "    - " << j->chunk_pos << ":" << j->offset << std::endl;
        }
    }

    std::cout << "topic_indexes:" << std::endl;
    for (map<string, multiset<IndexEntry> >::const_iterator i = topic_indexes_.begin(); i != topic_indexes_.end(); i++) {
        std::cout << "  topic: " << i->first << std::endl;
        for (multiset<IndexEntry>::const_iterator j = i->second.begin(); j != i->second.end(); j++) {
            std::cout << "    - " << j->chunk_pos << ":" << j->offset << std::endl;
        }
    }
}

}
