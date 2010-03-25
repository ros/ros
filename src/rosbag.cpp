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

#include "rosbag/rosbag.h"

#include "rosbag/constants.h"

#include <iomanip>
#include <signal.h>
#include <sys/statvfs.h>

#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>

using namespace rosbag;

rosbag::Bag::Bag()
  : file_header_pos_(0), index_data_pos_(0), header_buf_(NULL), header_buf_len_(0), header_buf_size_(0),
    message_buf_(NULL), message_buf_len_(0), message_buf_size_(0), writing_enabled_(true)
{
}

rosbag::Bag::~Bag()
{
  close();
}

const void* rosbag::Bag::getHeaderBuffer()
{
  return header_buf_;
}

unsigned int rosbag::Bag::getHeaderBufferLength()
{
  return header_buf_len_;
}

void rosbag::Bag::resetHeaderBuffer()
{
  header_buf_len_ = 0;
}

void rosbag::Bag::writeFieldToHeaderBuffer(const std::string& name, const void* value, unsigned int value_len)
{
  // Do a little buffer-size management.
  unsigned int new_len = header_buf_len_ + name.size() + 1 + 4 + value_len;
  if (header_buf_size_ < new_len)
  {
    if (header_buf_size_ == 0)
      header_buf_size_ = new_len;
    else
    {
      while (header_buf_size_ < new_len)
        header_buf_size_ *= 2;
    }
    header_buf_ = (unsigned char*) realloc(header_buf_, header_buf_size_);
    ROS_ASSERT(header_buf_);
  }

  // Copy in the data as
  //   <name>=<value_len><value>
  // where <value_len> is a 4-byte little-endian integer.
  //
  // (see http://pr.willowgarage.com/wiki/ROS/LogFormat)
  memcpy(header_buf_ + header_buf_len_, name.c_str(), name.size());
  header_buf_len_ += name.size();
  header_buf_[header_buf_len_] = FIELD_DELIM;
  header_buf_len_ += 1;
  memcpy(header_buf_ + header_buf_len_, &value_len, 4);
  header_buf_len_ += 4;
  memcpy(header_buf_ + header_buf_len_, value, value_len);
  header_buf_len_ += value_len;
}

bool rosbag::Bag::open(const std::string &file_name, int mode)
{
  mode_ = mode;
  file_name_ = file_name;

  if (readMode() && writeMode())
  {
    ROS_FATAL("Simultaneous reading and writing not supported yet.");
    assert(0);
  }

  if (appendMode())
  {
    ROS_FATAL("Appending not supported yet.");
    assert(0);
  }
  

  if (writeMode())
  {
    // Append mode will require:
    // write_stream_.open(file_name.c_str(), std::ios::in | std::ios::out | std::ios::binary);
    write_stream_.open(file_name.c_str(), std::ios::out | std::ios::binary);

    if (write_stream_.fail())
    {
      ROS_FATAL("Failed to open file: %s", file_name.c_str());
      return false;
    }
    
    check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);
    warn_next_ = ros::WallTime();
    
    record_pos_ = 0;
    
    checkDisk();

    writeVersion();
    writeFileHeader();
  }

  if (readMode())
  {
    read_stream_.open(file_name.c_str(), std::ios::in | std::ios::binary);

    if (write_stream_.fail())
    {
      ROS_FATAL("Failed to open file: %s", file_name.c_str());
      return false;
    }

    if (writeMode())
      read_stream_.tie(&write_stream_);

    ROS_ERROR("Reading version");
    readVersion();
    ROS_ERROR("Reading file header");
    readFileHeader();
    ROS_ERROR("Reading index");
    readIndex();
    ROS_ERROR("Loading defs");
    readDefs();
  }

  return true;
}


bool rosbag::Bag::readVersion()
{
    char logtypename[100];

    std::string version_line;
    getline(read_stream_, version_line);

    sscanf(version_line.c_str(), "#ROS%s V%d.%d", logtypename, &version_major_, &version_minor_);

    if (version_major_ == 0 && version_line[0] == '#')
    {
      version_major_ = 1;
    }

    version_ = version_major_ * 100 + version_minor_;

    int cur_version_major;
    int cur_version_minor;
    sscanf(VERSION.c_str(), "%d.%d", &cur_version_major, &cur_version_minor);

    if (version_major_ != cur_version_major && version_minor_ != cur_version_minor)
    {
      ROS_FATAL("Rosbag does not currently support reading anything but the current version.");
      assert(0);
    }

    return true;
}


// Parse a Version 1.2 header, which is a sequence of
// <name>=<value_len><value> fields.
//
// Writes the value of the 'op' field into op; if op is OP_MSG_DATA, then
// next_msg_dur gets filled in with the timestamp.
//
// Returns true on success, false otherwise.  On success, everything up
// through the data_len field has been read, leaving just the serialized
// message body in the file.
bool rosbag::Bag::readHeader(ros::Header& header, uint32_t& next_msg_size)
{
  unsigned int header_len;

  // Read the header length
  read_stream_.read((char*)&header_len, 4);
  ROS_ERROR_STREAM("Read " << read_stream_.gcount() << "/4");
  if (read_stream_.eof())
    return false;

  if (header_buf_len_ < header_len)
  {
    header_buf_len_ = header_len;
    header_buf_ = (unsigned char*)realloc(header_buf_,
                                             header_buf_len_);
    ROS_ASSERT(header_buf_);
  }

  // Read the header
  read_stream_.read((char*)header_buf_, header_len);
  if (read_stream_.eof())
    return false;

  // Parse the header
  std::string error_msg;
  bool parsed = header.parse(header_buf_, header_len, error_msg);
  if (!parsed)
  {
    ROS_ERROR("Error parsing header: %s", error_msg.c_str());
    return false;
  }

  read_stream_.read((char*)&next_msg_size, 4);

  return true;
}

void rosbag::Bag::close()
{
  if (!write_stream_.is_open())
    return;

  writeIndex();

  topics_recorded_.clear();
  topic_indexes_.clear();

  // Unfortunately closing this possibly enormous file takes a while
  // (especially over NFS) and handling of a SIGINT while a file is
  // closing leads to a double free.  So, we disable signals while
  // we close the file.

  // Darwin doesn't have sighandler_t; I hope that sig_t on Linux does
  // the right thing.
  //sighandler_t old = signal(SIGINT, SIG_IGN);

  sig_t old = signal(SIGINT, SIG_IGN);
  if (write_stream_.is_open())
  {
    write_stream_.close();
  }
  signal(SIGINT, old);
}

void rosbag::Bag::writeVersion()
{
  std::string version = std::string("#ROSRECORD V") + VERSION + std::string("\n");
  writefil(version);
}

bool rosbag::Bag::checkDisk()
{
  struct statvfs fiData;

  if ((statvfs(file_name_.c_str(), &fiData)) < 0)
  {
    ROS_WARN("rosrecord::Record: Failed to check filesystem stats.");
  }
  else
  {
    unsigned long long free_space = 0;

    free_space = (unsigned long long)(fiData.f_bsize) * (unsigned long long)(fiData.f_bavail);

    if (free_space < 1073741824ull)
    {
      ROS_ERROR("rosrecord::Record: Less than 1GB of space free on disk with %s.  Disabling logging.", file_name_.c_str());
      writing_enabled_ = false;
      return false;
    }
    else if (free_space < 5368709120ull)
    {
      ROS_WARN("rosrecord::Record: Less than 5GB of space free on disk with %s.", file_name_.c_str());
    }
    else
    {
      writing_enabled_ = true;
    }
  }
  return true;
}

void rosbag::Bag::write(const std::string& topic_name, ros::Time time, ros::Message::ConstPtr msg)
{
  write(topic_name, time, *msg);
}

void rosbag::Bag::write(const std::string& topic_name, ros::Time time, const ros::Message& msg)
{
  if (!writing_enabled_)
  {
    ros::WallTime nowtime = ros::WallTime::now();
    if (nowtime > warn_next_)
    {
      warn_next_ = nowtime + ros::WallDuration().fromSec(5.0);
      ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return;
  }

  bool needs_def_written = false;
  std::map<std::string, MsgInfo>::iterator key;
  {
    boost::mutex::scoped_lock lock(topics_recorded_mutex_);

    key = topics_recorded_.find(topic_name);

    if (key == topics_recorded_.end())
    {
      MsgInfo& info = topics_recorded_[topic_name];
      info.msg_def  = msg.__getMessageDefinition();
      info.datatype = msg.__getDataType();
      info.md5sum   = msg.__getMD5Sum();

      key = topics_recorded_.find(topic_name);

      topic_indexes_[topic_name] = std::vector<IndexEntry>();

      needs_def_written = true;
    }
  }
  const MsgInfo& msg_info = key->second;

  {
    boost::mutex::scoped_lock lock(check_disk_mutex_);

    if (ros::WallTime::now() > check_disk_next_)
    {
      check_disk_next_ = check_disk_next_ + ros::WallDuration().fromSec(20.0);

      checkDisk();
    }
  }


  // Get information about possible latching and callerid from the connection header
  bool latching = false;
  std::string callerid("");
  
  if (msg.__connection_header != NULL)
  {
    ros::M_string::iterator latch_iter = msg.__connection_header->find(std::string("latching"));
    if (latch_iter != msg.__connection_header->end())
    {
      if (latch_iter->second != std::string("0"))
      {
        latching = true;
      }
    }

    ros::M_string::iterator callerid_iter = msg.__connection_header->find(std::string("callerid"));
    if (callerid_iter != msg.__connection_header->end())
    {
      callerid = callerid_iter->second;
    }
  }

  {
    boost::mutex::scoped_lock lock(record_mutex_);

    // Assemble the header in memory first, because we need to write its length first.

    // Add to topic index
    IndexEntry index_entry;
    index_entry.sec  = time.sec;
    index_entry.nsec = time.nsec;
    index_entry.pos  = record_pos_;
    topic_indexes_[topic_name].push_back(index_entry);

    // Write a message definition record, if necessary
    if (needs_def_written)
    {
      ros::M_string header;
      header[OP_FIELD_NAME]    = std::string((char*)&OP_MSG_DEF, 1);
      header[TOPIC_FIELD_NAME] = topic_name;
      header[MD5_FIELD_NAME]   = msg_info.md5sum;
      header[TYPE_FIELD_NAME]  = msg_info.datatype;
      header[DEF_FIELD_NAME]   = msg_info.msg_def;
      writeHeader(header, 0);
    }

    // Serialize the message into the message buffer
    if (message_buf_size_ < msg.serializationLength())
    {
      if (message_buf_size_ == 0)
        message_buf_size_ = msg.serializationLength();
      else
      {
        while (message_buf_size_ < msg.serializationLength())
          message_buf_size_ *= 2;
      }
      message_buf_ = (unsigned char*)realloc(message_buf_, message_buf_size_);
      ROS_ASSERT(message_buf_);
    }
    msg.serialize(message_buf_, 0);

    // Write a message instance record
    ros::M_string header;
    header[OP_FIELD_NAME]    = std::string((char*)&OP_MSG_DATA, 1);
    header[TOPIC_FIELD_NAME] = topic_name;
    header[MD5_FIELD_NAME]   = msg_info.md5sum;
    header[TYPE_FIELD_NAME]  = msg_info.datatype;
    header[SEC_FIELD_NAME]   = std::string((char*)&time.sec, 4);
    header[NSEC_FIELD_NAME]  = std::string((char*)&time.nsec, 4);

    if (latching)
    {
      header[LATCHING_FIELD_NAME] = std::string("1");
      header[CALLERID_FIELD_NAME] = callerid;
    }

    writeRecord(header, (char*)message_buf_, msg.serializationLength());
    if (write_stream_.fail())
    {
      ROS_FATAL("rosrecord::Record: could not write to file.  Check permissions and diskspace\n");
    }
  }
}

void rosbag::Bag::writeFileHeader()
{
  boost::mutex::scoped_lock lock(record_mutex_);

  // Remember position of file header record
  file_header_pos_ = record_pos_;

  // Write file header record
  ros::M_string header;
  header[OP_FIELD_NAME]        = std::string((char*)&OP_FILE_HEADER, 1);
  header[INDEX_POS_FIELD_NAME] = std::string((char*)&index_data_pos_, 8);

  ROS_ERROR("Writing index_data_pos %llu", index_data_pos_);

  boost::shared_array<uint8_t> header_buffer;
  uint32_t header_len;
  ros::Header::write(header, header_buffer, header_len);
  uint32_t data_len = 0;
  if (header_len < FILE_HEADER_LENGTH)
    data_len = FILE_HEADER_LENGTH - header_len;
  writefil((char*)&header_len, 4);
  writefil((char*)header_buffer.get(), header_len);
  writefil((char*)&data_len, 4);

  // Pad the file header record out
  if (data_len > 0)
  {
    std::string padding;
    padding.resize(data_len, ' ');
    writefil(padding);
  }
}

bool rosbag::Bag::readFileHeader()
{
  ros::Header header;
  uint32_t data_size;

  readHeader(header, data_size);

  ros::M_string::const_iterator fitr;
  ros::M_stringPtr fields_ptr = header.getValues();
  ros::M_string& fields = *fields_ptr;

  unsigned char op;
  //  uint64_t index_data_pos;
  
  if((fitr = checkField(fields, OP_FIELD_NAME,
                        1, 1, true)) == fields.end())
    return false;

  memcpy(&op,fitr->second.data(),1);

  assert(op == OP_FILE_HEADER);

  if((fitr = checkField(fields, INDEX_POS_FIELD_NAME,
                        8, 8, true)) == fields.end())
    return false;

  memcpy(&index_data_pos_,fitr->second.data(),8);

  read_stream_.seekg(data_size, std::ios::cur);

  return true;
}

void rosbag::Bag::writeIndex()
{
  {
    boost::mutex::scoped_lock lock(record_mutex_);
    
    // Remember position of first index record
    index_data_pos_ = record_pos_;
    
    for (std::map<std::string, std::vector<IndexEntry> >::const_iterator i = topic_indexes_.begin();
         i != topic_indexes_.end();
         i++)
    {
      const std::string&             topic_name  = i->first;
      const std::vector<IndexEntry>& topic_index = i->second;
      
      uint32_t topic_index_size = topic_index.size();
      
      const MsgInfo& msg_info = topics_recorded_[topic_name];
      
      // Write the index record header
      ros::M_string header;
      header[OP_FIELD_NAME]    = std::string((char*)&OP_INDEX_DATA, 1);
      header[TOPIC_FIELD_NAME] = topic_name;
      header[TYPE_FIELD_NAME]  = msg_info.datatype;
      header[VER_FIELD_NAME]   = std::string((char*)&INDEX_VERSION, 4);
      header[COUNT_FIELD_NAME] = std::string((char*)&topic_index_size, 4);
      
      uint32_t data_len = topic_index_size * sizeof(IndexEntry);
      writeHeader(header, data_len);
      
      // Write the index record data (pairs of timestamp and position in file)
      for (std::vector<IndexEntry>::const_iterator j = topic_index.begin(); j != topic_index.end(); j++)
      {
        const IndexEntry& index_entry = *j;
        writefil((char*)&index_entry.sec,  4);
        writefil((char*)&index_entry.nsec, 4);
        writefil((char*)&index_entry.pos,  8);
      }
    }
  }
  
  seek(file_header_pos_);
  writeFileHeader();
}


bool rosbag::Bag::readIndex()
{

  ROS_ERROR("Seeking to %llu", index_data_pos_);

  read_stream_.seekg(index_data_pos_, std::ios::beg);

  do 
  {
    ros::Header header;
    uint32_t data_size;

    if (!readHeader(header, data_size))
      continue;

    ROS_ERROR("Loop");
    
    ros::M_string::const_iterator fitr;
    ros::M_stringPtr fields_ptr = header.getValues();
    ros::M_string& fields = *fields_ptr;
    
    unsigned char op;
    
    if((fitr = checkField(fields, OP_FIELD_NAME,
                          1, 1, true)) == fields.end())
      return false;

    memcpy(&op,fitr->second.data(),1);

    assert(op == OP_INDEX_DATA);

    if((fitr = checkField(fields, VER_FIELD_NAME,
                          4, 4, true)) == fields.end())
      return false;

    uint32_t index_version;

    memcpy(&index_version,fitr->second.data(),4);

    assert(index_version == INDEX_VERSION);

    std::string topic_name;
    std::string datatype;

    uint32_t count;
    
    if((fitr = checkField(fields, TOPIC_FIELD_NAME,
                          1, UINT_MAX, true)) == fields.end())
      return false;
    topic_name = fitr->second;

    if((fitr = checkField(fields, TYPE_FIELD_NAME,
                          1, UINT_MAX, true)) == fields.end())
      return false;
    datatype = fitr->second;      
    
    if((fitr = checkField(fields, COUNT_FIELD_NAME,
                          4, 4, true)) == fields.end())
      return false;
    
    memcpy(&count,fitr->second.data(),4);    

    assert(sizeof(IndexEntry) == 16);

    ROS_ERROR("%d x %d  = %u", count, sizeof(IndexEntry), data_size);

    assert(count*sizeof(IndexEntry) == data_size);

    std::vector<IndexEntry>& topic_index = topic_indexes_[topic_name];


    for (uint32_t i = 0; i < count; i ++)
    {
      IndexEntry index_entry;
      read_stream_.read((char*)&index_entry.sec, 4);
      read_stream_.read((char*)&index_entry.nsec, 4);
      read_stream_.read((char*)&index_entry.pos, 8);

      topic_index.push_back(index_entry);
    }
    
  } while (read_stream_.good());
  // We've read to end of file... reset
  read_stream_.clear();

  return true;
}


bool rosbag::Bag::readDefs()
{

  for (std::map<std::string, std::vector<IndexEntry> >::const_iterator i = topic_indexes_.begin();
       i != topic_indexes_.end();
       i++)
  {
    const std::vector<IndexEntry>& topic_index = i->second;
    
    std::vector<IndexEntry>::const_iterator j = topic_index.begin();

    ROS_ERROR("Trying to read def for %s at %llu", i->first.c_str(), j->pos);

    
    readDef(j->pos);
  }

  return true;
}

bool rosbag::Bag::readDef(uint64_t pos)
{

  ROS_ERROR("Seeking to %llu", pos);

  read_stream_.seekg(pos, std::ios::beg);

  // Remember position of first index record
  index_data_pos_ = record_pos_;
  
  ros::Header header;
  uint32_t data_size;

  if (!readHeader(header, data_size))
    return false;

  ros::M_string::const_iterator fitr;
  ros::M_stringPtr fields_ptr = header.getValues();
  ros::M_string& fields = *fields_ptr;
    
  unsigned char op;
    
  if((fitr = checkField(fields, OP_FIELD_NAME,
                        1, 1, true)) == fields.end())
    return false;

  memcpy(&op,fitr->second.data(),1);

  assert(op == OP_MSG_DEF);


  std::string topic_name;
  std::string md5sum;
  std::string datatype;
  std::string message_definition;

  if((fitr = checkField(fields, TOPIC_FIELD_NAME,
                        1, UINT_MAX, true)) == fields.end())
    return false;
  topic_name = fitr->second;
  
  if((fitr = checkField(fields, MD5_FIELD_NAME,
                        32, 32, true)) == fields.end())
    return false;
  md5sum = fitr->second;
  
  if((fitr = checkField(fields, TYPE_FIELD_NAME,
                        1, UINT_MAX, true)) == fields.end())
    return false;
  datatype = fitr->second;      
  
  // Note that the field length can be zero.  This can happen if a
  // publisher didn't supply the definition, e.g., this bag was created
  // by recording from the playback of a pre-1.2 bag.
  if((fitr = checkField(fields, DEF_FIELD_NAME,
                        0, UINT_MAX, true)) == fields.end())
    return false;
  message_definition = fitr->second;

  std::map<std::string, MsgInfo>::iterator key = topics_recorded_.find(topic_name);

  if (key == topics_recorded_.end())
  {
    MsgInfo& info = topics_recorded_[topic_name];
    info.msg_def  = message_definition;
    info.datatype = datatype;
    info.md5sum   = md5sum;

    ROS_ERROR("Index loaded for %s [%s]", topic_name.c_str(), datatype.c_str());
  }

  return true;
}


void rosbag::Bag::writeRecord(const ros::M_string& fields, const char* data, uint32_t data_len)
{
  writeHeader(fields, data_len);
  writefil(data, data_len);
}

void rosbag::Bag::writeHeader(const ros::M_string& fields, uint32_t data_len)
{
  boost::shared_array<uint8_t> header_buffer;
  uint32_t header_len;
  ros::Header::write(fields, header_buffer, header_len);

  writefil((char*)&header_len, 4);
  writefil((char*)header_buffer.get(), header_len);
  writefil((char*)&data_len, 4);
}

void rosbag::Bag::writefil(const char* s, std::streamsize n)
{
  write_stream_.write(s, n);
  record_pos_ += n;
}

void rosbag::Bag::writefil(const std::string& s)
{
  writefil(s.c_str(), s.length());
}

void rosbag::Bag::seek(pos_t pos)
{
  write_stream_.seekp(pos, std::ios_base::beg);
  record_pos_ = pos;
}


ros::M_string::const_iterator
rosbag::Bag::checkField(const ros::M_string& fields,
           const std::string& field,
           unsigned int min_len,
           unsigned int max_len,
           bool required)
{
  ros::M_string::const_iterator fitr;
  fitr = fields.find(field);
  if(fitr == fields.end())
  {
    if(required)
      ROS_ERROR("Required %s field missing", field.c_str());
  }
  else if((fitr->second.size() < min_len) ||
          (fitr->second.size() > max_len))
  {
    ROS_ERROR("Field %s is wrong size (%u bytes)",
              field.c_str(), (uint32_t)fitr->second.size());
    return fields.end();
  }

  return fitr;
}



rosbag::MessageList rosbag::Bag::getMessageListByTopic(const std::vector<std::string>& topics,
                                         const ros::Time& start_time, 
                                         const ros::Time& end_time)
{
  rosbag::MessageList message_list(*this, topics, start_time, end_time);

  return message_list;
}


rosbag::MessageList::MessageList(const Bag& bag,
                const std::vector<std::string>& topics,
                const ros::Time& start_time,
                const ros::Time& end_time) : bag_(bag), topics_(topics), start_time_(start_time), end_time_(end_time)
{
  
  // Todo: calculation count properly
  count_ = 1;

}

rosbag::MessageList::iterator rosbag::MessageList::begin() const
{
  rosbag::MessageList::iterator iter(*this, 0);
  return iter;
}

rosbag::MessageList::iterator rosbag::MessageList::end() const
{
  rosbag::MessageList::iterator iter(*this, count_);
  return iter;
}
