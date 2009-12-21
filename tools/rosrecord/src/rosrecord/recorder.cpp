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

#include "rosrecord/Recorder.h"

#include "rosrecord/constants.h"

#include <iomanip>
#include <signal.h>
#include <sys/statvfs.h>

#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>

using namespace ros::record;

ros::record::Recorder::Recorder()
  : file_header_pos_(0), index_data_pos_(0), header_buf_(NULL), header_buf_len_(0), header_buf_size_(0),
    message_buf_(NULL), message_buf_len_(0), message_buf_size_(0), logging_enabled_(true)
{
}

ros::record::Recorder::~Recorder()
{
  close();
}

const void* ros::record::Recorder::getHeaderBuffer()
{
  return header_buf_;
}

unsigned int ros::record::Recorder::getHeaderBufferLength()
{
  return header_buf_len_;
}

void ros::record::Recorder::resetHeaderBuffer()
{
  header_buf_len_ = 0;
}

void ros::record::Recorder::writeFieldToHeaderBuffer(const std::string& name, const void* value, unsigned int value_len)
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

bool ros::record::Recorder::open(const std::string& file_name, bool random_access)
{
  if (!openFile(file_name, random_access))
    return false;

  writeVersion();
  writeFileHeader();

  return true;
}

bool ros::record::Recorder::openFile(const std::string& file_name, bool random_access)
{
  file_name_ = file_name;

  std::string ext = boost::filesystem::extension(file_name);

  store_index_ = !(ext == ".gz" || ext == ".bz2");
  if (store_index_ && random_access)
    record_file_.open(file_name.c_str(), std::ios::in | std::ios::out | std::ios::binary);
  else
    record_file_.open(file_name.c_str(), std::ios::out | std::ios::binary | std::ios::app);

  if (record_file_.fail())
  {
    ROS_FATAL("rosrecord::Record: Failed to open file: %s", file_name.c_str());
    return false;
  }

  // Removing compression until we work out how to play nicely with index: JML
  /*
  if (ext == ".gz")
    record_stream_.push(boost::iostreams::gzip_compressor());
  else if (ext == ".bz2")
    record_stream_.push(boost::iostreams::bzip2_compressor());
  */
  record_stream_.push(record_file_);

  check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);
  warn_next_ = ros::WallTime();

  record_pos_ = 0;

  checkDisk();

  return true;
}

pos_t ros::record::Recorder::getOffset()
{
  return record_pos_;
}

void ros::record::Recorder::close()
{
  if (!record_file_.is_open())
    return;

  if (store_index_)
    writeIndex();

  topics_recorded_.clear();
  topic_indexes_.clear();

  closeFile();
}

void ros::record::Recorder::closeFile()
{
  // Unfortunately closing this possibly enormous file takes a while
  // (especially over NFS) and handling of a SIGINT while a file is
  // closing leads to a double free.  So, we disable signals while
  // we close the file.

  // Darwin doesn't have sighandler_t; I hope that sig_t on Linux does
  // the right thing.
  //sighandler_t old = signal(SIGINT, SIG_IGN);
  sig_t old = signal(SIGINT, SIG_IGN);
  if (record_file_.is_open())
  {
    while (!record_stream_.empty())
      record_stream_.pop();
    record_file_.close();
  }
  signal(SIGINT, old);
}

void ros::record::Recorder::writeVersion()
{
  std::string version = std::string("#ROSRECORD V") + VERSION + std::string("\n");
  write(version);
}

bool ros::record::Recorder::checkDisk()
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
      logging_enabled_ = false;
      return false;
    }
    else if (free_space < 5368709120ull)
    {
      ROS_WARN("rosrecord::Record: Less than 5GB of space free on disk with %s.", file_name_.c_str());
    }
    else
    {
      logging_enabled_ = true;
    }
  }
  return true;
}

bool ros::record::Recorder::record(std::string topic_name, ros::Message::ConstPtr msg, ros::Time time)
{
  if (!logging_enabled_)
  {
    ros::WallTime nowtime = ros::WallTime::now();
    if (nowtime > warn_next_)
    {
      warn_next_ = nowtime + ros::WallDuration().fromSec(5.0);
      ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
  }

  bool needs_def_written = false;
  std::map<std::string, MsgInfo>::iterator key;
  {
    boost::mutex::scoped_lock lock(topics_recorded_mutex_);

    key = topics_recorded_.find(topic_name);

    if (key == topics_recorded_.end())
    {
      MsgInfo& info = topics_recorded_[topic_name];
      info.msg_def  = msg->__getMessageDefinition();
      info.datatype = msg->__getDataType();
      info.md5sum   = msg->__getMD5Sum();

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

      if (!checkDisk())
        return false;
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
      M_string header;
      header[OP_FIELD_NAME]    = std::string((char*)&OP_MSG_DEF, 1);
      header[TOPIC_FIELD_NAME] = topic_name;
      header[MD5_FIELD_NAME]   = msg_info.md5sum;
      header[TYPE_FIELD_NAME]  = msg_info.datatype;
      header[DEF_FIELD_NAME]   = msg_info.msg_def;
      writeHeader(header, 0);
    }

    // Serialize the message into the message buffer
    if (message_buf_size_ < msg->__serialized_length)
    {
      if (message_buf_size_ == 0)
        message_buf_size_ = msg->__serialized_length;
      else
      {
        while (message_buf_size_ < msg->__serialized_length)
          message_buf_size_ *= 2;
      }
      message_buf_ = (unsigned char*)realloc(message_buf_, message_buf_size_);
      ROS_ASSERT(message_buf_);
    }
    msg->serialize(message_buf_, 0);

    // Write a message instance record
    M_string header;
    header[OP_FIELD_NAME]    = std::string((char*)&OP_MSG_DATA, 1);
    header[TOPIC_FIELD_NAME] = topic_name;
    header[MD5_FIELD_NAME]   = msg_info.md5sum;
    header[TYPE_FIELD_NAME]  = msg_info.datatype;
    header[SEC_FIELD_NAME]   = std::string((char*)&time.sec, 4);
    header[NSEC_FIELD_NAME]  = std::string((char*)&time.nsec, 4);
    writeRecord(header, (char*)message_buf_, msg->__serialized_length);
    if (record_file_.fail())
    {
      ROS_FATAL("rosrecord::Record: could not write to file.  Check permissions and diskspace\n");
      return false;
    }
  }

  return true;
}

void ros::record::Recorder::writeFileHeader()
{
  boost::mutex::scoped_lock lock(record_mutex_);

  // Remember position of file header record
  file_header_pos_ = record_pos_;

  // Write file header record
  M_string header;
  header[OP_FIELD_NAME]        = std::string((char*)&OP_FILE_HEADER, 1);
  header[INDEX_POS_FIELD_NAME] = std::string((char*)&index_data_pos_, 8);

  boost::shared_array<uint8_t> header_buffer;
  uint32_t header_len;
  Header::write(header, header_buffer, header_len);
  uint32_t data_len = 0;
  if (header_len < FILE_HEADER_LENGTH)
    data_len = FILE_HEADER_LENGTH - header_len;
  write((char*)&header_len, 4);
  write((char*)header_buffer.get(), header_len);
  write((char*)&data_len, 4);

  // Pad the file header record out
  if (data_len > 0)
  {
    std::string padding;
    padding.resize(data_len, ' ');
    write(padding);
  }
}

void ros::record::Recorder::writeIndex()
{
	{
		boost::mutex::scoped_lock lock(record_mutex_);

		// Remember position of first index record
		index_data_pos_ = record_pos_;

		for (std::map<std::string, std::vector<IndexEntry> >::const_iterator i = topic_indexes_.begin(); i != topic_indexes_.end(); i++)
		{
			const std::string&             topic_name  = i->first;
			const std::vector<IndexEntry>& topic_index = i->second;

			uint32_t topic_index_size = topic_index.size();

	    const MsgInfo& msg_info = topics_recorded_[topic_name];

	    // Write the index record header
			M_string header;
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
				write((char*)&index_entry.sec,  4);
				write((char*)&index_entry.nsec, 4);
				write((char*)&index_entry.pos,  8);
			}
		}
	}

	// Re-open the file for random access, and rewrite the file header to point to the first index data message
  closeFile();
  openFile(file_name_, true);
  seek(file_header_pos_);
  writeFileHeader();
}

//

void ros::record::Recorder::writeRecord(const M_string& fields, const char* data, uint32_t data_len)
{
  writeHeader(fields, data_len);
  write(data, data_len);
}

void ros::record::Recorder::writeHeader(const M_string& fields, uint32_t data_len)
{
  boost::shared_array<uint8_t> header_buffer;
  uint32_t header_len;
  Header::write(fields, header_buffer, header_len);

  write((char*)&header_len, 4);
  write((char*)header_buffer.get(), header_len);
  write((char*)&data_len, 4);
}

void ros::record::Recorder::write(const char* s, std::streamsize n)
{
  record_stream_.write(s, n);
  record_pos_ += n;
}

void ros::record::Recorder::write(const std::string& s)
{
  write(s.c_str(), s.length());
}

void ros::record::Recorder::seek(pos_t pos)
{
  record_file_.seekp(pos, std::ios_base::beg);
  record_pos_ = pos;
}
