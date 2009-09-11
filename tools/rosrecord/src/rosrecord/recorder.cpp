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

#include "rosrecord/constants.h"
#include <iomanip>
#include <signal.h>
#include <sys/statvfs.h>

#include "rosrecord/Recorder.h"

using namespace ros::record;

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

void ros::record::Recorder::writeFieldToHeaderBuffer(const std::string& name,
                                        const void* value,
                                        unsigned int value_len)
{
  // Do a little buffer-size management.
  unsigned int new_len = header_buf_len_ + name.size() + 1 + 4 + value_len;
  if(header_buf_size_ < new_len)
  {
    if(header_buf_size_ == 0)
      header_buf_size_ = new_len;
    else
    {
      while(header_buf_size_ < new_len)
        header_buf_size_ *= 2;
    }
    header_buf_ = (unsigned char*)realloc(header_buf_, header_buf_size_);
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


void ros::record::Recorder::close()
{
  // Unfortuantely closing this possibly enormous file takes a while
  // (especially over NFS) and handling of a SIGINT while a file is
  // closing leads to a double free.  So, we disable signals while
  // we close the file.

  // Darwin doesn't have sighandler_t; I hope that sig_t on Linux does
  // the right thing.
  //sighandler_t old = signal(SIGINT, SIG_IGN);
  sig_t old = signal(SIGINT, SIG_IGN);
  if (record_file_.is_open())
    record_file_.close();
  signal(SIGINT, old);
}


bool ros::record::Recorder::open(const std::string &file_name)
{
  file_name_ = file_name;

  record_file_.open(file_name.c_str());
  if (record_file_.fail())
  {
    ROS_FATAL("rosrecord::Record: Failed to open file: %s", file_name.c_str());
    return false;
  }

  checkDisk();

  check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);

  record_file_ << "#ROSRECORD V" << VERSION << std::endl;

  return true;
}

bool ros::record::Recorder::checkDisk()
{
  struct statvfs fiData;

  if((statvfs(file_name_.c_str(),&fiData)) < 0 ) {
    ROS_WARN("rosrecord::Record: Failed to check filesystem stats.");
  } else {
    free_space_ = (unsigned long long)(fiData.f_bsize) * (unsigned long long)(fiData.f_bavail);

    if (free_space_ < 1073741824ull) {
      ROS_ERROR("rosrecord::Record: Less than 1GB of space free on disk with %s.  Disabling logging.", file_name_.c_str());
      return false;
    }
    else if (free_space_ < 5368709120ull) {
      ROS_WARN("rosrecord::Record: Less than 5GB of space free on disk with %s.", file_name_.c_str());
    }
  }
  return true;
}

bool ros::record::Recorder::record(std::string topic_name, ros::Message::ConstPtr msg, ros::Time time)
{

  bool needs_def_written = false;
  std::map<std::string, MsgInfo>::iterator key;

  {
    boost::mutex::scoped_lock lock(topics_recorded_mutex_);

    key = topics_recorded_.find(topic_name);

    if (key == topics_recorded_.end())
    {
      MsgInfo& info = topics_recorded_[topic_name];

      info.msg_def  = (*(msg->__connection_header))["message_definition"];
      info.datatype = (*(msg->__connection_header))["type"];
      info.md5sum   = (*(msg->__connection_header))["md5sum"];
      
      key = topics_recorded_.find(topic_name);

      needs_def_written = true;
    }
  }


  {
    boost::mutex::scoped_lock lock(check_disk_mutex_);

    if (ros::WallTime::now() > check_disk_next_)
    {
      if (!checkDisk())
        return false;
                     
      check_disk_next_ = check_disk_next_ + ros::WallDuration().fromSec(20.0);
    }
  }

  {
    boost::mutex::scoped_lock lock(record_mutex_);

    // Assemble the header in memory first, because we need to write its
    // length first.

    // Have we written the definition for this topic yet?
    if(needs_def_written)
    {
      // Assemble a message definition header, which has no data, and
      // write it to file
      M_string m;
      m[OP_FIELD_NAME] = std::string((char*)&OP_MSG_DEF, 1);
      m[TOPIC_FIELD_NAME] = topic_name;
      m[MD5_FIELD_NAME] = (key->second).md5sum;
      m[TYPE_FIELD_NAME] = (key->second).datatype;
      m[DEF_FIELD_NAME] = (key->second).msg_def;

      boost::shared_array<uint8_t> header_buffer;
      uint32_t header_len;
      Header::write(m, header_buffer, header_len);

      unsigned int data_len = 0;
      record_file_.write((char*)&header_len, 4);
      record_file_.write((char*)header_buffer.get(), header_len);
      record_file_.write((char*)&data_len, 4);
    }

    // Write a message data header, followed by the data
    M_string m;
    m[OP_FIELD_NAME] = std::string((char*)&OP_MSG_DATA, 1);
    m[TOPIC_FIELD_NAME] = topic_name;
    m[MD5_FIELD_NAME] = (key->second).md5sum;
    m[TYPE_FIELD_NAME] = (key->second).datatype;
    m[SEC_FIELD_NAME] = std::string((char*)&time.sec, 4);
    m[NSEC_FIELD_NAME] = std::string((char*)&time.nsec, 4);

    boost::shared_array<uint8_t> header_buffer;
    uint32_t header_len;
    Header::write(m, header_buffer, header_len);
    record_file_.write((char*)&header_len, 4);
    record_file_.write((char*)header_buffer.get(), header_len);

    if(message_buf_size_ < msg->__serialized_length)
    {
      if(message_buf_size_ == 0)
        message_buf_size_ = msg->__serialized_length;
      else
      {
        while(message_buf_size_ < msg->__serialized_length)
          message_buf_size_ *= 2;
      }
      message_buf_ = (unsigned char*)realloc(message_buf_, message_buf_size_);
      ROS_ASSERT(message_buf_);
    }
    msg->serialize(message_buf_, 0);

    record_file_.write((char*)&msg->__serialized_length, 4);
    record_file_.write((char*)message_buf_, msg->__serialized_length);

    if (record_file_.fail())
    {
      ROS_FATAL("rosrecord::Record: could not write to file.  Check permissions and diskspace\n");
      return false;
    }
  }

  return true;
}
