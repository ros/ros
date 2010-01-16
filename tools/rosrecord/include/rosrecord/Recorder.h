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

#ifndef ROSRECORDRECORDER_H
#define ROSRECORDRECORDER_H

#include "ros/header.h"
#include "ros/time.h"
#include "ros/message.h"

#include <boost/thread/mutex.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <fstream>
#include <iostream>
#include <map>

namespace ros
{
namespace record
{

typedef unsigned long long pos_t;

struct MsgInfo
{
  std::string msg_def;
  std::string md5sum;
  std::string datatype;
};

struct IndexEntry
{
  uint32_t sec;
  uint32_t nsec;
  pos_t    pos;
};

class Recorder
{
public:
  Recorder();
  virtual ~Recorder();

  bool open(const std::string& file_name, bool random_access=false);
  bool record(std::string topic_name, ros::Message::ConstPtr msg, ros::Time time);
  bool record(std::string topic_name, const ros::Message& msg, ros::Time time);
  void close();

  pos_t getOffset();

private:
  bool         openFile(const std::string& file_name, bool random_access=false);
  void         closeFile();

  const void*  getHeaderBuffer();
  unsigned int getHeaderBufferLength();
  void         resetHeaderBuffer();
  void         writeFieldToHeaderBuffer(const std::string& name, const void* value, unsigned int value_len);
  bool         checkDisk();

  void         writeVersion();
  void         writeFileHeader();
  void         writeIndex();

  void         writeRecord(const M_string& fields, const char* data, uint32_t data_len);
  void         writeHeader(const M_string& fields, uint32_t data_len);
  void         write(const char* s, std::streamsize n);
  void         write(const std::string& s);
  void         seek(pos_t pos);

private:
  std::string file_name_;

  std::ofstream                       record_file_;
  boost::iostreams::filtering_ostream record_stream_;
  pos_t                               record_pos_;
  bool                                store_index_;
  pos_t                               file_header_pos_;
  pos_t                               index_data_pos_;
  boost::mutex                        record_mutex_;

  // Keep track of which topics have had their message definitions written already
  std::map<std::string, MsgInfo>      topics_recorded_;
  boost::mutex                        topics_recorded_mutex_;

  std::map<std::string, std::vector<IndexEntry> > topic_indexes_;

  // Reusable buffer in which to assemble the message header before writing to file
  unsigned char* header_buf_;
  unsigned int   header_buf_len_;
  unsigned int   header_buf_size_;

  // Reusable buffer in which to assemble the message data before writing to file
  unsigned char* message_buf_;
  unsigned int   message_buf_len_;
  unsigned int   message_buf_size_;

  bool logging_enabled_;

  boost::mutex       check_disk_mutex_;
  ros::WallTime      check_disk_next_;
  ros::WallTime      warn_next_;
};

}
}

#endif
