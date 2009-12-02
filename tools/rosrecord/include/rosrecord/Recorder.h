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

struct MsgInfo
{
  std::string msg_def;
  std::string md5sum;
  std::string datatype;
};

class Recorder
{
  private:

  // Keep track of which topics have had their message definitions written
  // already
  std::map<std::string, MsgInfo> topics_recorded_;

  boost::mutex topics_recorded_mutex_;

  // Reusable buffer in which to assemble the message header before writing
  // to file.
  unsigned char* header_buf_;
  unsigned int header_buf_len_;
  unsigned int header_buf_size_;

  // Reusable buffer in which to assemble the message data before writing
  // to file.
  unsigned char* message_buf_;
  unsigned int message_buf_len_;
  unsigned int message_buf_size_;

  std::ofstream record_file_;
  boost::iostreams::filtering_ostream record_stream_;

  boost::mutex record_mutex_;

  std::string file_name_;

  unsigned long long free_space_;

  ros::WallTime check_disk_next_;

  boost::mutex check_disk_mutex_;

  const void* getHeaderBuffer();

  unsigned int getHeaderBufferLength();

  void resetHeaderBuffer();

  void writeFieldToHeaderBuffer(const std::string& name,
                                const void* value,
                                unsigned int value_len);

  bool checkDisk();

public:

  Recorder() : header_buf_(NULL), header_buf_len_(0), header_buf_size_(0), message_buf_(NULL), message_buf_len_(0), message_buf_size_(0), 
               free_space_(0) {}

  virtual ~Recorder()  { close(); }

  bool open(const std::string &file_name);

  void close();

  bool record(std::string topic_name, ros::Message::ConstPtr msg, ros::Time time);
};

}
}

#endif
