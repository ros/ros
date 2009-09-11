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
#include "rosrecord/MsgFunctor.h"
#include "rosrecord/constants.h"

#include <boost/thread/mutex.hpp>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>

#include <sys/statvfs.h>

namespace ros
{
namespace record
{

class Recorder
{
  private:

  typedef std::vector<std::pair<std::string, std::string> > TopicList;

  // Keep track of which topics have had their message definitions written
  // already
  std::map<std::string, bool> topic_def_written_;

  // Reusable buffer in which to assemble the message header before writing
  // to file.
  unsigned char* header_buf_;
  unsigned int header_buf_len_;
  unsigned int header_buf_size_;

  class RecorderHelper : public ros::Message
  {

    Recorder* recorder_;

    std::string topic_name_;
    std::string datatype_;
    std::string md5sum_;
    std::string message_definition_;          ///< \todo Fill this in

    AbstractMsgFunctor* callback_;

    ros::Message* msg_;

    unsigned int max_queue_;

    bool first_;

  public:

  RecorderHelper(Recorder* recorder, std::string topic_name, std::string datatype, std::string md5sum, unsigned int max_queue)
    : recorder_(recorder),
      topic_name_(topic_name), datatype_(datatype), md5sum_(md5sum),
      callback_(NULL),
      msg_(NULL),
      max_queue_(max_queue),
      first_(true)
    {}

    virtual ~RecorderHelper()
    {
      if (msg_)
        delete msg_;

      if (callback_)
        delete callback_;
    }

    void addHandler(AbstractMsgFunctor* callback)
    {
      if (msg_ == NULL)
        msg_ = callback->allocateMsg();
      else
        msg_ = this;

      callback_ = callback;
    }

    void callHandler()
    {
      if (callback_)
      {
        ros::Time now = ros::Time::now();
        callback_->call(topic_name_, msg_, now, now);
      }
    }

    std::string get_topic_name() {return topic_name_;}

    int get_max_queue() {return max_queue_;}

    virtual const std::string __getDataType() const { return datatype_; }
    virtual const std::string __getMD5Sum()   const { return md5sum_; }
    virtual const std::string __getMessageDefinition()   const { return message_definition_; }

     virtual uint32_t serializationLength() const   { return __serialized_length; }

    virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t) const { assert(0); return NULL; }

    virtual uint8_t *deserialize(uint8_t *read_ptr)
    {
      if (first_)
      {
        if (datatype_ == "*")
        {
          ros::M_string::iterator d = __connection_header->find(std::string("type"));
          if (d != __connection_header->end())
            datatype_ = d->second;
          else
          {
            ROS_FATAL("Connection header was not populated.\n");
            recorder_->node_->shutdown();
          }

          ros::M_string::iterator m = __connection_header->find(std::string("md5sum"));
          if (m != __connection_header->end())
            md5sum_ = m->second;
          else
          {
            ROS_FATAL("Connection header was not populated.\n");
            recorder_->node_->shutdown();
          }

          ros::M_string::iterator f = __connection_header->find(std::string("message_definition"));
          if (f != __connection_header->end())
            message_definition_ = f->second;
          else
          {
            ROS_WARN("Could not find message_definition in connection header.\n");
            recorder_->node_->shutdown();
          }
        }

        std::cout << "Recorder is saving topic: \"" << get_topic_name() << "\" with datatype: " << __getDataType() << std::endl;

        first_ = false;
      }

      uint8_t* sz = read_ptr + __serialized_length;

      sz = recorder_->record(__getMessageDefinition(),
                             get_topic_name(),
                             __getDataType(),
                             __getMD5Sum(),
                             read_ptr,
                             __serialized_length);

      if (msg_)
      {
        msg_->__serialized_length = __serialized_length;
        msg_->deserialize(read_ptr);
      }

      return sz;
    }
  };

  std::ofstream record_file_;

  boost::mutex record_mutex_;

  ros::Time start_time_;

  std::map<std::string, RecorderHelper*> topics_;

  bool started_;

  std::string file_name_;

  unsigned long long free_space_;

  ros::Time check_disk_next_;

  const void* getHeaderBuffer()
  {
    return header_buf_;
  }
  unsigned int getHeaderBufferLength()
  {
    return header_buf_len_;
  }
  void resetHeaderBuffer()
  {
    header_buf_len_ = 0;
  }
  void writeFieldToHeaderBuffer(const std::string& name,
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

public:

  Recorder(ros::Node* node) :
          started_(false),
          free_space_(0),
          node_(node) {}

  virtual ~Recorder()
  {
    for (std::map<std::string, RecorderHelper*>::iterator topic_it = topics_.begin();
         topic_it != topics_.end();
         topic_it++)
    {
      if (topic_it->second)
        delete topic_it->second;
    }
  }

  void close()
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

  bool open(const std::string &file_name, ros::Time start_time)
  {
    file_name_ = file_name;

    start_time_ = start_time;

    record_file_.open(file_name.c_str());
    if (record_file_.fail())
    {
      ROS_FATAL("rosrecord::Record: Failed to open file: %s", file_name.c_str());
      return false;
    }

    checkDisk();

    check_disk_next_ = ros::Time::now() + ros::Duration().fromSec(20.0);

    record_file_ << "#ROSRECORD V" << VERSION << std::endl;

    return true;
  }

  void checkDisk()
  {
    struct statvfs fiData;

    if((statvfs(file_name_.c_str(),&fiData)) < 0 ) {
      ROS_WARN("rosrecord::Record: Failed to check filesystem stats.");
    } else {
      free_space_ = (unsigned long long)(fiData.f_bsize) * (unsigned long long)(fiData.f_bavail);

      if (free_space_ < 1073741824ull) {
        ROS_ERROR("rosrecord::Record: Less than 1GB of space free on disk with %s.  Disabling logging.", file_name_.c_str());
        node_->shutdown();
      }
      else if (free_space_ < 5368709120ull) {
        ROS_WARN("rosrecord::Record: Less than 5GB of space free on disk with %s.", file_name_.c_str());
      }

    }
  }

  template <class M>
  RecorderHelper*  addTopic_(std::string topic_name, int max_queue)
  {
    if (topics_.find(topic_name) == topics_.end())
    {
      std::string datatype = M::__s_getDataType();
      std::string md5sum   = M::__s_getMD5Sum();

      RecorderHelper* l = new RecorderHelper(this, topic_name, datatype, md5sum, max_queue);

      topics_[topic_name] = l;

      if (started_)
        node_->subscribe(l->get_topic_name(), *l, &Recorder::recorderCb, this, l, l->get_max_queue());

      return l;
    } else {
      return NULL;
    }
  }

  template <class M>
  void addTopic(std::string topic_name, int max_queue)
  {
    addTopic_<M>(topic_name, max_queue);
  }

  template <class M>
    void addTopic(std::string topic_name, int max_queue, void (*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr = NULL, bool inflate = false)
  {
    RecorderHelper* l = addTopic_<M>(topic_name, max_queue);

    if (!l)
      return;

    if (fp != NULL)
    {
      AbstractMsgFunctor* lf = new MsgFunctor<M>(fp, ptr, inflate);
      l->addHandler(lf);
    }
  }

  // Handler for explicit message type must be inflated (Note function pointer takes M vs. ros::msg)
  template <class M>
    void addTopic(std::string topic_name, int max_queue, void (*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr = NULL)
  {
    RecorderHelper* l = addTopic_<M>(topic_name, max_queue);

    if (!l)
      return;

    if (fp != NULL)
    {
      AbstractMsgFunctor* lf = new MsgFunctor<M>(fp, ptr);
      l->addHandler(lf);
    }
  }

  template <class M, class T>
    void addTopic(std::string topic_name, int max_queue, void (T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr = NULL, bool inflate = false)
  {
    RecorderHelper* l = addTopic_<M>(topic_name, max_queue);

    if (!l)
      return;

    if (fp != NULL)
    {
      AbstractMsgFunctor* lf = new MsgFunctor<M,T>(obj, fp, ptr, inflate);
      l->addHandler(lf);
    }
  }

  // Handler for explicit message type MUST be inflated (Note function pointer takes M vs. ros::msg)
  template <class M, class T>
    void addTopic(std::string topic_name, int max_queue, void (T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr = NULL)
  {
    RecorderHelper* l = addTopic_<M>(topic_name, max_queue);

    if (!l)
      return;

    if (fp != NULL)
    {
      AbstractMsgFunctor* lf = new MsgFunctor<M,T>(obj, fp, ptr);
      l->addHandler(lf);
    }
  }

  void start() {
    for (std::map<std::string, RecorderHelper*>::iterator topic_it = topics_.begin();
         topic_it != topics_.end();
         topic_it++)
      node_->subscribe((topic_it->second)->get_topic_name(), *(topic_it->second), &Recorder::recorderCb, this, (topic_it->second), (topic_it->second)->get_max_queue());
    started_ = true;
  }

  void recorderCb(void* record_helper)
  {
    ((RecorderHelper*)(record_helper))->callHandler();
  }


protected:

  ros::Node* node_;

  uint8_t* record(const std::string& msg_def,
                  const std::string& topic_name,
                  const std::string& datatype,
                  const std::string& md5sum,
                  uint8_t *read_ptr, const uint32_t& length)
  {
    ros::Duration elapsed = ros::Time::now() - start_time_;

    if (ros::Time::now() > check_disk_next_)
    {
      checkDisk();
      check_disk_next_ = check_disk_next_ + ros::Duration().fromSec(20.0);
    }

    {
      boost::mutex::scoped_lock lock(record_mutex_);

      // Assemble the header in memory first, because we need to write its
      // length first.

      // Have we written the definition for this topic yet?
      if(topic_def_written_.find(topic_name) == topic_def_written_.end())
      {
        // Assemble a message definition header, which has no data, and
        // write it to file
        M_string m;
        m[OP_FIELD_NAME] = std::string((char*)&OP_MSG_DEF, 1);
        m[TOPIC_FIELD_NAME] = topic_name;
        m[MD5_FIELD_NAME] = md5sum;
        m[TYPE_FIELD_NAME] = datatype;
        m[DEF_FIELD_NAME] = msg_def;

        boost::shared_array<uint8_t> header_buffer;
        uint32_t header_len;
        Header::write(m, header_buffer, header_len);

        unsigned int data_len = 0;
        record_file_.write((char*)&header_len, 4);
        record_file_.write((char*)header_buffer.get(), header_len);
        record_file_.write((char*)&data_len, 4);

        topic_def_written_[topic_name] = true;
      }

      // Write a message data header, followed by the data
      M_string m;
      m[OP_FIELD_NAME] = std::string((char*)&OP_MSG_DATA, 1);
      m[TOPIC_FIELD_NAME] = topic_name;
      m[MD5_FIELD_NAME] = md5sum;
      m[TYPE_FIELD_NAME] = datatype;
      m[SEC_FIELD_NAME] = std::string((char*)&elapsed.sec, 4);
      m[NSEC_FIELD_NAME] = std::string((char*)&elapsed.nsec, 4);

      boost::shared_array<uint8_t> header_buffer;
      uint32_t header_len;
      Header::write(m, header_buffer, header_len);
      record_file_.write((char*)&header_len, 4);
      record_file_.write((char*)header_buffer.get(), header_len);

      record_file_.write((char*)&length, 4);
      record_file_.write((char*)read_ptr, length);

      if (record_file_.fail())
      {
        ROS_FATAL("rosrecord::Record: could not write to file.  Check permissions and diskspace\n");
        node_->shutdown();
      }
    }

    return read_ptr + length;
  }

};

}
}

#endif
