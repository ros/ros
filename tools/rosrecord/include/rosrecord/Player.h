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

#ifndef ROSRECORDPLAYER_H
#define ROSRECORDPLAYER_H

#include "ros/node.h"
#include "ros/time.h"
#include "rosrecord/AnyMsg.h"
#include "rosrecord/MsgFunctor.h"
#include <fstream>
#include <sstream>
#include <cstdio>

namespace ros
{
namespace record
{

class Player
{
  struct FilteredMsgFunctor
  {
    std::string topic_name;
    std::string md5sum;
    std::string datatype;
    bool inflate;
    AbstractMsgFunctor* f;
  };


  class PlayerHelper : public ros::Message
  {
    Player* player_;

    std::string topic_name_;
    std::string md5sum_;
    std::string datatype_;

    ros::Message* msg_;
    uint8_t* next_msg_;
    uint32_t next_msg_size_;

  public:

    PlayerHelper(Player* player, std::string topic_name, std::string md5sum, std::string datatype)
      : player_(player), topic_name_(topic_name),  md5sum_(md5sum), datatype_(datatype),
        msg_(NULL), next_msg_(NULL), next_msg_size_(0) {}

    virtual ~PlayerHelper()
    {
      if (msg_)
        delete msg_;
    }

    void callHandlers()
    {

      next_msg_ = player_->next_msg_;
      next_msg_size_ = player_->next_msg_size_;

      __serialized_length = next_msg_size_;

      if (msg_)
      {
        msg_->__serialized_length = next_msg_size_;
        msg_->deserialize(next_msg_);
      }

      for (std::vector<FilteredMsgFunctor>::iterator fmf_it = player_->callbacks_.begin();
           fmf_it != player_->callbacks_.end();
           fmf_it++)
      {

        if (topic_name_  == fmf_it->topic_name ||
            fmf_it->topic_name == std::string("*"))
        {

          if (fmf_it->md5sum != md5sum_ &&
              fmf_it->md5sum != std::string("*"))
            break;

          if (fmf_it->datatype != datatype_ &&
              fmf_it->datatype != std::string("*") &&
              datatype_ != std::string("*"))
            break;

          if (fmf_it->inflate && msg_ == NULL)
          {
            msg_ = fmf_it->f->allocateMsg();

            if (msg_)
            {
              msg_->__serialized_length = next_msg_size_;
              msg_->deserialize(next_msg_);
            }
          }

          if (fmf_it->inflate)
            fmf_it->f->call(topic_name_, msg_, player_->next_msg_time_,  player_->next_msg_time_recorded_);
          else
            fmf_it->f->call(topic_name_, this, player_->next_msg_time_,  player_->next_msg_time_recorded_);
        }
      }
    }

    std::string get_topic_name() {return topic_name_;}

    virtual const std::string __getDataType() const { return datatype_; }

    virtual const std::string __getMD5Sum()   const { return md5sum_; }

    virtual uint8_t *deserialize(uint8_t *read_ptr) { assert(0); return NULL; }

    virtual uint32_t serializationLength() const   { return __serialized_length; }

    virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t) const
    {
      assert(next_msg_);
      memcpy(write_ptr, next_msg_, next_msg_size_);
      return write_ptr + next_msg_size_;
    }
  }; // class PlayerHelper




  std::ifstream record_file_;

  int version_;

  double time_scale_;

  ros::Time start_time_;

  std::map<std::string, PlayerHelper*> topics_;

  std::string next_msg_name_;

  ros::Time next_msg_time_, next_msg_time_recorded_;

  bool done_;

  ros::Duration first_duration_, duration_;


public:

 Player(double time_scale=1) : version_(0),  time_scale_(time_scale),  done_(false), first_duration_(0,0), duration_(0,0), next_msg_(NULL), next_msg_size_(0), next_msg_alloc_size_(0) {}

  virtual ~Player()
  {
    for (std::vector<FilteredMsgFunctor>::iterator fmf_it = callbacks_.begin();
         fmf_it != callbacks_.end();
         fmf_it++)
      if (fmf_it->f)
        delete (fmf_it->f);

    close();
  }

  ros::Duration getFirstDuration() { return first_duration_; }

  bool isDone() {return done_;}

  void close() {
    record_file_.close();

    for (std::map<std::string, PlayerHelper*>::iterator topic_it = topics_.begin();
         topic_it != topics_.end();
         topic_it++)
    {
      if (topic_it->second)
        delete topic_it->second;
    }

    topics_.clear();
    done_ = false;
  }

  bool open(const std::string &file_name, ros::Time start_time)
  {
    start_time_ = start_time;

    record_file_.open(file_name.c_str());

    if (record_file_.fail())
    {
      std::cerr << "Failed to open file: " << file_name <<  std::endl;
      return false;
    }

    char logtypename[100];
    int version_major = 0;
    int version_minor = 0;

    std::string version_line;
    getline(record_file_, version_line);

    sscanf(version_line.c_str(), "#ROS%s V%d.%d", logtypename, &version_major, &version_minor);

    if (version_major == 0 && version_line[0] == '#')
    {
      version_major = 1;
    }

    version_ = version_major * 100 + version_minor;

    int quantity = 0;

    if (version_ == 0)
    {
      record_file_.seekg(0, std::ios_base::beg);

      quantity = 1;

    }
    else if (version_ == 100)
    {
      std::string quantity_line;
      getline(record_file_,quantity_line);
      sscanf(quantity_line.c_str(), "%d", &quantity);
    }
    else if (version_ == 101)
    {

    }

    if (version_ == 0 || version_ == 100)
    {

      std::string topic_name;
      std::string md5sum;
      std::string datatype;

      for (int i = 0; i < quantity; i++)
      {
        getline(record_file_,topic_name);
        getline(record_file_,md5sum);
        getline(record_file_,datatype);

        // support type remapping of these core datatypes. I don't want
        // to match rostools/* as rostools will be a package again in
        // the future. We should do an audit of bags so this code does
        // not stay here forever
        if(datatype == "rostools/Time")
          datatype = "roslib/Time";
        else if (datatype == "rostools/Log")
          datatype = "roslib/Log";

        PlayerHelper* l = new PlayerHelper(this, topic_name, md5sum, datatype);

        topics_[topic_name] = l;
      }
    }

    readNextMsg();

    return true;
  }

  template <class M>
    void addHandler(std::string topic_name, void (*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr, bool inflate)
  {
    FilteredMsgFunctor fmf;

    fmf.topic_name = topic_name;
    fmf.md5sum   = M::__s_getMD5Sum();
    fmf.datatype = M::__s_getDataType();
    fmf.inflate = inflate;
    fmf.f = new MsgFunctor<M>(fp, ptr, inflate);

    callbacks_.push_back(fmf);
  }

  // Handler for explicit message type MUST be inflated (Note function pointer takes M vs. ros::Message)
  template <class M>
    void addHandler(std::string topic_name, void (*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr)
  {
    FilteredMsgFunctor fmf;

    fmf.topic_name = topic_name;
    fmf.md5sum   = M::__s_getMD5Sum();
    fmf.datatype = M::__s_getDataType();
    fmf.inflate = true;
    fmf.f = new MsgFunctor<M>(fp, ptr);

    callbacks_.push_back(fmf);
  }

  template <class M, class T>
    void addHandler(std::string topic_name, void (T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr, bool inflate)
  {
    FilteredMsgFunctor fmf;

    fmf.topic_name = topic_name;
    fmf.md5sum   = M::__s_getMD5Sum();
    fmf.datatype = M::__s_getDataType();
    fmf.inflate = inflate;
    fmf.f = new MsgFunctor<M, T>(obj, fp, ptr, inflate);

    callbacks_.push_back(fmf);
  }

  // Handler for explicit message type MUST be inflated (Note function pointer takes M vs. ros::Message)
  template <class M, class T>
    void addHandler(std::string topic_name, void (T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr)
  {
    FilteredMsgFunctor fmf;

    fmf.topic_name = topic_name;
    fmf.md5sum   = M::__s_getMD5Sum();
    fmf.datatype = M::__s_getDataType();
    fmf.inflate = true;
    fmf.f = new MsgFunctor<M, T>(obj, fp, ptr);

    callbacks_.push_back(fmf);
  }

  ros::Time get_next_msg_time()
  {
    if (!done_)
      return next_msg_time_;
    else
      return ros::Time();
  }

  ros::Duration get_duration()
  {
    return duration_;
  }

  bool nextMsg()
  {
    if (done_)
      return false;

    if (topics_.find(next_msg_name_) != topics_.end())
    {
      topics_[next_msg_name_]->callHandlers();
    }

    readNextMsg();

    return true;
  }

  void shiftTime(ros::Duration shift)
  {
    start_time_ = start_time_ + shift;
    next_msg_time_ = next_msg_time_ + shift;
  }

protected:

  uint8_t *next_msg_;
  uint32_t next_msg_size_, next_msg_alloc_size_;

  std::vector<FilteredMsgFunctor> callbacks_;

  bool readNextMsg()
  {
    if (!record_file_.good())
    {
      done_ = true;
      return false;
    }

    if (version_ <= 100)
    {
      if (version_ == 0)
        next_msg_name_ = (topics_.begin())->first;
      else
        getline(record_file_, next_msg_name_);

      ros::Duration next_msg_dur;


    } else {

      std::string topic_name;
      std::string md5sum;
      std::string datatype;

      getline(record_file_,topic_name);
      getline(record_file_,md5sum);
      getline(record_file_,datatype);

      // support type remapping of these core datatypes. I don't want
      // to match rostools/* as rostools will be a package again in
      // the future. We should do an audit of bags so this code does
      // not stay here forever
      if(datatype == "rostools/Time")
        datatype = "roslib/Time";
      else if (datatype == "rostools/Log")
        datatype = "roslib/Log";

      next_msg_name_ = topic_name;

      if (topics_.find(topic_name) == topics_.end())
      {
        PlayerHelper* l = new PlayerHelper(this, topic_name, md5sum, datatype);
        topics_[topic_name] = l;
      }

    }

    if (record_file_.eof())
    {
      done_ = true;
      return false;
    }

    ros::Duration next_msg_dur;

    record_file_.read((char*)&next_msg_dur.sec, 4);
    record_file_.read((char*)&next_msg_dur.nsec, 4);
    record_file_.read((char*)&next_msg_size_, 4);

    if (record_file_.eof())
    {
      done_ = true;
      return false;
    }

    if(first_duration_ == ros::Duration(0,0))
      first_duration_ = next_msg_dur;

    if (first_duration_ > next_msg_dur)
    {
      ROS_WARN("Messages in bag were not saved in chronological order %f > %f\n", first_duration_.toSec(), next_msg_dur.toSec());
      first_duration_ = next_msg_dur;
    }

    duration_ = next_msg_dur - first_duration_;

    next_msg_time_ = start_time_ + (duration_ * (1/time_scale_) );
    next_msg_time_recorded_.fromSec(next_msg_dur.toSec());


    if (next_msg_size_ > next_msg_alloc_size_)
    {
      if (next_msg_)
        delete[] next_msg_;
      next_msg_alloc_size_ = next_msg_size_ * 2;
      next_msg_ = new uint8_t[next_msg_alloc_size_];
    }

    record_file_.read((char*)next_msg_, next_msg_size_);

    if (record_file_.eof())
    {
      done_ = true;
      return false;
    }

    return true;

  }
};

class MultiPlayer
{
  std::vector<Player*> players_;

public:

  MultiPlayer() { }

  ~MultiPlayer()
  {
    for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      if (*player_it)
        delete (*player_it);
    }
  }

  ros::Duration getDuration()
  {
    ros::Duration d(0.0);
    for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      ros::Duration dd = (*player_it)->get_duration();
      if(dd > d)
        d = dd;
    }
    return d;
  }

  bool open(std::vector<std::string> file_names, ros::Time start, double time_scale=1)
  {

    ros::Duration first_duration;

    for (std::vector<std::string>::iterator name_it = file_names.begin();
         name_it != file_names.end();
         name_it++)
    {
      Player* l = new Player(time_scale);

      if (l->open(*name_it, start))
      {
        players_.push_back(l);

        if (first_duration == ros::Duration() || l->getFirstDuration() < first_duration)
          first_duration = l->getFirstDuration();
      }
      else
      {
        delete l;
        return false;
      }

      for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
      {
        (*player_it)->shiftTime( (*player_it)->getFirstDuration() - first_duration );
      }

    }

    return true;
  }

  bool nextMsg()
  {
    Player* next_player = 0;

    ros::Time min_t = ros::Time().fromNSec(ULLONG_MAX); // This should be the maximum unsigned int;

    bool remaining = false;

    for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      if ((*player_it)->isDone())
      {
        continue;
      }
      else
      {
        remaining = true;
        ros::Time t = (*player_it)->get_next_msg_time();
        if (t < min_t)
        {
          next_player = (*player_it);
          min_t = (*player_it)->get_next_msg_time();
        }
      }
    }

    if (next_player)
      next_player->nextMsg();

    return remaining;
  }

  void shiftTime(ros::Duration shift)
  {
    for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->shiftTime(shift);
    }
  }

  template <class M>
    void addHandler(std::string topic_name, void (*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr, bool inflate)
  {
    for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->addHandler<M>(topic_name, fp, ptr, inflate);
    }
  }

  template <class M>
    void addHandler(std::string topic_name, void (*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr)
  {
    for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->addHandler<M>(topic_name, fp, ptr);
    }
  }

  template <class M, class T>
    void addHandler(std::string topic_name, void (T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr, bool inflate)
  {
    for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->addHandler<M>(topic_name, fp, obj, ptr, inflate);
    }
  }

  template <class M, class T>
    void addHandler(std::string topic_name, void (T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr)
  {
    for (std::vector<Player*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->addHandler<M>(topic_name, fp, obj, ptr);
    }
  }
};

}
}

#endif

