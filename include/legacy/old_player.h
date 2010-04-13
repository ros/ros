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

#ifndef ROSBAG_PLAYER_H
#define ROSBAG_PLAYER_H

#include <ros/ros.h>
#include <ros/header.h>
#include <ros/time.h>

#include "rosbag/any_msg.h"
#include "rosbag/msg_functor.h"
#include "rosbag/constants.h"

#include <fstream>
#include <sstream>
#include <cstdio>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>

#include <iostream>

namespace rosbag
{

class Player
{
    struct FilteredMsgFunctor
    {
        std::string         topic_name;
        std::string         md5sum;
        std::string         datatype;
        bool                inflate;
        AbstractMsgFunctor* f;
    };
    
    class PlayerHelper : public ros::Message
    {
    public:
        PlayerHelper(Player* player, std::string topic_name, std::string md5sum, std::string datatype, std::string message_definition);
        virtual ~PlayerHelper();
        
        void callHandlers();
        
        std::string get_topic_name();
        
        virtual const std::string __getDataType()          const;
        virtual const std::string __getMD5Sum()            const;
        virtual const std::string __getMessageDefinition() const;
        
        virtual uint8_t* deserialize(uint8_t* read_ptr);
        virtual uint32_t serializationLength() const;
        virtual uint8_t* serialize(uint8_t *write_ptr, uint32_t) const;
        
    private:
        Player* player_;
        
        std::string topic_name_;
        std::string md5sum_;
        std::string datatype_;
        std::string message_definition_;
        
        ros::Message* msg_;
        uint8_t* next_msg_;
        uint32_t next_msg_size_;
    };

public:
    Player(double time_scale=1);
    virtual ~Player();
    
    std::string   getVersionString();
    ros::Duration getFirstDuration();
    
    bool isDone();
    void close();
    
    bool open(const std::string &file_name, ros::Time start_time, bool try_future=false);

    template<class M>
    void addHandler(const std::string& topic_name, void (*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr, bool inflate);

    //! Handler for explicit message type (must be inflated)
    template<class M>
    void addHandler(const std::string& topic_name, void (*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr);
    
    template<class M, class T>
    void addHandler(const std::string& topic_name, void (T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr, bool inflate);

    //! Handler for explicit message type (must be inflated)
    template<class M, class T>
    void addHandler(const std::string& topic_name, void (T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr);

    ros::Time     get_next_msg_time();
    ros::Duration get_duration() const;
    
    bool nextMsg();
    void shiftTime(ros::Duration shift);

protected:
    // Helper to check for the presence of a field in a map of fields.
    // Checks min and max lengths, and prints errors.
    //
    // Returns field.end() if all checks pass, valid iterator pointing to the
    // field in question otherwise.
    ros::M_string::const_iterator checkField(const ros::M_string& fields, const std::string& field, unsigned int min_len, unsigned int max_len, bool required);
    
    // Parse a Version 1.2 header, which is a sequence of
    // <name>=<value_len><value> fields.
    //
    // Writes the value of the 'op' field into op; if op is OP_MSG_DATA, then
    // next_msg_dur gets filled in with the timestamp.
    //
    // Returns true on success, false otherwise.  On success, everything up
    // through the data_len field has been read, leaving just the serialized
    // message body in the file.
    bool parseVersion102Header(unsigned char& op, ros::Duration& next_msg_dur);

    bool readNextMsg();

private:
    std::ifstream                       record_file_;
    boost::iostreams::filtering_istream record_stream_;

    int version_;
    int version_major_;
    int version_minor_;
    
    double time_scale_;
    
    ros::Time start_time_;
    
    std::map<std::string, PlayerHelper*> topics_;
    
    std::string next_msg_name_;
    
    ros::Time next_msg_time_;
    ros::Time next_msg_time_recorded_;
    
    bool done_;
    
    ros::Duration first_duration_, duration_;
    
    unsigned char* header_buffer_;
    unsigned int   header_buffer_size_;
    
protected:
    uint8_t* next_msg_;
    uint32_t next_msg_size_;
    uint32_t next_msg_alloc_size_;
    
    // Putting these here feels very wrong
    std::string next_msg_latching_;
    std::string next_msg_callerid_;
    
    std::vector<FilteredMsgFunctor> callbacks_;
};

// Templated method implementation

template<class M>
void Player::addHandler(const std::string& topic_name, void (*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr, bool inflate) {
    FilteredMsgFunctor fmf;
    fmf.topic_name = topic_name;
    fmf.md5sum   = M::__s_getMD5Sum();
    fmf.datatype = M::__s_getDataType();
    fmf.inflate  = inflate;
    fmf.f        = new MsgFunctor<M>(fp, ptr, inflate);
        
    callbacks_.push_back(fmf);
 }

template<class M>
void Player::addHandler(const std::string& topic_name, void (*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr) {
    FilteredMsgFunctor fmf;       
    fmf.topic_name = topic_name;
    fmf.md5sum   = M::__s_getMD5Sum();
    fmf.datatype = M::__s_getDataType();
    fmf.inflate = true;
    fmf.f = new MsgFunctor<M>(fp, ptr);
    
    callbacks_.push_back(fmf);
}
    
template<class M, class T>
void Player::addHandler(const std::string& topic_name, void (T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr, bool inflate) {
    FilteredMsgFunctor fmf;
    fmf.topic_name = topic_name;
    fmf.md5sum     = M::__s_getMD5Sum();
    fmf.datatype   = M::__s_getDataType();
    fmf.inflate    = inflate;
    fmf.f          = new MsgFunctor<M, T>(obj, fp, ptr, inflate);
        
    callbacks_.push_back(fmf);
}

template<class M, class T>
void Player::addHandler(const std::string& topic_name, void (T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr) {
    FilteredMsgFunctor fmf;
    fmf.topic_name = topic_name;
    fmf.md5sum     = M::__s_getMD5Sum();
    fmf.datatype   = M::__s_getDataType();
    fmf.inflate    = true;
    fmf.f          = new MsgFunctor<M, T>(obj, fp, ptr);
    
    callbacks_.push_back(fmf);
}

}

#endif
