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

#include "rosbag/player.h"
#include <boost/foreach.hpp>

using std::map;
using std::string;
using std::vector;
using ros::Duration;
using ros::Time;

namespace rosbag {

Player::PlayerHelper::PlayerHelper(Player* player, string topic_name, string md5sum, string datatype, string message_definition) :
    player_(player),
    topic_name_(topic_name),
    md5sum_(md5sum),
    datatype_(datatype),
    message_definition_(message_definition),
    msg_(NULL),
    next_msg_(NULL),
    next_msg_size_(0)
{
    __connection_header = boost::shared_ptr<ros::M_string>(new ros::M_string);
    (*__connection_header)["type"]               = datatype;
    (*__connection_header)["md5sum"]             = md5sum;
    (*__connection_header)["message_definition"] = message_definition;
}

Player::PlayerHelper::~PlayerHelper() {
    delete msg_;
}

void Player::PlayerHelper::callHandlers() {
    next_msg_      = player_->next_msg_;
    next_msg_size_ = player_->next_msg_size_;
    (*__connection_header)["callerid"] = player_->next_msg_callerid_;
    (*__connection_header)["latching"] = player_->next_msg_latching_;
    
    __serialized_length = next_msg_size_;
    
    if (msg_) {
        msg_->__serialized_length = next_msg_size_;
        msg_->deserialize(next_msg_);
    }

    BOOST_FOREACH(const FilteredMsgFunctor& fmf, player_->callbacks_) {
        if (topic_name_ == fmf.topic_name || fmf.topic_name == string("*")) {
            if (fmf.md5sum != string("*") && fmf.md5sum != md5sum_)
                break;
            if (fmf.datatype != string("*") && datatype_ != string("*") && fmf.datatype != datatype_)
                break;

            // If necessary, deserialize the message
            if (fmf.inflate && msg_ == NULL) {
                msg_ = fmf.f->allocateMsg();                
                if (msg_) {
                    msg_->__serialized_length = next_msg_size_;
                    msg_->deserialize(next_msg_);
                }
            }

            if (fmf.inflate) {
                msg_->__connection_header = __connection_header;
                fmf.f->call(topic_name_, msg_, player_->next_msg_time_, player_->next_msg_time_recorded_);
            }
            else
                fmf.f->call(topic_name_, this, player_->next_msg_time_, player_->next_msg_time_recorded_);
        }
    }
}

string Player::PlayerHelper::get_topic_name() { return topic_name_; }

const string Player::PlayerHelper::__getDataType()          const { return datatype_;           }
const string Player::PlayerHelper::__getMD5Sum()            const { return md5sum_;             }
const string Player::PlayerHelper::__getMessageDefinition() const { return message_definition_; }

uint8_t* Player::PlayerHelper::deserialize(uint8_t* read_ptr) { assert(0); return NULL; }

uint32_t Player::PlayerHelper::serializationLength() const   { return __serialized_length; }

uint8_t* Player::PlayerHelper::serialize(uint8_t* write_ptr, uint32_t) const {
    assert(next_msg_);
    memcpy(write_ptr, next_msg_, next_msg_size_);
    return write_ptr + next_msg_size_;
}

//

Player::Player(double time_scale) :
    version_(0),
    version_major_(0),
    version_minor_(0),
    time_scale_(time_scale),
    done_(true),
    first_duration_(0, 0),
    duration_(0, 0),
    header_buffer_(NULL),
    header_buffer_size_(0),
    next_msg_(NULL),
    next_msg_size_(0),
    next_msg_alloc_size_(0)
{
}

Player::~Player() {
    BOOST_FOREACH(const FilteredMsgFunctor& fmf, callbacks_) {
        delete fmf.f;
    }

    close();

    if (header_buffer_)
        free(header_buffer_);
    delete[] next_msg_;
}

string Player::getVersionString() {
    std::stringstream ss;
    ss << version_major_ << "." << version_minor_;
    return ss.str();
}

Duration Player::getFirstDuration() { return first_duration_; }

bool Player::isDone() { return done_; }

void Player::close() {
    if (!record_stream_.empty())
        record_stream_.pop();
    record_file_.close();
    
    for (map<string, PlayerHelper*>::iterator i = topics_.begin(); i != topics_.end(); i++)
        delete i->second;

    topics_.clear();
    done_ = false;
}

bool Player::open(const string& file_name, Time start_time, bool try_future) {
    start_time_ = start_time;

    record_file_.open(file_name.c_str());
    if (record_file_.fail()) {
        ROS_FATAL_STREAM("Failed to open file: " << file_name);
        return false;
    }

    string ext = boost::filesystem::extension(file_name);
    if (ext != ".bag") {
        ROS_ERROR("File: '%s' does not have .bag extension",file_name.c_str());
        return false;
    }

    record_stream_.push(record_file_);

    char logtypename[100];

    string version_line;
    getline(record_stream_, version_line);

    sscanf(version_line.c_str(), "#ROS%s V%d.%d", logtypename, &version_major_, &version_minor_);

    if (version_major_ == 0 && version_line[0] == '#')
        version_major_ = 1;

    version_ = version_major_ * 100 + version_minor_;

    int quantity = 0;
    if (version_ == 0) {
        ROS_WARN("No #ROSRECORD header found.  Assuming a V0.0 bag file, but more likely a corrupt file, or not really a .bag at all");
        record_stream_.seekg(0, std::ios_base::beg);        
        quantity = 1;
    }
    else if (version_ == 100) {
        string quantity_line;
        getline(record_stream_, quantity_line);
        sscanf(quantity_line.c_str(), "%d", &quantity);
    }
    else if (version_ == 101) {
        // nop
    }

    if (version_ == 0 || version_ == 100) {
        string topic_name;
        string md5sum;
        string datatype;
        for (int i = 0; i < quantity; i++) {
            getline(record_stream_, topic_name);
            getline(record_stream_, md5sum);
            getline(record_stream_, datatype);
            
            // support type remapping of these core datatypes. I don't want
            // to match rostools/* as rostools will be a package again in
            // the future. We should do an audit of bags so this code does
            // not stay here forever
            if (datatype == "rostools/Time")
                datatype = "roslib/Time";
            else if (datatype == "rostools/Log")
                datatype = "roslib/Log";

            topics_[topic_name] = new PlayerHelper(this, topic_name, md5sum, datatype, string(""));
        }
    }

    int cur_version_major;
    int cur_version_minor;
    sscanf(VERSION.c_str(), "%d.%d", &cur_version_major, &cur_version_minor);

    if (!try_future && version_ > cur_version_major * 100 + cur_version_minor) {
        ROS_ERROR("'%s' has version %d.%d, but Reader only knows about versions up to %s.", file_name.c_str(), version_major_, version_minor_, VERSION.c_str());
        return false;
    }

    done_ = false;
    readNextMsg();

    return true;
}

Time Player::get_next_msg_time() {
    if (!done_)
        return next_msg_time_;
    else
        return Time();
}

Duration Player::get_duration() const { return duration_; }

bool Player::nextMsg() {
    if (done_)
        return false;
    
    if (topics_.find(next_msg_name_) != topics_.end())
        topics_[next_msg_name_]->callHandlers();
    
    readNextMsg();
    
    return true;
}

void Player::shiftTime(Duration shift) {
    start_time_ = start_time_ + shift;
    next_msg_time_ = next_msg_time_ + shift;
}

// Helper to check for the presence of a field in a map of fields.
// Checks min and max lengths, and prints errors.
//
// Returns field.end() if all checks pass, valid iterator pointing to the
// field in question otherwise.
ros::M_string::const_iterator Player::checkField(const ros::M_string& fields, const string& field, unsigned int min_len, unsigned int max_len, bool required) {
    ros::M_string::const_iterator fitr;
    fitr = fields.find(field);
    if (fitr == fields.end()) {
        if (required)
            ROS_ERROR("Required %s field missing", field.c_str());
    }
    else if ((fitr->second.size() < min_len) || (fitr->second.size() > max_len)) {
        ROS_ERROR("Field %s is wrong size (%u bytes)", field.c_str(), (uint32_t) fitr->second.size());
        return fields.end();
    }

    return fitr;
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
bool Player::parseVersion102Header(unsigned char& op, Duration& next_msg_dur) {
    unsigned int header_len;
    string topic_name;
    string md5sum;
    string datatype;
    string message_definition;
    string latching("0");
    string callerid("");

    // Read the header length
    record_stream_.read((char*)&header_len, 4);
    if (record_stream_.eof())
        return false;
    
    if (header_buffer_size_ < header_len) {
        header_buffer_size_ = header_len;
        header_buffer_ = (unsigned char*)realloc(header_buffer_, header_buffer_size_);
        ROS_ASSERT(header_buffer_);
    }

    // Read the header
    record_stream_.read((char*)header_buffer_, header_len);
    if (record_stream_.eof())
        return false;

    // Parse the header
    ros::Header header;
    string error_msg;
    bool parsed = header.parse(header_buffer_, header_len, error_msg);
    if (!parsed) {
        ROS_ERROR("Error parsing header: %s", error_msg.c_str());
        return false;
    }

    // Check for necessary fields, validating as we go.

    // Some fields are always required
    ros::M_string::const_iterator fitr;
    ros::M_stringPtr fields_ptr = header.getValues();
    ros::M_string& fields = *fields_ptr;

    if ((fitr = checkField(fields, OP_FIELD_NAME, 1, 1, true)) == fields.end())
        return false;

    memcpy(&op,fitr->second.data(),1);

    // Read the body length
    record_stream_.read((char*)&next_msg_size_, 4);
    if (record_stream_.eof())
        return false;

    // Extra checking on the value of op
    switch (op)
    {
    case OP_MSG_DATA:
        if ((fitr = checkField(fields, TOPIC_FIELD_NAME, 1, UINT_MAX, true)) == fields.end())
            return false;
        topic_name = fitr->second;

        if ((fitr = checkField(fields, MD5_FIELD_NAME, 32, 32, true)) == fields.end())
            return false;
        md5sum = fitr->second;

        if ((fitr = checkField(fields, TYPE_FIELD_NAME, 1, UINT_MAX, true)) == fields.end())
            return false;
        datatype = fitr->second;      

        if ((fitr = checkField(fields, SEC_FIELD_NAME, 4, 4, true)) == fields.end())
            return false;
        memcpy(&next_msg_dur.sec,fitr->second.data(),4);

        if ((fitr = checkField(fields, NSEC_FIELD_NAME, 4, 4, true)) == fields.end())
            return false;
        memcpy(&next_msg_dur.nsec,fitr->second.data(),4);

        // Latching and callerid fields are optional
        if ((fitr = checkField(fields, LATCHING_FIELD_NAME, 1, UINT_MAX, false)) != fields.end())
            latching = fitr->second;

        if ((fitr = checkField(fields, CALLERID_FIELD_NAME, 1, UINT_MAX, false)) != fields.end())
            callerid = fitr->second;

        next_msg_name_ = topic_name;

        // We always set to defaults specified at the beginning
        next_msg_latching_ = latching;
        next_msg_callerid_ = callerid;

        // If this is the first time that we've encountered this topic, we need
        // to create a PlayerHelper, which inherits from ros::Message and is
        // used to publish messages from this topic.
        if (topics_.find(topic_name) == topics_.end())
            topics_[topic_name] = new PlayerHelper(this, topic_name, md5sum, datatype, message_definition);
      
        return true;

    case OP_MSG_DEF:
        if ((fitr = checkField(fields, TOPIC_FIELD_NAME, 1, UINT_MAX, true)) == fields.end())
            return false;
        topic_name = fitr->second;

        if ((fitr = checkField(fields, MD5_FIELD_NAME, 32, 32, true)) == fields.end())
            return false;
        md5sum = fitr->second;

        if ((fitr = checkField(fields, TYPE_FIELD_NAME, 1, UINT_MAX, true)) == fields.end())
            return false;
        datatype = fitr->second;      

        // Note that the field length can be zero.  This can happen if a
        // publisher didn't supply the definition, e.g., this bag was created
        // by recording from the playback of a pre-1.2 bag.
        if ((fitr = checkField(fields, DEF_FIELD_NAME, 0, UINT_MAX, true)) == fields.end())
            return false;
        message_definition = fitr->second;

        // If this is the first time that we've encountered this topic, we need
        // to create a PlayerHelper, which inherits from ros::Message and is
        // used to publish messages from this topic.
        if (topics_.find(topic_name) == topics_.end())
            topics_[topic_name] = new PlayerHelper(this, topic_name, md5sum, datatype, message_definition);

        return true;

    case OP_FILE_HEADER:
        return true;

    case OP_INDEX_DATA:
        return true;

    default:
        ROS_ERROR("Field %s has invalid value %u\n", OP_FIELD_NAME.c_str(), op);
        return false;      
    }

    return false;
}

bool Player::readNextMsg() {
    if (!record_stream_.good()) {
        done_ = true;
        return false;
    }
    
    Duration next_msg_dur;

    if (version_ >= 102) {
        unsigned char op;
        if (!parseVersion102Header(op, next_msg_dur)) {
            done_ = true;
            return false;
        }

        // If it was just a definition, we return here, to avoid publishing
        // the zero-length body that follows
        if (op != OP_MSG_DATA) {
            // Just throw these bytes away for now.
            record_stream_.ignore(next_msg_size_);
            
            return readNextMsg();
        }
    }
    else {
        // Support for older versions
        
        if (version_ <= 100) {
            if (version_ == 0)
                next_msg_name_ = (topics_.begin())->first;
            else
                getline(record_stream_, next_msg_name_);
            
            Duration next_msg_dur;
        }
        else {
            string topic_name;
            string md5sum;
            string datatype;
            getline(record_stream_, topic_name);
            getline(record_stream_, md5sum);
            getline(record_stream_, datatype);
            
            // support type remapping of these core datatypes. I don't want
            // to match rostools/* as rostools will be a package again in
            // the future. We should do an audit of bags so this code does
            // not stay here forever
            if (datatype == "rostools/Time")
                datatype = "roslib/Time";
            else if (datatype == "rostools/Log")
                datatype = "roslib/Log";
            
            next_msg_name_ = topic_name;
            
            if (topics_.find(topic_name) == topics_.end())
                topics_[topic_name] = new PlayerHelper(this, topic_name, md5sum, datatype, string(""));
        }
        
        if (record_stream_.eof()) {
            done_ = true;
            return false;
        }
        
        record_stream_.read((char*)&next_msg_dur.sec, 4);
        record_stream_.read((char*)&next_msg_dur.nsec, 4);
        record_stream_.read((char*)&next_msg_size_, 4);
        if (record_stream_.eof()) {
            done_ = true;
            return false;
        }
    }
    
    // At this point, we've done all the version-specific work, and read up
    // through the specifier for the length of serialized message body.
    // Along the way, we filled in the following variables:
    //   next_msg_dur
    //   next_msg_size_
    //   next_msg_name_
    // Next we read the body and put it into next_msg_.

    if (first_duration_ == Duration(0, 0))
        first_duration_ = next_msg_dur;

    if (first_duration_ > next_msg_dur) {
        ROS_WARN("Messages in bag were not saved in chronological order %f > %f\n", first_duration_.toSec(), next_msg_dur.toSec());
        first_duration_ = next_msg_dur;
    }
    
    duration_ = next_msg_dur - first_duration_;
    
    next_msg_time_ = start_time_ + (duration_ * (1 / time_scale_));
    next_msg_time_recorded_.fromSec(next_msg_dur.toSec());
    
    if (next_msg_size_ > next_msg_alloc_size_) {
        if (next_msg_)
            delete[] next_msg_;

        next_msg_alloc_size_ = next_msg_size_ * 2;
        next_msg_ = new uint8_t[next_msg_alloc_size_];
    }

    // Read in the message body
    record_stream_.read((char*)next_msg_, next_msg_size_);

    if (record_stream_.eof()) {
        done_ = true;
        return false;
    }

    return true;
}

}
