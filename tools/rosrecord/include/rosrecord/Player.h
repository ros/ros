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

#include "rosrecord/AnyMsg.h"
#include "rosrecord/MsgFunctor.h"
#include "rosrecord/constants.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/time_translator.h"

#include "topic_tools/shape_shifter.h"

#include "std_msgs/String.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

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

public:
    Player(double time_scale=1.0) //time_scale_(time_scale)
    {
        translator_.setTimeScale(time_scale);
    }

    virtual ~Player() {
        foreach(FilteredMsgFunctor const& callback, callbacks_)
            delete callback.f;

        close();
    }

    std::string getVersionString() {
        std::stringstream ss;
        ss << bag_.getMajorVersion() << "." << bag_.getMinorVersion();
        return ss.str();
    }

    bool          isDone()           { return iterator_ == view_.end(); }

    // todo: implement
    ros::Duration getFirstDuration() { return first_duration_;  }

    ros::Time getFirstTime() { return first_time_;  }

    void setStartTime(const ros::Time& start_time)
    {
        translator_.setTranslatedStartTime(start_time);
    }

    // todo: implement
    ros::Duration get_duration()     { return duration_;  }

    void close() {
        iterator_ = view_.end();

        bag_.close();
    }

    bool open(std::string const& file_name, ros::Time start_time, bool try_future = false) {
        translator_.setTranslatedStartTime(start_time);

        std::string ext = boost::filesystem::extension(file_name);
        if (ext != ".bag") {
            ROS_ERROR("File: '%s' does not have .bag extension",file_name.c_str());
            return false;
        }

        try
        {
            bag_.open(file_name, rosbag::bagmode::Read);
        }
        catch (rosbag::BagException ex)
        {
            ROS_FATAL_STREAM("Failed to open file: " << file_name << " (" << ex.what() << ")");
            return false;
        }

        view_.addQuery(bag_);
        iterator_ = view_.begin();

        if (iterator_ != view_.end())
        {
            translator_.setRealStartTime(iterator_->getTime());

            first_time_ = iterator_->getTime();
            first_duration_ = iterator_->getTime() - ros::Time(0,0);
            duration_ = iterator_->getTime() - first_time_;
        }

        return true;
    }

    template<class M>
    void addHandler(std::string topic_name, void(*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr, bool inflate)
    {
        FilteredMsgFunctor fmf;
        fmf.topic_name = topic_name;
        fmf.md5sum = M::__s_getMD5Sum();
        fmf.datatype = M::__s_getDataType();
        fmf.inflate = inflate;
        fmf.f = new MsgFunctor<M>(fp, ptr, inflate);

        callbacks_.push_back(fmf);
    }

    // Handler for explicit message type MUST be inflated (Note function pointer takes M vs. ros::Message)
    template<class M>
    void addHandler(std::string topic_name, void(*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr)
    {
        FilteredMsgFunctor fmf;
        fmf.topic_name = topic_name;
        fmf.md5sum = M::__s_getMD5Sum();
        fmf.datatype = M::__s_getDataType();
        fmf.inflate = true;
        fmf.f = new MsgFunctor<M>(fp, ptr);

        callbacks_.push_back(fmf);
    }

    template<class M, class T>
    void addHandler(std::string topic_name, void(T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr, bool inflate)
    {
        FilteredMsgFunctor fmf;
        fmf.topic_name = topic_name;
        fmf.md5sum = M::__s_getMD5Sum();
        fmf.datatype = M::__s_getDataType();
        fmf.inflate = inflate;
        fmf.f = new MsgFunctor<M, T>(obj, fp, ptr, inflate);

        callbacks_.push_back(fmf);
    }

    // Handler for explicit message type MUST be inflated (Note function pointer takes M vs. ros::Message)
    template<class M, class T>
    void addHandler(std::string topic_name, void(T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr)
    {
        FilteredMsgFunctor fmf;
        fmf.topic_name = topic_name;
        fmf.md5sum = M::__s_getMD5Sum();
        fmf.datatype = M::__s_getDataType();
        fmf.inflate = true;
        fmf.f = new MsgFunctor<M, T>(obj, fp, ptr);

        callbacks_.push_back(fmf);
    }

    ros::Time get_next_msg_time() {
        if (isDone())
            return ros::Time();

        return (*iterator_).getTime();
    }

    bool nextMsg() {
        if (iterator_ == view_.end())
            return false;

        rosbag::MessageInstance msg = (*iterator_);

        std::string const& topic    = msg.getTopic();
        std::string const& md5sum   = msg.getMD5Sum();
        std::string const& datatype = msg.getDataType();

        duration_ = msg.getTime() - first_time_;

        // Filter the list of callbacks
        std::vector<FilteredMsgFunctor> callbacks;
        foreach(FilteredMsgFunctor fmf, callbacks_)
        {
            if (topic != fmf.topic_name && fmf.topic_name != std::string("*"))
                continue;
            if (fmf.md5sum != md5sum && fmf.md5sum != std::string("*"))
            	continue;
            if (fmf.datatype != datatype && fmf.datatype != std::string("*") && datatype != std::string("*"))
            	continue;

            callbacks.push_back(fmf);
        }

        if (callbacks.size() > 0) {
        	boost::shared_ptr<topic_tools::ShapeShifter const> ss = msg.instantiate<topic_tools::ShapeShifter>();

        	ros::Time const& time = msg.getTime();
                ros::Time const& translated_time = translator_.translate(time);

        	foreach(FilteredMsgFunctor fmf, callbacks)
        		fmf.f->call(topic, (ros::Message*) ss.get(), translated_time, time);
        }

        iterator_++;

        return true;
    }

    void shiftTime(ros::Duration shift) {
        translator_.shift(shift);
    }

private:

    //double    time_scale_;
    //ros::Time start_time_;

    rosbag::TimeTranslator translator_;
    ros::Time first_time_;
    ros::Duration first_duration_;
    ros::Duration duration_;

    rosbag::Bag bag_;
    rosbag::View view_;
    rosbag::View::const_iterator iterator_;

    std::vector<FilteredMsgFunctor> callbacks_;
};

class MultiPlayer
{
public:
    MultiPlayer() { }

    ~MultiPlayer() {
        foreach(Player* player, players_)
            delete player;
    }

    ros::Duration getDuration()
    {
        ros::Duration d(0.0);
        foreach(Player* player, players_) {
            ros::Duration dd = player->get_duration();
            if (dd > d)
                d = dd;
        }
        return d;
    }

    bool open(std::vector<std::string> file_names, ros::Time start, double time_scale = 1, bool try_future = false)
    {
        ros::Duration first_duration;
        ros::Time first_time;

        foreach(std::string file_name, file_names) {
            Player* l = new Player(time_scale);
            if (l->open(file_name, ros::Time(), try_future)) {
                players_.push_back(l);

                // We don't actually use first_duration
                if (first_duration == ros::Duration() || l->getFirstDuration() < first_duration)
                    first_duration = l->getFirstDuration();

                if (first_time == ros::Time() || l->getFirstTime() < first_time)
                    first_time = l->getFirstTime();
            }
            else {
                delete l;
                return false;
            }
        }

        foreach(Player* player, players_)
            player->setStartTime(first_time);

        return true;
    }

    void shiftTime(ros::Duration shift)
    {
        for (std::vector<Player*>::iterator player_it = players_.begin(); player_it != players_.end(); player_it++)
            (*player_it)->shiftTime(shift);
    }

    template<class M>
    void addHandler(std::string topic_name, void(*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr, bool inflate)
    {
        for (std::vector<Player*>::iterator player_it = players_.begin(); player_it != players_.end(); player_it++)
            (*player_it)->addHandler<M>(topic_name, fp, ptr, inflate);
    }

    template<class M>
    void addHandler(std::string topic_name, void(*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr)
    {
        for (std::vector<Player*>::iterator player_it = players_.begin(); player_it != players_.end(); player_it++)
            (*player_it)->addHandler<M>(topic_name, fp, ptr);
    }

    template<class M, class T>
    void addHandler(std::string topic_name, void(T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr, bool inflate)
    {
        for (std::vector<Player*>::iterator player_it = players_.begin(); player_it != players_.end(); player_it++)
            (*player_it)->addHandler<M>(topic_name, fp, obj, ptr, inflate);
    }

    template<class M, class T>
    void addHandler(std::string topic_name, void(T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr)
    {
        for (std::vector<Player*>::iterator player_it = players_.begin(); player_it != players_.end(); player_it++)
            (*player_it)->addHandler<M>(topic_name, fp, obj, ptr);
    }

    bool nextMsg()
    {
        Player* next_player = NULL;

        bool first = true;
        ros::Time min_t = ros::Time(); // This should be the maximum unsigned int

        bool remaining = false;

        foreach(Player* player, players_) {
            if (player->isDone())
                continue;

            remaining = true;
            ros::Time t = player->get_next_msg_time();
            if (first || t < min_t) {
                first = false;
                next_player = player;
                min_t = player->get_next_msg_time();
            }
        }
        if (next_player)
            next_player->nextMsg();

        return remaining;
    }

private:
    std::vector<Player*> players_;
};

}

}

#endif
