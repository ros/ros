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

#ifndef ROSBAG_MULTI_PLAYER_H
#define ROSBAG_MULTI_PLAYER_H

#include <boost/foreach.hpp>

#include "rosbag/player.h"

namespace rosbag
{

class MultiPlayer
{
public:
    MultiPlayer();
    ~MultiPlayer();

    ros::Duration getDuration() const;

    bool open(const std::vector<std::string>& filenames, ros::Time start, double time_scale = 1, bool try_future = false);
    bool nextMsg();
    void shiftTime(ros::Duration shift);

    template<class M>
    void addHandler(const std::string& topic_name, void (*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr, bool inflate);

    template<class M>
    void addHandler(const std::string& topic_name, void (*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr);

    template<class M, class T>
    void addHandler(const std::string& topic_name, void (T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr, bool inflate);

    template<class M, class T>
    void addHandler(const std::string& topic_name, void (T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr);

private:
    std::vector<Player*> players_;
};

// Templated method definitions

template<class M>
void MultiPlayer::addHandler(const std::string& topic_name, void (*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), void* ptr, bool inflate) {
	BOOST_FOREACH(Player* p, players_)
        p->addHandler<M>(topic_name, fp, ptr, inflate);
}

template<class M>
void MultiPlayer::addHandler(const std::string& topic_name, void (*fp)(std::string, M*, ros::Time, ros::Time, void*), void* ptr) {
	BOOST_FOREACH(Player* p, players_)
        p->addHandler<M>(topic_name, fp, ptr);
}

template<class M, class T>
void MultiPlayer::addHandler(const std::string& topic_name, void (T::*fp)(std::string, ros::Message*, ros::Time, ros::Time, void*), T* obj, void* ptr, bool inflate) {
	BOOST_FOREACH(Player* p, players_)
        p->addHandler<M>(topic_name, fp, obj, ptr, inflate);
}

template<class M, class T>
void MultiPlayer::addHandler(const std::string& topic_name, void (T::*fp)(std::string, M*, ros::Time, ros::Time, void*), T* obj, void* ptr) {
	BOOST_FOREACH(Player* p, players_)
        p->addHandler<M>(topic_name, fp, obj, ptr);
}

}

#endif
