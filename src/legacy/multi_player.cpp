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

#include "rosbag/multi_player.h"
#include <boost/foreach.hpp>

using std::string;
using std::vector;
using ros::Duration;
using ros::Time;
using namespace rosbag;

MultiPlayer::MultiPlayer() { }

MultiPlayer::~MultiPlayer() {
    BOOST_FOREACH(Player* player, players_) {
        delete player;
    }
}

Duration MultiPlayer::getDuration() const {
    Duration max_d = ros::DURATION_MIN;

    BOOST_FOREACH(const Player* player, players_) {
        Duration d = player->get_duration();
        if (d > max_d)
            max_d = d;
    }

    return max_d;
}

bool MultiPlayer::open(const vector<string>& filenames, Time start, double time_scale, bool try_future) {
    // Open a player for each file
    BOOST_FOREACH(string filename, filenames) {
        Player* player = new Player(time_scale);
        if (!player->open(filename, start, try_future)) {
            delete player;
            return false;
        }

        players_.push_back(player);
    }

    // Get the shortest first duration
    Duration shortest_first_duration = ros::DURATION_MAX;
    BOOST_FOREACH(Player* player, players_) {
        Duration d = player->getFirstDuration();
        if (d < shortest_first_duration)
            shortest_first_duration = d;
    }

    // Shift time for each player
    BOOST_FOREACH(Player* player, players_) {
        player->shiftTime((player->getFirstDuration() - shortest_first_duration) * (1.0 / time_scale));
    }

    return true;
}

bool MultiPlayer::nextMsg() {
    Player* next_player = NULL;
    Time min_t = ros::TIME_MAX;

    BOOST_FOREACH(Player* player, players_) {
        if (!player->isDone()) {
            Time t = player->get_next_msg_time();
            if (!next_player || t < min_t) {
                next_player = player;
                min_t = t;
            }
        }
    }
    if (!next_player)
        return false;

    next_player->nextMsg();
    return true;
}

void MultiPlayer::shiftTime(Duration shift) {
    BOOST_FOREACH(Player* player, players_) {
        player->shiftTime(shift);
    }
}
