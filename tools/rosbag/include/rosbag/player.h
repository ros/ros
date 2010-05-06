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

#ifndef ROSBAG_PLAYER_H
#define ROSBAG_PLAYER_H

#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <queue>
#include <string>

#include <ros/ros.h>
#include <ros/time.h>

#include "rosbag/bag.h"

#include "rosbag/time_translator.h"

namespace rosbag {

struct PlayerOptions
{
    PlayerOptions();

    void check();

    bool     quiet;
    bool     start_paused;
    bool     at_once;
    bool     bag_time;
    double   bag_time_frequency;
    double   time_scale;
    int      queue_size;
    ros::WallDuration advertise_sleep;
    bool     try_future;
    bool     has_time;
    float    time;

    std::vector<std::string> bags;
};


//! PRIVATE. A helper class to track relevant state for publishing time
class TimePublisher {
public:
    /*! Create a time publisher
     *  A publish_frequency of < 0 indicates that time shouldn't actually be published
     */
    TimePublisher();

    void setPublishFrequency(double publish_frequency);
    
    void setTimeScale(double time_scale);

    /*! Set the horizon that the clock will run to */
    void setHorizon(const ros::Time& horizon);

    /*! Set the horizon that the clock will run to */
    void setWCHorizon(const ros::WallTime& horizon);

    /*! Set the current time */
    void setTime(const ros::Time& time);

    /*! Run the clock for AT MOST duration
     *
     * If horizon has been reached this function returns immediately
     */
    void runClock(const ros::WallDuration& duration);

    //! Sleep as necessary, but don't let the click run 
    void runStalledClock(const ros::WallDuration& duration);

    //! Step the clock to the horizon
    void stepClock();

    bool horizonReached();

    
private:
    bool do_publish_;
    
    double publish_frequency_;
    double time_scale_;
    
    ros::NodeHandle node_handle_;
    ros::Publisher time_pub_;
    
    ros::WallDuration wall_step_;
    
    ros::WallTime next_pub_;

    ros::WallTime wc_horizon_;
    ros::Time horizon_;
    ros::Time current_;
};


//! PRIVATE.  Player class to abstract the interface to the player
/*!
 *  This API is currently considered private, but will be released in the 
 * future after view.
 */
class Player
{
public:
    Player(PlayerOptions const& options);
    ~Player();

    void publish();

private:
    char readCharFromStdin();
    void setupTerminal();
    void restoreTerminal();

    ros::Time getSysTime();
    void      doPublish(rosbag::MessageInstance const& m);

private:
    PlayerOptions options_;

    ros::NodeHandle node_handle_;

    bool      paused_;

    ros::WallTime paused_time_;

    std::vector<boost::shared_ptr<Bag> >  bags_;
    std::map<std::string, ros::Publisher> publishers_;

    // Terminal
    bool    terminal_modified_;
    termios orig_flags_;
    fd_set  stdin_fdset_;
    int     maxfd_;

    TimeTranslator time_translator_;
    TimePublisher time_publisher_;
};


} // namespace rosbag

#endif
