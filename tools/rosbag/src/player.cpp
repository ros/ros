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

#include "rosbag/player.h"
#include "rosbag/message_instance.h"
#include "rosbag/view.h"

#include <sys/select.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "roslib/Clock.h"

#define foreach BOOST_FOREACH

using std::map;
using std::pair;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Exception;

namespace rosbag {

// PlayerOptions

PlayerOptions::PlayerOptions() :
    quiet(false),
    start_paused(false),
    at_once(false),
    bag_time(false),
    bag_time_frequency(0.0),
    time_scale(1.0),
    queue_size(0),
    advertise_sleep(200000),
    try_future(false),
    has_time(false),
    time(0.0f)
{
}

void PlayerOptions::check() {
    if (bags.size() == 0)
        throw Exception("You must specify at least one bag file to play from");
}

// Player

Player::Player(PlayerOptions const& options) :
    options_(options),
    paused_(false),
    terminal_modified_(false)
{
}

Player::~Player() {
    foreach(shared_ptr<Bag> bag, bags_)
        bag->close();

    restoreTerminal();
}

void Player::publish() {
    options_.check();

    // Open all the bag files
    foreach(string const& filename, options_.bags) {
        ROS_INFO("Opening %s", filename.c_str());

        try
        {
            shared_ptr<Bag> bag(new Bag);
            bag->open(filename, bagmode::Read);
            bags_.push_back(bag);
        }
        catch (BagUnindexedException ex) {
            std::cerr << "Bag file " << filename << " is unindexed.  Run rosbag reindex." << std::endl;
            return;
        }
    }

    setupTerminal();

    if (!node_handle_.ok())
      return;

    if (!options_.quiet)
      puts("");
    
    // Publish all messages in the bags
    View view;
    foreach(shared_ptr<Bag> bag, bags_)
        view.addQuery(*bag);

    // Advertise all of our messages
    foreach(const ConnectionInfo* c, view.getConnections())
    {
        ros::M_string::const_iterator header_iter = c->header->find("callerid");
        std::string callerid = (header_iter != c->header->end() ? header_iter->second : string(""));

        string callerid_topic = callerid + c->topic;

        map<string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
        if (pub_iter == publishers_.end()) {

            ros::AdvertiseOptions opts = createAdvertiseOptions(c, options_.queue_size);

            ros::Publisher pub = node_handle_.advertise(opts);
            publishers_.insert(publishers_.begin(), pair<string, ros::Publisher>(callerid_topic, pub));

            pub_iter = publishers_.find(callerid_topic);
        }
    }

    std::cout << "Waiting " << options_.advertise_sleep.toSec() << " seconds after advertising topics..." << std::flush;
    options_.advertise_sleep.sleep();
    std::cout << " done." << std::endl;

    std::cout << std::endl << "Hit space to toggle paused, or 's' to step." << std::endl;

    // Set up our time_translator and publishers

    time_translator_.setTimeScale(options_.time_scale);

    start_time_ = view.begin()->getTime();
    time_translator_.setRealStartTime(start_time_);
    bag_length_ = view.getEndTime() - view.getBeginTime();

    time_publisher_.setTime(start_time_);

    ros::WallTime now_wt = ros::WallTime::now();
    time_translator_.setTranslatedStartTime(ros::Time(now_wt.sec, now_wt.nsec));


    time_publisher_.setTimeScale(options_.time_scale);
    if (options_.bag_time)
        time_publisher_.setPublishFrequency(options_.bag_time_frequency);
    else
        time_publisher_.setPublishFrequency(-1.0);

    paused_ = options_.start_paused;
    paused_time_ = now_wt;

    // Call do-publish for each message
    foreach(MessageInstance m, view) {
        if (!node_handle_.ok())
            break;
        
        doPublish(m);
    }
    
    std::cout << std::endl << "Done." << std::endl;
    
    ros::shutdown();
}

void Player::printTime()
{
    if (!options_.quiet) {

        ros::Time current_time = time_publisher_.getTime();
        ros::Duration d = current_time - start_time_;

        if (paused_)
        {
            printf("\r [PAUSED]   Bag Time: %13.6f   Duration: %.6f / %.6f     \r", time_publisher_.getTime().toSec(), d.toSec(), bag_length_.toSec());
        }
        else
        {
            printf("\r [RUNNING]  Bag Time: %13.6f   Duration: %.6f / %.6f     \r", time_publisher_.getTime().toSec(), d.toSec(), bag_length_.toSec());
        }
        fflush(stdout);
    }
}

void Player::doPublish(MessageInstance const& m) {
    string const& topic   = m.getTopic();
    ros::Time const& time = m.getTime();
    string callerid       = m.getCallerId();
    
    ros::Time translated = time_translator_.translate(time);
    ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

    time_publisher_.setHorizon(time);
    time_publisher_.setWCHorizon(horizon);

    string callerid_topic = callerid + topic;

    map<string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
    ROS_ASSERT(pub_iter != publishers_.end());

    if (options_.at_once) {
        time_publisher_.stepClock();
        pub_iter->second.publish(m);
        printTime();
        return;
    }

    while ((paused_ || !time_publisher_.horizonReached()) && node_handle_.ok())
    {
        bool charsleftorpaused = true;
        while (charsleftorpaused && node_handle_.ok())
        {
            switch (readCharFromStdin()){
            case ' ':
                paused_ = !paused_;
                if (paused_) {
                    paused_time_ = ros::WallTime::now();
                }
                else
                {
                    ros::WallDuration shift = ros::WallTime::now() - paused_time_;
                    paused_time_ = ros::WallTime::now();
         
                    time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                    horizon += shift;
                    time_publisher_.setWCHorizon(horizon);
                }
                break;
            case 's':
                if (paused_) {
                    time_publisher_.stepClock();

                    ros::WallDuration shift = ros::WallTime::now() - horizon ;
                    paused_time_ = ros::WallTime::now();

                    time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                    horizon += shift;
                    time_publisher_.setWCHorizon(horizon);
            
                    (pub_iter->second).publish(m);

                    printTime();
                    return;
                }
                break;
            case EOF:
                if (paused_)
                {
                    printTime();
                    time_publisher_.runStalledClock(ros::WallDuration(.1));
                }
                else
                    charsleftorpaused = false;
            }
        }

        printTime();
        time_publisher_.runClock(ros::WallDuration(.1));
    }

    pub_iter->second.publish(m);
}

void Player::setupTerminal() {
	if (terminal_modified_)
		return;

    const int fd = fileno(stdin);
    termios flags;
    tcgetattr(fd, &orig_flags_);
    flags = orig_flags_;
    flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
    flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;         // block if waiting for char
    tcsetattr(fd, TCSANOW, &flags);

    FD_ZERO(&stdin_fdset_);
    FD_SET(fd, &stdin_fdset_);
    maxfd_ = fd + 1;

    terminal_modified_ = true;
}

void Player::restoreTerminal() {
	if (!terminal_modified_)
		return;

    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);

    terminal_modified_ = false;
}

char Player::readCharFromStdin() {
#ifdef __APPLE__
    fd_set testfd;
    FD_COPY(&stdin_fdset_, &testfd);
#else
    fd_set testfd = stdin_fdset_;
#endif

    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 0;
    if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0)
        return EOF;

    return getc(stdin);
}

TimePublisher::TimePublisher() : time_scale_(1.0)
{
  setPublishFrequency(-1.0);
  time_pub_ = node_handle_.advertise<roslib::Clock>("clock",1);
}

void TimePublisher::setPublishFrequency(double publish_frequency)
{
  publish_frequency_ = publish_frequency;
  
  do_publish_ = (publish_frequency > 0.0);

  wall_step_.fromSec(1.0 / publish_frequency);
}

void TimePublisher::setTimeScale(double time_scale)
{
    time_scale_ = time_scale;
}

void TimePublisher::setHorizon(const ros::Time& horizon)
{
    horizon_ = horizon;
}

void TimePublisher::setWCHorizon(const ros::WallTime& horizon)
{
  wc_horizon_ = horizon;
}

void TimePublisher::setTime(const ros::Time& time)
{
    current_ = time;
}

ros::Time const& TimePublisher::getTime() const
{
    return current_;
}

void TimePublisher::runClock(const ros::WallDuration& duration)
{
    if (do_publish_)
    {
        roslib::Clock pub_msg; 

        ros::WallTime t = ros::WallTime::now();
        ros::WallTime done = t + duration;

        while (t < done && t < wc_horizon_)
        {
            ros::WallDuration leftHorizonWC = wc_horizon_ - t;

            ros::Duration d(leftHorizonWC.sec, leftHorizonWC.nsec);
            d *= time_scale_;

            current_ = horizon_ - d;

            if (current_ >= horizon_)
              current_ = horizon_;

            if (t >= next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }

            ros::WallTime target = done;
            if (target > wc_horizon_)
              target = wc_horizon_;
            if (target > next_pub_)
              target = next_pub_;

            ros::WallTime::sleepUntil(target);

            t = ros::WallTime::now();
        }
    } else {

        ros::WallTime t = ros::WallTime::now();

        ros::WallDuration leftHorizonWC = wc_horizon_ - t;

        ros::Duration d(leftHorizonWC.sec, leftHorizonWC.nsec);
        d *= time_scale_;

        current_ = horizon_ - d;
        
        if (current_ >= horizon_)
            current_ = horizon_;

        ros::WallTime target = ros::WallTime::now() + duration;

        if (target > wc_horizon_)
            target = wc_horizon_;

        ros::WallTime::sleepUntil(target);
    }
}

void TimePublisher::stepClock()
{
    if (do_publish_)
    {
        current_ = horizon_;

        roslib::Clock pub_msg; 

        pub_msg.clock = current_;
        time_pub_.publish(pub_msg);

        ros::WallTime t = ros::WallTime::now();
        next_pub_ = t + wall_step_;
    } else {
        current_ = horizon_;
    }
}

void TimePublisher::runStalledClock(const ros::WallDuration& duration)
{
    if (do_publish_)
    {
        roslib::Clock pub_msg; 

        ros::WallTime t = ros::WallTime::now();
        ros::WallTime done = t + duration;

        while ( t < done )
        {
            if (t > next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }

            ros::WallTime target = done;

            if (target > next_pub_)
              target = next_pub_;

            ros::WallTime::sleepUntil(target);

            t = ros::WallTime::now();
        }
    } else {
        duration.sleep();
    }
}

bool TimePublisher::horizonReached()
{
  return ros::WallTime::now() > wc_horizon_;
}

} // namespace rosbag
