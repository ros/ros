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
	if (bag_time && bags.size() > 1)
		throw Exception("You can only play one single bag when using bag time [-b]");
}

// Player

Player::Player(PlayerOptions const& options) :
    options_(options),
    node_handle_(NULL),
    started_(false),
    paused_(false),
    terminal_modified_(false)
{
}

Player::~Player() {
    if (node_handle_) {
        usleep(1000000);     // this shouldn't be necessary
        delete node_handle_;
    }

    foreach(shared_ptr<Bag> bag, bags_)
        bag->close();

	restoreTerminal();
}

void Player::publish() {
    options_.check();

    // Open all the bag files
    foreach(string const& filename, options_.bags) {
        ROS_INFO("Opening %s", filename.c_str());

        shared_ptr<Bag> bag(new Bag);
        bag->open(filename, bagmode::Read);

        bags_.push_back(bag);
    }

    if (!options_.at_once) {
        if (options_.start_paused) {
            paused_ = true;
            std::cout << "Hit space to resume, or 's' to step." << std::flush;
        }
        else
            std::cout << "Hit space to pause." << std::flush;
    }

    setupTerminal();

    node_handle_ = new ros::NodeHandle();
    if (!node_handle_ || !node_handle_->ok())
    	return;

	if (!options_.quiet)
		puts("");

	ros::WallTime last_print_time(0.0);
	ros::WallDuration max_print_interval(0.1);

	// Publish all messages in the bags
	View view;
	foreach(shared_ptr<Bag> bag, bags_)
		view.addQuery(*bag, Query());

	foreach(MessageInstance m, view) {
		if (!node_handle_->ok())
			break;

		// Print out time
		ros::WallTime t = ros::WallTime::now();
		if (!options_.quiet && ((t - last_print_time) >= max_print_interval)) {
			printf(" Time: %16.6f    Duration: %16.6f\r", ros::Time::now().toSec(), m.getTime().toSec());
			fflush(stdout);
			last_print_time = t;
		}

		doPublish(m);
	}

	std::cout << std::endl << "Done." << std::endl;

	ros::shutdown();
}

void Player::doPublish(MessageInstance const& m) {
	string const& topic = m.getTopic();
	ros::Time const& time = m.getTime();

    // Make a unique id composed of the callerid and the topic allowing separate advertisers for separate latching topics

	ros::M_string const& header = m.getConnectionHeader();

	ros::M_string::const_iterator header_iter = header.find("callerid");
    string callerid;
    if (header_iter != header.end())
        callerid = header_iter->second;

    string callerid_topic = callerid + topic;

    map<string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
    if (pub_iter == publishers_.end()) {
        ros::AdvertiseOptions opts = createAdvertiseOptions(m, options_.queue_size);

        ros::M_string::const_iterator header_iter = header.find("latching");
        opts.latch = (header_iter != header.end() && header_iter->second == "1");

        ros::Publisher pub = node_handle_->advertise(opts);
        publishers_.insert(publishers_.begin(), pair<string, ros::Publisher>(callerid_topic, pub));

        pub_iter = publishers_.find(callerid_topic);

        std::cout << "Waiting " << options_.advertise_sleep / 1e6 << " seconds after advertising " << topic << " [caller-id: " << callerid << "]..." << std::flush;
        usleep(options_.advertise_sleep);
        std::cout << " done." << std::endl;
    }

    if (options_.at_once) {
        pub_iter->second.publish(m);
        return;
    }

    if (!started_) {
    	last_played_message_time_ = time;
    	last_played_wall_time_ = getSysTime();
    	started_ = true;
    }

    ros::Duration message_delta  = time - last_played_message_time_;
    ros::Duration wall_delta     = getSysTime() - last_played_wall_time_;
    ros::Duration sleep_duration = message_delta - wall_delta;

    while ((paused_ || sleep_duration > ros::Duration(0, 100000)) && node_handle_->ok()) {
        bool charsleftorpaused = true;
        while (charsleftorpaused && node_handle_->ok()) {
            switch (readCharFromStdin()) {
            case ' ':
                // <space>: toggle paused
                if (!paused_) {
                    paused_ = true;
                    std::cout << std::endl << "Hit space to resume, or 's' to step." << std::endl;
                }
                else {
                    paused_ = false;
                    std::cout << std::endl << "Hit space to pause." << std::endl;
                }
                break;

            case 's':
                // 's': step
                if (paused_) {
                    pub_iter->second.publish(m);
                    return;
                }
                break;

            case EOF:
                if (paused_)
                    usleep(10000);  // sleep for 10ms
                else
                    charsleftorpaused = false;
            }
        }

        usleep(100000);  // sleep for 100ms

        wall_delta     = getSysTime() - last_played_wall_time_;
        sleep_duration = message_delta - wall_delta;
    }

    if (!paused_ && sleep_duration > ros::Duration(0, 5000) && node_handle_->ok())
        usleep(sleep_duration.toNSec() / 1000 - 5);
    last_played_message_time_ = time;
    last_played_wall_time_ = getSysTime();

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

ros::Time Player::getSysTime() {
    struct timeval timeofday;
    gettimeofday(&timeofday, NULL);
    return ros::Time().fromNSec(1e9 * timeofday.tv_sec + 1e3 * timeofday.tv_usec);
}

}
