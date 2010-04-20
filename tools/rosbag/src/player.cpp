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
#include "rosbag/message_info.h"
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

// BagContent

BagContent::BagContent(std::string const& d, std::string const& m, std::string const& def) :
    datatype(d),
    md5sum(m),
    definition(def),
    count(1)
{
}

// PlayerOptions

PlayerOptions::PlayerOptions() :
    check_bag(false),
    show_defs(false),
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
    if (check_bag) {
        if (at_once)
            throw Exception("Option -a is not valid when checking bag");
        if (start_paused)
            throw Exception("Option -p is not valid when checking bag");
        if (has_time)
            throw Exception("Option -t is not valid when checking bag");
        if (queue_size)
            throw Exception("Option -q is not valid when checking bag");
        if (bags.size() > 1)
            throw Exception("Only 1 bag can be checked at a time");
    }
    else {
        if (bags.size() == 0)
            throw Exception("You must specify at least one bag file to play from");
        if (bag_time && bags.size() > 1)
            throw Exception("You can only play one single bag when using bag time [-b]");
    }
}

// Player

Player::Player(PlayerOptions const& options) :
    options_(options),
    node_handle_(NULL),
    paused_(false)
{
}

Player::~Player() {
    if (node_handle_) {
        usleep(1000000);     // this shouldn't be necessary
        delete node_handle_;
    }

    foreach(shared_ptr<Bag> bag, bags_)
        bag->close();

    unsetTerminalSettings();
}

void Player::publish() {
    options_.check();

    setTerminalSettings();

    node_handle_ = new ros::NodeHandle();

    // Open all the bag files
    foreach(string const& filename, options_.bags) {
        ROS_INFO("Opening %s", filename.c_str());

        shared_ptr<Bag> bag(new Bag);
        if (!bag->open(filename, bagmode::Read))
            throw Exception((boost::format("Error opening file: %1%") % filename.c_str()).str());

        bags_.push_back(bag);
    }

    View view;
    foreach(shared_ptr<Bag> bag, bags_)
		view.addQuery(*bag, Query());

    if (!options_.at_once) {
        if (options_.start_paused) {
            paused_ = true;
            std::cout << "Hit space to resume, or 's' to step." << std::flush;
        }
        else
            std::cout << "Hit space to pause." << std::flush;
    }

    if (node_handle_ && node_handle_->ok()) {
        if (!options_.quiet)
            puts("");

        ros::WallTime last_print_time(0.0);
        ros::WallDuration max_print_interval(0.1);

        foreach(MessageInfo m, view) {
            if (!node_handle_->ok())
                break;

            // Print out time
            ros::WallTime t = ros::WallTime::now();
            if (!options_.quiet && ((t - last_print_time) >= max_print_interval)) {
                printf("Time: %16.6f    Duration: %16.6f\r", ros::Time::now().toSec(), m.getTime().toSec());
                fflush(stdout);
                last_print_time = t;
            }

            // Publish the message
            std::cout << "Publishing: " << m.getTime() << std::endl;
            doPublish(m.getTopic(), m, m.getTime());
        }

        std::cout << std::endl << "Done." << std::endl;

        ros::shutdown();
    }
}

void Player::doPublish(string const& topic, rosbag::MessageInfo const& m, ros::Time const& time) {

    // Pull latching and callerid info out of the connection_header if it's available (which it always should be)
    bool latching = m.getLatching();
    string callerid = m.getCallerid();

    // Make a unique id composed of the callerid and the topicname allowing us to have separate advertisers for separate latching topics
    string name = callerid + topic;

    map<string, ros::Publisher>::iterator pub_iter = publishers_.find(name);
    if (pub_iter == publishers_.end()) {

        ros::AdvertiseOptions opts = rosbag::createAdvertiseOptions(m, options_.queue_size);
        opts.latch = latching;

        ros::Publisher pub = node_handle_->advertise(opts);
        publishers_.insert(publishers_.begin(), pair<string, ros::Publisher>(name, pub));

        pub_iter = publishers_.find(name);

        ROS_INFO("Sleeping %.3f seconds after advertising %s...", options_.advertise_sleep / 1e6, topic.c_str());
        usleep(options_.advertise_sleep);
        ROS_INFO("Done sleeping.\n");
    }

    if (options_.at_once) {
        pub_iter->second.publish(m);
        return;
    }

    ros::Time now = getSysTime();
    ros::Duration delta = time - getSysTime();

    while ((paused_ || delta > ros::Duration(0, 100000)) && node_handle_->ok()) {
        bool charsleftorpaused = true;
        while (charsleftorpaused && node_handle_->ok()) {
            switch (readCharFromStdin()) {
            case ' ':
                // <space>: toggle paused
                if (!paused_) {
                    paused_ = true;
                    std::cout << std::endl << "Hit space to resume, or 's' to step." << std::flush;
                }
                else {
                    paused_ = false;
                    std::cout << std::endl << "Hit space to pause." << std::flush;
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
                    usleep(10000);
                else
                    charsleftorpaused = false;
            }
        }

        usleep(100000);  // sleep for 0.1 secs

        delta = time - getSysTime();
    }

    if (!paused_ && delta > ros::Duration(0, 5000) && node_handle_->ok())
        usleep(delta.toNSec() / 1000 - 5);      // todo should this be a ros::Duration::Sleep?

    pub_iter->second.publish(m);
}

void Player::setTerminalSettings() {
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
}

void Player::unsetTerminalSettings() {
    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);
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

int Player::checkBag() {
    options_.check();

    // Open the bag file for reading
    string filename = options_.bags[0];
    Bag bag;
    if (!bag.open(filename, bagmode::Read))
        throw Exception((boost::format("Error opening file: %1%") % filename.c_str()).str());

    // Build a count of messages in each topic
    map<string, BagContent> content_by_topic;
    ros::Time end_time;

    View view;
	view.addQuery(bag, Query());

	foreach(MessageInfo m, view) {
        string const& topic = m.getTopic();

        map<string, BagContent>::iterator i = content_by_topic.find(topic);
        if (i == content_by_topic.end())
            content_by_topic.insert(std::pair<string, BagContent>(topic, BagContent(m.getDataType(), m.getMD5Sum(), m.getMessageDefinition())));
        else {
            BagContent& content = i->second;
            content.count++;
        }

        end_time = m.getTime();
    }

    // Print information about bag file
    printf("bag:        %s\n",    filename.c_str());
    printf("version:    %d.%d\n", bag.getMajorVersion(), bag.getMinorVersion());
    //printf("start_time: %llu\n",  (unsigned long long) player.getFirstDuration().toNSec());
    //printf("end_time:   %llu\n",  (unsigned long long) (end_time + player.getFirstDuration()).toNSec());
    printf("length:     %llu\n",  (unsigned long long) end_time.toNSec());

    printf("topics:\n");
    for (map<string, BagContent>::const_iterator i = content_by_topic.begin(); i != content_by_topic.end(); i++) {
        string const&     topic   = i->first;
        BagContent const& content = i->second;

        printf("  - name:       %s\n", topic.c_str());
        printf("    count:      %d\n", content.count);
        printf("    datatype:   %s\n", content.datatype.c_str());
        printf("    md5sum:     %s\n", content.md5sum.c_str());

        if (options_.show_defs) {
            string def = content.definition.c_str();
            if (def.length() > 0) {
                printf("    definition: |\n");

                size_t oldind = 0;
                size_t ind = def.find_first_of('\n', 0);
                while (ind != def.npos) {
                    printf("      %s\n", def.substr(oldind, ind - oldind).c_str());
                    oldind = ind + 1;
                    ind = def.find_first_of('\n', oldind);
                }
                ind = def.length();

                printf("      %s\n", def.substr(oldind, ind - oldind).c_str());
            }
            else {
                printf("    definition: NONE\n");
            }
        }

    }
    return 0;
}

}
