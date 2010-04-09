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

#include <sys/select.h>

#include <boost/foreach.hpp>

using std::map;
using std::pair;
using std::string;
using std::vector;
using ros::Exception;

namespace rosbag {

// PlayerOptions

PlayerOptions::PlayerOptions() :
    check_bag(false),
    show_defs(false),
    at_once(false),
    quiet(false),
    paused(false),
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
        if (paused)
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

Player::Player(const PlayerOptions& options) :
    options_(options),
    node_handle_(NULL),
    bag_time_initialized_(false),
    shifted_(false),
    bag_time_publisher_(NULL)
{
}

Player::~Player() {
    if (node_handle_) {
        usleep(1000000);     // this shouldn't be necessary
        delete node_handle_;
    }

    delete bag_time_publisher_;

    unsetTerminalSettings();
}

void Player::run() {
    options_.check();

    setTerminalSettings();

    node_handle_ = new ros::NodeHandle;
    bag_time_publisher_ = new TimePublisher;

    if (options_.bag_time)
        bag_time_publisher_->initialize(options_.bag_time_frequency, options_.time_scale);

    start_time_ = getSysTime();
    requested_start_time_ = start_time_;

    /*
    Bag bag;
    if (!bag.open(filename, bagmode::Read))
        throw Exception("Error opening file: %s", filename.c_str());

    BOOST_FOREACH(const MessageInstance& m, bag.getMessageList()) {
    }
    */
    /*
    if (player_->open(options_.bags, start_time_ + ros::Duration().fromSec(-options_.time), time_scale_, options_.try_future))
        player_->addHandler<AnyMsg>(string("*"), &Player::doPublish, this, NULL, false);
    else {
        // Not sure exactly why, but this shutdown is necessary for the
        // exception to be received by the main thread without giving a Ctrl-C.
        ros::shutdown();  
        throw std::runtime_error("Failed to open one of the bag files.");
    }
    */

    if (!options_.at_once) {
        if (options_.paused) {
            paused_time_ = getSysTime();
            std::cout << "Hit space to resume, or 's' to step.";
            std::cout.flush();
        }
        else {
            std::cout << "Hit space to pause.";
            std::cout.flush();
        }
    }
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

bool Player::spin() {
    if (node_handle_ && node_handle_->ok()) {
        if (!options_.quiet)
            puts("");

        ros::WallTime last_print_time(0.0);
        ros::WallDuration max_print_interval(0.1);

        /*
        while (node_handle_->ok()) {
            if (!player_->nextMsg())
                break;

            ros::WallTime t = ros::WallTime::now();
            if (!options_.quiet && ((t - last_print_time) >= max_print_interval)) {
                printf("Time: %16.6f    Duration: %16.6f\r", ros::Time::now().toSec(), player_->getDuration().toSec());
                fflush(stdout);
                last_print_time = t;
            }
        }
        */
        std::cout << std::endl << "Done." << std::endl;

        ros::shutdown();
    }

    return true;
}

ros::Time Player::getSysTime() {
    struct timeval timeofday;
    gettimeofday(&timeofday, NULL);
    return ros::Time().fromNSec(1e9 * timeofday.tv_sec + 1e3 * timeofday.tv_usec);
}

void Player::doPublish(string topic_name, ros::Message* m, ros::Time play_time, ros::Time record_time, void* n) {
    if (play_time < requested_start_time_)
        return;

    // if we are using the bag time
    if (options_.bag_time) {
        // initialize bag time publisher
        if (!bag_time_initialized_) {
            // starting in paused mode
            if (options_.paused)
                bag_time_publisher_->stepTime(record_time);
            // starting in play mode
            else 
                bag_time_publisher_->startTime(record_time);
            bag_time_initialized_ = true;
        }

        if (options_.at_once)
            bag_time_publisher_->startTime(record_time);
        else
            bag_time_publisher_->setHorizon(play_time);
    }
    
    // We pull latching and callerid info out of the connection_header if it's available (which it always should be)
    bool latching = false;
    string callerid("");
    
    if (m->__connection_header != NULL) {
        ros::M_string::iterator latch_iter = m->__connection_header->find(string("latching"));
        if (latch_iter != m->__connection_header->end() && latch_iter->second != string("0"))
            latching = true;

        ros::M_string::iterator callerid_iter = m->__connection_header->find(string("callerid"));
        if (callerid_iter != m->__connection_header->end())
            callerid = callerid_iter->second;
    }

    // We make a unique id composed of the callerid and the topicname allowing us to have separate advertisers
    // for separate latching topics.
    
    string name = callerid + topic_name;
  
    map<string, ros::Publisher>::iterator pub_token = publishers_.find(name);
    if (pub_token == publishers_.end()) {
        ros::AdvertiseOptions opts(topic_name, options_.queue_size, m->__getMD5Sum(), m->__getDataType(), m->__getMessageDefinition());

        // Have to set the latching field explicitly
        opts.latch = latching;
        ros::Publisher pub = node_handle_->advertise(opts);
        publishers_.insert(publishers_.begin(), pair<string, ros::Publisher>(name, pub));
        pub_token = publishers_.find(name);

        if (options_.bag_time)
            bag_time_publisher_->freezeTime();
        ros::Time paused_time_ = getSysTime();

        ROS_INFO("Sleeping %.3f seconds after advertising %s...", options_.advertise_sleep / 1e6, topic_name.c_str());
        usleep(options_.advertise_sleep);
        ROS_INFO("Done sleeping.\n");

        ros::Duration shift = getSysTime() - paused_time_;
        //player_->shiftTime(shift);
        if (options_.bag_time)
            bag_time_publisher_->startTime(record_time);
    }

    if (!options_.at_once) {
        ros::Time now = getSysTime();
        ros::Duration delta = play_time - getSysTime();
        
        while ((options_.paused || delta > ros::Duration(0, 100000)) && node_handle_->ok()) {
            bool charsleftorpaused = true;
      
            while (charsleftorpaused && node_handle_->ok()) {
                // Read from stdin:
        
                char c = EOF;
                
#ifdef __APPLE__
                fd_set testfd;
                FD_COPY(&stdin_fdset_, &testfd);           
#else
                fd_set testfd = stdin_fdset_;
#endif

                timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 0;
                
                if (select(maxfd_, &testfd, NULL, NULL, &tv) > 0)
                    c = getc(stdin);
                
                switch (c) {
                case ' ':
                    options_.paused = !options_.paused;
                    if (options_.paused) {
                        if (options_.bag_time)
                            bag_time_publisher_->freezeTime();
                        paused_time_ = getSysTime();
                        std::cout << std::endl << "Hit space to resume, or 's' to step.";
                        std::cout.flush();
                    }
                    else {
                        if (options_.bag_time)
                        	bag_time_publisher_->startTime(record_time);

                        ros::Duration shift;
                        if (shifted_) {
                            shift = getSysTime() - play_time;
                            play_time = getSysTime();
                            shifted_ = false;
                        }
                        else {
                            shift = getSysTime() - paused_time_;
                            play_time = play_time + shift;
                        }
                        //player_->shiftTime(shift);

                        std::cout << std::endl << "Hit space to pause.";
                        std::cout.flush();
                    }
                    break;
                case 's':
                    if (options_.paused) {
                        shifted_ = true;
                        if (options_.bag_time)
                            bag_time_publisher_->stepTime(record_time);
                        pub_token->second.publish(*m);
                        return;
                    }
                    break;
                case EOF:
                    if (options_.paused)
                        usleep(10000);
                    else
                        charsleftorpaused = false;
                }
            }
      
            usleep(100000);
            
            delta = play_time - getSysTime();
        }
        
        if (!options_.paused && delta > ros::Duration(0, 5000) && node_handle_->ok())
            usleep(delta.toNSec() / 1000 - 5);      // todo should this be a ros::Duration::Sleep?
    }

    pub_token->second.publish(*m);
}

int Player::checkBag() {
    options_.check();

    string filename = options_.bags[0];

    Bag bag;
    if (!bag.open(filename, bagmode::Read))
        throw Exception("Error opening file");

    ros::Time end_time;
    BOOST_FOREACH(const MessageInstance& m, bag.getMessageList()) {
        const string& topic = m.getTopic();

        map<string, BagContent>::iterator i = content_.find(topic);
        if (i == content_.end()) {
            BagContent content(m.__getDataType(), m.__getMD5Sum(), m.__getMessageDefinition());
            content_.insert(pair<string, BagContent>(topic, content));
        }
        else
            i->second.count++;

        end_time = m.getTime();
    }

    // Print information about bag file
    printf("bag:        %s\n",    filename.c_str());
    printf("version:    %d.%d\n", bag.getMajorVersion(), bag.getMinorVersion());
    //printf("start_time: %llu\n",  (unsigned long long) player.getFirstDuration().toNSec());
    //printf("end_time:   %llu\n",  (unsigned long long) (end_time + player.getFirstDuration()).toNSec());
    printf("length:     %llu\n",  (unsigned long long) end_time.toNSec());

    printf("topics:\n");
    for (map<string, BagContent>::const_iterator i = content_.begin(); i != content_.end(); i++) {
        const string&     topic   = i->first;
        const BagContent& content = i->second;

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
