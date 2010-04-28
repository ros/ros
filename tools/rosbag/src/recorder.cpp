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

#include "rosbag/recorder.h"

#include <sys/stat.h>
#include <sys/statvfs.h>
#include <time.h>

#include <queue>
#include <set>
#include <sstream>
#include <string>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#define foreach BOOST_FOREACH

using std::cout;
using std::endl;
using std::set;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Time;

namespace rosbag {

// OutgoingMessage

OutgoingMessage::OutgoingMessage(string const& _topic, topic_tools::ShapeShifter::ConstPtr _msg, Time _time) :
    topic(_topic),
    msg(_msg),
    time(_time)
{
}

// OutgoingQueue

OutgoingQueue::OutgoingQueue(string const& _filename, std::queue<OutgoingMessage>* _queue, Time _time) :
    filename(_filename),
    queue(_queue),
    time(_time)
{
}

// RecorderOptions

RecorderOptions::RecorderOptions() :
    trigger(false),
    record_all(false),
    quiet(false),
    append_date(true),
    snapshot(false),
    verbose(false),
    compression(compression::None),
    prefix(""),
    name(""),
    split_size(0),
    buffer_size(0),
    limit(0)
{
}

// Recorder

Recorder::Recorder(RecorderOptions const& options) :
    options_(options),
	num_subscribers_(0),
	exit_code_(0),
	queue_size_(0),
	max_queue_size_(1048576 * 256),
	split_count_(0),
    writing_enabled_(true)
{
}

int Recorder::run() {
    if (options_.trigger) {
        doTrigger();
        return 0;
    }

    if (options_.topics.size() == 0) {
        // Make sure limit is not specified with automatic topic subscription
        if (options_.limit > 0) {
            fprintf(stderr, "Specifing a count is not valid with automatic topic subscription.\n");
            return 1;
        }

        // Make sure topics are specified
        if (!options_.record_all) {
            fprintf(stderr, "No topics specified.\n");
            return 1;
        }
    }

    ros::NodeHandle nh;
    if (!nh.ok())
        return 0;

    last_buffer_warn_ = Time();

    queue_ = new std::queue<OutgoingMessage>;

    // Spin up a thread for writing to the file
    boost::thread record_thread;
    if (options_.snapshot)
        record_thread = boost::thread(boost::bind(&Recorder::doRecordSnapshotter, this));
    else
        record_thread = boost::thread(boost::bind(&Recorder::doRecord, this));

    // Subscribe to the snapshot trigger
    ros::Subscriber trigger_sub = nh.subscribe<std_msgs::Empty>("snapshot_trigger", 100, boost::bind(&Recorder::snapshotTrigger, this, _1));

    // Subscribe to each topic
    foreach(string const& topic, options_.topics)
        subscribe(topic);

    ros::Timer check_master_timer;
    if (options_.record_all)
        check_master_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&Recorder::doCheckMaster, this, _1, boost::ref(nh)));

    ros::MultiThreadedSpinner s(10);
    ros::spin(s);

    queue_condition_.notify_all();

    record_thread.join();

    delete queue_;

    return exit_code_;
}

bool Recorder::isSubscribed(string const& topic) const {
    return currently_recording_.find(topic) != currently_recording_.end();
}

shared_ptr<ros::Subscriber> Recorder::subscribe(string const& topic) {
    ros::NodeHandle nh;
    shared_ptr<int> count(new int(options_.limit));
    shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
    *sub = nh.subscribe<topic_tools::ShapeShifter>(topic, 100, boost::bind(&Recorder::doQueue, this, _1, topic, sub, count));
    currently_recording_.insert(topic);
    num_subscribers_++;

    return sub;
}

template<class T>
std::string Recorder::timeToStr(T ros_t) {
    char buf[1024] = "";
    time_t t = ros_t.sec;
    struct tm* tms = localtime(&t);
    strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
    return string(buf);
}

//! Callback to be invoked to save messages into a queue
void Recorder::doQueue(topic_tools::ShapeShifter::ConstPtr msg, string const& topic, shared_ptr<ros::Subscriber> subscriber, shared_ptr<int> count) {
    Time rectime = Time::now();
    
    if (options_.verbose)
        cout << "Received message on topic " << subscriber->getTopic() << endl;

    OutgoingMessage out(topic, msg, rectime);
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);

        queue_->push(out);
        queue_size_ += out.msg->msgBufUsed;
        
        // Check to see if buffer has been exceeded
        while (max_queue_size_ > 0 && queue_size_ > max_queue_size_) {
            OutgoingMessage drop = queue_->front();
            queue_->pop();
            queue_size_ -= drop.msg->msgBufUsed;

            if (!options_.snapshot) {
                Time now = Time::now();
                if (now > last_buffer_warn_ + ros::Duration(5.0)) {
                    ROS_WARN("rosbag record buffer exceeded.  Dropping oldest queued message.");
                    last_buffer_warn_ = now;
                }
            }
        }
    }
  
    if (!options_.snapshot)
        queue_condition_.notify_all();

    // If we are book-keeping count, decrement and possibly shutdown
    if ((*count) > 0) {
        (*count)--;
        if ((*count) == 0) {
            subscriber->shutdown();

            num_subscribers_--;

            if (num_subscribers_ == 0)
                ros::shutdown();
        }
    }
}

void Recorder::updateFilenames() {
    vector<string> parts;

    if (options_.prefix.length() > 0)
        parts.push_back(options_.prefix);
    if (options_.append_date)
        parts.push_back(timeToStr(ros::WallTime::now()));
    if (options_.split_size > 0)
        parts.push_back(boost::lexical_cast<string>(split_count_));

    target_filename_ = parts[0];
    for (unsigned int i = 1; i < parts.size(); i++)
        target_filename_ += string("_") + parts[i];

    target_filename_ += string(".bag");
    write_filename_ = target_filename_ + string(".active");
}

//! Callback to be invoked to actually do the recording
void Recorder::snapshotTrigger(std_msgs::Empty::ConstPtr trigger) {
    updateFilenames();
    
    ROS_INFO("Triggered snapshot recording with name %s.", target_filename_.c_str());
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        queue_queue_.push(OutgoingQueue(target_filename_, queue_, Time::now()));
        queue_      = new std::queue<OutgoingMessage>;
        queue_size_ = 0;
    }

    queue_condition_.notify_all();
}

void Recorder::startWriting() {
    bag_.setCompression(options_.compression);

    updateFilenames();
    try {
        bag_.open(write_filename_, bagmode::Write);
    }
    catch (rosbag::BagException e) {
        ROS_ERROR(e.what());
        exit_code_ = 1;
        ros::shutdown();
    }
    ROS_INFO("Recording to %s.", target_filename_.c_str());
}

void Recorder::stopWriting() {
    ROS_INFO("Closing %s.", target_filename_.c_str());
    bag_.close();
    rename(write_filename_.c_str(), target_filename_.c_str());
}

//! Thread that actually does writing to file.
void Recorder::doRecord() {
    // Open bag file for writing
    startWriting();

    // Schedule the disk space check
    warn_next_ = ros::WallTime();
    checkDisk();
    check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);

    // Technically the queue_mutex_ should be locked while checking empty.
    // Except it should only get checked if the node is not ok, and thus
    // it shouldn't be in contention.
    ros::NodeHandle nh;
    while (nh.ok() || !queue_->empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);

        bool finished = false;
        while (queue_->empty()) {
            if (!nh.ok()) {
                lock.release()->unlock();
                finished = true;
                break;
            }
            queue_condition_.wait(lock);
        }
        if (finished)
            break;

        OutgoingMessage out = queue_->front();
        queue_->pop();
        queue_size_ -= out.msg->msgBufUsed;
        
        lock.release()->unlock();

        if (options_.split_size > 0 && bag_.getOffset() > options_.split_size) {
            stopWriting();
            split_count_++;
            startWriting();
        }

        if (scheduledCheckDisk() && checkLogging())
        	bag_.write(out.topic, out.time, out.msg);
    }

    stopWriting();
}

void Recorder::doRecordSnapshotter() {
    ros::NodeHandle nh;
  
    while (nh.ok() || !queue_queue_.empty()) {
        boost::unique_lock<boost::mutex> lock(queue_mutex_);
        while (queue_queue_.empty()) {
            if (!nh.ok())
                return;
            queue_condition_.wait(lock);
        }
        
        OutgoingQueue out_queue = queue_queue_.front();
        queue_queue_.pop();
        
        lock.release()->unlock();
        
        string target_filename = out_queue.filename;
        string write_filename  = target_filename + string(".active");
        
        try {
            bag_.open(write_filename, bagmode::Write);
        }
        catch (rosbag::BagException ex) {
            ROS_ERROR(ex.what());
            return;
        }

        while (!out_queue.queue->empty()) {
            OutgoingMessage out = out_queue.queue->front();
            out_queue.queue->pop();

            bag_.write(out.topic, out.time, out.msg);
        }

        stopWriting();
    }
}

void Recorder::doCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle) {
    ros::master::V_TopicInfo all_topics;
    if (ros::master::getTopics(all_topics)) {
        foreach(ros::master::TopicInfo const& topic_info, all_topics) {
            if (!isSubscribed(topic_info.name))
                subscribe(topic_info.name);
        }
    }
}

void Recorder::doTrigger() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("snapshot_trigger", 1, true);
    pub.publish(std_msgs::Empty());

    ros::Timer terminate_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&ros::shutdown));
    ros::spin();
}

bool Recorder::scheduledCheckDisk() {
    boost::mutex::scoped_lock lock(check_disk_mutex_);

    if (ros::WallTime::now() < check_disk_next_)
        return true;

    check_disk_next_ += ros::WallDuration().fromSec(20.0);
    return checkDisk();
}

bool Recorder::checkDisk() {
    struct statvfs fiData;
    if ((statvfs(bag_.getFileName().c_str(), &fiData)) < 0) {
        ROS_WARN("Failed to check filesystem stats.");
        return true;
    }

    unsigned long long free_space = 0;
    free_space = (unsigned long long) (fiData.f_bsize) * (unsigned long long) (fiData.f_bavail);
    if (free_space < 1073741824ull) {
        ROS_ERROR("Less than 1GB of space free on disk with %s.  Disabling recording.", bag_.getFileName().c_str());
        writing_enabled_ = false;
        return false;
    }
    else if (free_space < 5368709120ull) {
        ROS_WARN("Less than 5GB of space free on disk with %s.", bag_.getFileName().c_str());
    }
    else {
        writing_enabled_ = true;
    }

    return true;
}

bool Recorder::checkLogging() {
    if (writing_enabled_)
        return true;

    ros::WallTime now = ros::WallTime::now();
    if (now >= warn_next_) {
        warn_next_ = now + ros::WallDuration().fromSec(5.0);
        ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
}

}
