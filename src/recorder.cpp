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

#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <time.h>

#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include "rosbag/bag.h"

using std::cout;
using std::endl;
using std::set;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Time;

namespace rosbag {

// OutgoingMessage

OutgoingMessage::OutgoingMessage(const string& _topic_name, topic_tools::ShapeShifter::ConstPtr _msg, Time _time) :
    topic_name(_topic_name),
    msg(_msg),
    time(_time)
{
}

// OutgoingQueue

OutgoingQueue::OutgoingQueue(const string& _fname, std::queue<OutgoingMessage>* _queue, Time _time) :
    fname(_fname),
    queue(_queue),
    time(_time)
{
}

Recorder::Recorder() :
	verbose_(false),
	snapshot_(false),
	exit_code_(0),
	count_(-1),
	num_subscribers_(0),
	queue_size_(0),
	max_queue_size_(1048576 * 256),
	split_size_(0),
	split_count_(0),
	add_date_(true)
{
};

template<class T>
std::string Recorder::time_to_str(T ros_t) {
    char buf[1024] = "";
    time_t t = ros_t.sec;
    struct tm* tms = localtime(&t);
    strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
    return string(buf);
}

//! Helper function to print executable usage
void Recorder::print_usage() {
    fprintf(stderr, "Usage: rosbag [options] TOPIC1 [TOPIC2 TOPIC3...]\n"
                    "  rosbag logs ROS message data to a file.\n");
}

//! Helper function to print executable options
void Recorder::print_help() {
    print_usage();

    fprintf(stderr, "Options:\n");
    fprintf(stderr, " -c <num>    : Only receive <num> messages on each topic\n");
    fprintf(stderr, " -f <prefix> : Prepend file prefix to beginning of bag name (name will always end with date stamp)\n");
    fprintf(stderr, " -F <fname>  : Record to a file named exactly <fname>.bag\n");
    fprintf(stderr, " -a          : Record all published messages.\n");
    fprintf(stderr, " -v          : Display a message every time a message is received on a topic\n");
    fprintf(stderr, " -m          : Maximize internal buffer size in MB (Default: 256MB)  0 = infinite.\n");
    fprintf(stderr, " -s          : (EXPERIMENTAL) Enable snapshot recording (don't write to file unless triggered)\n");
    fprintf(stderr, " -t          : (EXPERIMENTAL) Trigger snapshot recording\n");
    fprintf(stderr, " -h          : Display this help message\n");
}

//! Callback to be invoked to save messages into a queue
void Recorder::do_queue(topic_tools::ShapeShifter::ConstPtr msg, string topic_name, shared_ptr<ros::Subscriber> subscriber, shared_ptr<int> count) {
    Time rectime = Time::now();
    
    if (verbose_)
        cout << "Received message on topic " << subscriber->getTopic() << endl;

    OutgoingMessage out(topic_name, msg, rectime);
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        queue_->push(out);
        queue_size_ += out.msg->msgBufUsed;
        
        while (max_queue_size_ > 0 && queue_size_ > max_queue_size_) {
            OutgoingMessage drop = queue_->front();
            queue_->pop();
            queue_size_ -= drop.msg->msgBufUsed;
            if (!snapshot_) {
                static ros::Time last = ros::Time();
                ros::Time now = ros::Time::now();
                if (now > last + ros::Duration(5.0)) {
                    ROS_WARN("rosbag buffer exceeded.  Dropping oldest queued message.");
                    last = now;
                }
            }
        }
    }
  
    if (!snapshot_)
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

//! Callback to be invoked to actually do the recording
void Recorder::snapshot_trigger(std_msgs::Empty::ConstPtr trigger) {
	// Build filename
    ros::WallTime rectime = ros::WallTime::now();
    
    vector<string> join;
    if (prefix_.length() > 0)
        join.push_back(prefix_);
    if (add_date_)
        join.push_back(time_to_str(rectime));
    
    string tgt_fname = join[0];
    for (size_t i = 1; i < join.size(); i++)
        tgt_fname += string("_") + join[i];    
    tgt_fname += string(".bag");
    
    ROS_INFO("Triggered snapshot recording with name %s.", tgt_fname.c_str());
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        
        OutgoingQueue out(tgt_fname, queue_, Time::now());
        
        queue_ = new std::queue<OutgoingMessage>;
        queue_size_ = 0;
        
        queue_queue_.push(out);
    }
    queue_condition_.notify_all();
}

//! Thread that actually does writing to file.
void Recorder::do_record() {
    ros::WallTime rectime = ros::WallTime::now();

    vector<string> join;
    if (prefix_.length() > 0)
        join.push_back(prefix_);
    if (add_date_)
        join.push_back(time_to_str(rectime));

    string base_name = join[0];
    for (size_t i = 1; i < join.size(); i++)
        base_name += string("_") + join[i];

    string split_name("");
    if (split_size_ > 0)
        split_name = string("_") + boost::lexical_cast<string>(split_count_++);

    string tgt_fname = base_name + split_name + string(".bag");
    string fname     = tgt_fname + string(".active");

    ROS_INFO("Recording to %s.", tgt_fname.c_str());

    ros::NodeHandle nh;

    // Open our bag file and add available topics
    Bag bag;
    if (!bag.open(fname, bagmode::Write)) {
        ROS_FATAL("Could not open output file: %s", fname.c_str());
        exit_code_ = 1;
        ros::shutdown();
    }

    // Technically the queue_mutex_ should be locked while checking empty.
    // Except it should only get checked if the node is not ok, and thus
    // it shouldn't be in contention.
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

        if (split_size_ > 0 && bag.getOffset() > split_size_) {
            bag.close();
            rename(fname.c_str(),tgt_fname.c_str());
            
            split_name = string("_") + boost::lexical_cast<string>(split_count_++);
            tgt_fname = base_name + split_name + string(".bag");
            fname     = tgt_fname + string(".active");

            if (!bag.open(string(fname), bagmode::Write)) {
                ROS_FATAL("Could not open output file: %s", fname.c_str());
                exit_code_ = 1;
                ros::shutdown();
            }
            
            ROS_INFO("Recording to %s.", tgt_fname.c_str());
        }
        
        bag.write(out.topic_name, out.time, out.msg);
    }

    // Close the file nicely
    ROS_INFO("Closing %s.", tgt_fname.c_str());
    bag.close();
    rename(fname.c_str(), tgt_fname.c_str());
}

void Recorder::do_record_bb() {
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
        
        string tgt_fname = out_queue.fname;
        string fname = tgt_fname + string(".active");
        
        Bag bag;
        if (bag.open(fname, bagmode::Write)) {
            while (!out_queue.queue->empty()) {
                OutgoingMessage out = out_queue.queue->front();
                out_queue.queue->pop();
                bag.write(out.topic_name, out.time, out.msg);
            }

            bag.close();

            // Rename the file to the actual target name
            rename(fname.c_str(), tgt_fname.c_str());
        }
        else {
            ROS_ERROR("Could not open file: %s", out_queue.fname.c_str());
        }
    }
}

void Recorder::do_check_master(const ros::TimerEvent& e, ros::NodeHandle& node_handle) {
    ros::master::V_TopicInfo all_topics;
    if (!ros::master::getTopics(all_topics))
    	return;

    for (ros::master::V_TopicInfo::const_iterator topic_iter = all_topics.begin(); topic_iter != all_topics.end(); topic_iter++) {
		// If we're not currently recording it, do so
		if (currently_recording_.find(topic_iter->name) == currently_recording_.end()) {
			shared_ptr<int> count(new int(count_));
			shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
			*sub = node_handle.subscribe<topic_tools::ShapeShifter>(topic_iter->name, 100, boost::bind(&Recorder::do_queue, this, _1, topic_iter->name, sub, count));
			currently_recording_.insert(topic_iter->name);
		}
	}
}

void Recorder::do_trigger() {
    ros::NodeHandle node_handle;
    ros::Publisher pub = node_handle.advertise<std_msgs::Empty>("snapshot_trigger", 1, true);
    pub.publish(std_msgs::Empty());
    ros::Timer terminate_timer = node_handle.createTimer(ros::Duration(1.0), boost::bind(&ros::shutdown));
    ros::spin();  
}

int Recorder::run(int argc, char** argv) {
    ros::init(argc, argv, "rosbag", ros::init_options::AnonymousName);

    bool check_master = false;   // whether master should be checked periodically

    int option_char;
    while ((option_char = getopt(argc, argv, "f:F:c:m:S:asthv")) != -1) {
        switch (option_char) {
        case 'f': prefix_ = string(optarg); break;
        case 'F': prefix_ = string(optarg); add_date_ = false; break;
        case 'c': count_ = atoi(optarg); break;
        case 'a': check_master = true; break;
        case 's': snapshot_ = true; break;
        case 't': do_trigger(); return 0; break;
        case 'v': verbose_ = true; break;
        case 'm': {
            int m = 0;
            m = atoi(optarg);
            if (m < 0) {
                fprintf(stderr, "Buffer size must be 0 or positive.\n");
                return 1;
            }
            max_queue_size_ = 1048576 * m;
        }
        break;
        case 'S': {
            int S = 0;
            S = atoi(optarg);
            if (S < 0) {
                fprintf(stderr, "Splitting size must be 0 or positive.\n");
                return 1;
            }
            split_size_ = 1048576 * S;
        }
        break;
        case 'h': print_help(); return 1;
        case '?': print_usage(); return 1;
        }
    }
    
    if (snapshot_)
        ROS_WARN("Using snapshot mode in rosbag is experimental and usage syntax is subject to change");

    // Logic to make sure count is not specified with automatic topic subscription (implied by no listed topics)
    if ((argc - optind) < 1) {
        if (count_ > 0) {
            fprintf(stderr, "Specifing a count is not valid with automatic topic subscription.\n");
            return 1;
        }
        if (!check_master) {
            ROS_WARN("Running rosbag with no arguments has been deprecated.  Please use 'rosbag -a' instead\n");
            check_master = true;
        }
    }
    
    ros::NodeHandle node_handle;
    if (!node_handle.ok())
        return 0;

    queue_ = new std::queue<OutgoingMessage>;

    // Spin up a thread for actually writing to file
    boost::thread record_thread;
    if (snapshot_)
        record_thread = boost::thread(boost::bind(&Recorder::do_record_bb, this));
    else
        record_thread = boost::thread(boost::bind(&Recorder::do_record, this));
    
    ros::Subscriber trigger = node_handle.subscribe<std_msgs::Empty>("snapshot_trigger", 100, boost::bind(&Recorder::snapshot_trigger, this, _1));
    
    // Every non-processed argument is assumed to be a topic
    for (int i = optind; i < argc; i++) {
        shared_ptr<int> count(new int(count_));
        shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
        *sub = node_handle.subscribe<topic_tools::ShapeShifter>(argv[i], 100, boost::bind(&Recorder::do_queue, this, _1, argv[i], sub, count));
        num_subscribers_++;
    }
    
    ros::Timer check_master_timer;
    if (check_master)
        check_master_timer = node_handle.createTimer(ros::Duration(1.0), boost::bind(&Recorder::do_check_master, this, _1, boost::ref(node_handle)));
    
    ros::MultiThreadedSpinner s(10);
    ros::spin(s);
    
    queue_condition_.notify_all();
    
    record_thread.join();
    
    delete queue_;

    return exit_code_;
}

}

int main(int argc, char** argv) {
	rosbag::Recorder recorder;
	return recorder.run(argc, argv);
}
