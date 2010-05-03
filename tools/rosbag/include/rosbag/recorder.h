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

#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <queue>
#include <string>
#include <vector>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Empty.h>
#include <topic_tools/shape_shifter.h>

#include "rosbag/bag.h"
#include "rosbag/stream.h"

namespace rosbag {

class OutgoingMessage
{
public:
    OutgoingMessage(std::string const& _topic, topic_tools::ShapeShifter::ConstPtr _msg, ros::Time _time);

    std::string                         topic;
    topic_tools::ShapeShifter::ConstPtr msg;
    ros::Time                           time;
};

class OutgoingQueue
{
public:
    OutgoingQueue(std::string const& _filename, std::queue<OutgoingMessage>* _queue, ros::Time _time);

    std::string                  filename;
    std::queue<OutgoingMessage>* queue;
    ros::Time                    time;
};

struct RecorderOptions
{
    RecorderOptions();

    bool            trigger;
    bool            record_all;
    bool            quiet;
    bool            append_date;
    bool            snapshot;
    bool            verbose;
    CompressionType compression;
    std::string     prefix;
    std::string     name;
    uint32_t        split_size;
    uint32_t        buffer_size;
    uint32_t        limit;

    std::vector<std::string> topics;
};

class Recorder
{
public:
	Recorder(RecorderOptions const& options);

    void doTrigger();

    bool isSubscribed(std::string const& topic) const;

    boost::shared_ptr<ros::Subscriber> subscribe(std::string const& topic);

    int run();

private:
	void printUsage();

    void updateFilenames();
    void startWriting();
    void stopWriting();

    bool checkLogging();
    bool scheduledCheckDisk();
    bool checkDisk();

    void snapshotTrigger(std_msgs::Empty::ConstPtr trigger);
    void doQueue(topic_tools::ShapeShifter::ConstPtr msg, std::string const& topic, boost::shared_ptr<ros::Subscriber> subscriber, boost::shared_ptr<int> count);
	void doRecord();
	void doRecordSnapshotter();
	void doCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle);

	template<class T>
	static std::string timeToStr(T ros_t);

private:
	RecorderOptions               options_;

	Bag                           bag_;

	std::string                   target_filename_;
	std::string                   write_filename_;

    std::set<std::string>         currently_recording_;  //!< set of currenly recording topics
    int                           num_subscribers_;      //!< used for book-keeping of our number of subscribers

    int                           exit_code_;            //!< eventual exit code

    boost::condition_variable_any queue_condition_;      //!< conditional variable for queue
    boost::mutex                  queue_mutex_;          //!< mutex for queue
	std::queue<OutgoingMessage>*  queue_;                //!< queue for storing
	uint64_t                      queue_size_;           //!< queue size
	uint64_t                      max_queue_size_;       //!< max queue size

	uint64_t                      split_count_;          //!< split count

	std::queue<OutgoingQueue>     queue_queue_;          //!< queue of queues to be used by the snapshot recorders

	ros::Time                     last_buffer_warn_;

    bool          writing_enabled_;
    boost::mutex  check_disk_mutex_;
    ros::WallTime check_disk_next_;
    ros::WallTime warn_next_;
};

} // namespace rosbag

#endif
