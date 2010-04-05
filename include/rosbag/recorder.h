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

#include <queue>
#include <sys/stat.h>
#include <string>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Empty.h>
#include <topic_tools/shape_shifter.h>

namespace rosbag
{

class OutgoingMessage
{
public:
    OutgoingMessage(const std::string& _topic_name, topic_tools::ShapeShifter::ConstPtr _msg, ros::Time _time);

    std::string                         topic_name;
    topic_tools::ShapeShifter::ConstPtr msg;
    ros::Time                           time;
};

class OutgoingQueue
{
public:
    OutgoingQueue(const std::string& _fname, std::queue<OutgoingMessage>* _queue, ros::Time _time);

    std::string                  fname;
    std::queue<OutgoingMessage>* queue;
    ros::Time                    time;
};

class Recorder
{
public:
	Recorder();

	int run(int argc, char** argv);

private:
	void print_usage();
	void print_help();
	void do_queue(topic_tools::ShapeShifter::ConstPtr msg, std::string topic_name, boost::shared_ptr<ros::Subscriber> subscriber, boost::shared_ptr<int> count);
	void snapshot_trigger(std_msgs::Empty::ConstPtr trigger);
	void do_record();
	void do_record_bb();
	void do_check_master(const ros::TimerEvent& e, ros::NodeHandle& node_handle);
	void do_trigger();

	template<class T>
	static std::string time_to_str(T ros_t);

private:
	bool                          verbose_;              //!< verbose flag
	bool                          snapshot_;             //!< snapshot flag

	int                           exit_code_;            //!< eventual exit code
	std::set<std::string>         currently_recording_;  //!< set of currenly recording topics
	int                           count_;                //!< used for initialization of counting messages
	int                           num_subscribers_;      //!< used for book-keeping of our number of subscribers
	std::queue<OutgoingMessage>*  queue_;                //!< queue for storing
	uint64_t                      queue_size_;           //!< queue size
	uint64_t                      max_queue_size_;       //!< max queue size
	uint64_t                      split_size_;           //!< split size
	uint64_t                      split_count_;          //!< split count
	std::queue<OutgoingQueue>     queue_queue_;          //!< queue of queues to be used by the snapshot recorders
	boost::mutex                  queue_mutex_;          //!< mutex for queue
	boost::condition_variable_any queue_condition_;      //!< conditional variable for queue
	bool                          add_date_;             //!< flag to add date
	std::string                   prefix_;               //!< prefix
};

}

#endif
