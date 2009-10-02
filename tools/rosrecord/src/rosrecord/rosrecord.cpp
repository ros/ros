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

#include <time.h>
#include <sys/stat.h>
#include "ros/ros.h"
#include "rosrecord/Recorder.h"
#include "topic_tools/shape_shifter.h"

#include <boost/thread.hpp>

#include <string>
#include <set>
#include <queue>

class OutgoingMessage
{
public:
  OutgoingMessage(std::string _topic_name, topic_tools::ShapeShifter::ConstPtr _msg, ros::Time _time) :
    topic_name(_topic_name), msg(_msg), time(_time) {}

  std::string topic_name;
  topic_tools::ShapeShifter::ConstPtr msg;
  ros::Time time;
};



//! Global recorder instance so we can record from callback
ros::record::Recorder g_recorder;

//! Global verbose flag so we can easily use from callback
bool g_verbose = false;

//! Global variable including the set of currenly recording topics
std::set<std::string> g_currently_recording;

//! Global variable sued for initialization of counting messages
int g_count = -1;

//! Global queue for storing 
std::queue<OutgoingMessage> g_queue;

//! Mutex for global queue
boost::mutex g_queue_mutex;
boost::condition_variable_any g_queue_condition;

//! Helper function to print executable usage
void print_usage() {
  fprintf (stderr, "USAGE: rosrecord [options] TOPIC1 [TOPIC2 TOPIC3...]\n"
                   "  rosrecord logs ROS message data to a file.\n"
                   );
}


//! Helper function to print executable options
void print_help() {
  print_usage();
  fprintf(stderr, "Options:\n");
  fprintf(stderr, " -c <num>    : Only receive <num> messages on each topic\n");
  fprintf(stderr, " -f <prefix> : Prepend file prefix to beginning of bag name (name will always end with date stamp)\n");
  fprintf(stderr, " -F <fname>  : Record to a file named exactly <fname>.bag\n");
  fprintf(stderr, " -a          : Record all published messages.\n");
  fprintf(stderr, " -v          : Display a message every time a message is received on a topic\n");
  fprintf(stderr, " -h          : Display this help message\n");
}


//! Callback to be invoked to actually do the recording
void do_queue(topic_tools::ShapeShifter::ConstPtr msg,
               std::string topic_name,
               boost::shared_ptr<ros::Subscriber> subscriber,
               boost::shared_ptr<int> count)
{
  if (g_verbose)
    std::cout << "Received message on topic " << subscriber->getTopic() << std::endl;

  // Actually record the message to file
  //  g_recorder.record(topic_name, msg, ros::Time::now());
  OutgoingMessage out(topic_name, msg, ros::Time::now());

  {
    boost::mutex::scoped_lock lock(g_queue_mutex);
    g_queue.push(out);
  }
  g_queue_condition.notify_all();

  // If we are book-keeping count, decrement and possibly shutdown
  if ((*count) > 0)
  {
    (*count)--;
    if ((*count) == 0)
      subscriber->shutdown();
  }
}


//! Thread that actually does writing to file.
// TODO: Do some checking here to make sure our queue isn't getting too big.
void do_record()
{
  ros::NodeHandle nh;
  
  // Technically the g_queue_mutex should be locked while checking empty
  // Except it should only get checked if the node is not ok, and thus
  // it shouldn't be in contention.
  while (nh.ok() || !g_queue.empty())
  {
    boost::unique_lock<boost::mutex> lock(g_queue_mutex);
    while(g_queue.empty())
    {
      if (!nh.ok())
        return;
      g_queue_condition.wait(lock);
    }

    OutgoingMessage out = g_queue.front();
    g_queue.pop();

    lock.release()->unlock();

    g_recorder.record(out.topic_name, out.msg, out.time);
  }
}



void do_check_master(const ros::TimerEvent& e, ros::NodeHandle& node_handle)
{
  ros::master::V_TopicInfo all_topics;

  if (ros::master::getTopics(all_topics))
  {
    // For each topic
    for (ros::master::V_TopicInfo::iterator topic_iter = all_topics.begin();
         topic_iter != all_topics.end(); 
         topic_iter++)
    {
      // If we're not currently recording it, do so
      if (g_currently_recording.find(topic_iter->name) == g_currently_recording.end())
      {
        boost::shared_ptr<int> count(new int(g_count));
        boost::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
        *sub = node_handle.subscribe<topic_tools::ShapeShifter>(topic_iter->name, 100, boost::bind(&do_queue, _1, topic_iter->name, sub, count));
        g_currently_recording.insert(topic_iter->name);
      }
    }
  }
}


void do_check_subscribers_left(const ros::TimerEvent& e)
{
  ros::V_string subscribed_topics;
  ros::this_node::getSubscribedTopics(subscribed_topics);

  // If there is only 1 topic left (time), we can shutdown
  if (subscribed_topics.size() == 1)
    ros::shutdown();
}


//! Main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosrecord", ros::init_options::AnonymousName);

  // Variables
  bool check_master    = false; // Whether master should be checked periodically
  int  exit_code       = 0;     // Eventual exit code
  char name[1024]      = "";    // Name
  char prefix[1024]    = "";    // Prefix
  char tgt_fname[2048] = "";    // Eventual Target Filename
  char fname[2048]     = "";    // Actual Filename (has .active appended until end)

  // TODO: Change this to a ros-centric walltime call?
  time_t t = ::time(NULL);
  struct tm *tms = localtime(&t);
  snprintf(name, sizeof(name), "%d-%02d-%02d-%02d-%02d-%02d-topic.bag",
           tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
           tms->tm_hour     , tms->tm_min  , tms->tm_sec);


  // Parse options  
  int option_char;

  while ((option_char = getopt(argc,argv,"f:F:c:ahv")) != -1)
  {
    switch (option_char)
    {
    case 'f': strncpy(prefix,optarg,sizeof(prefix)); break;
    case 'F': strncpy(tgt_fname,optarg,sizeof(tgt_fname)); break;
    case 'c': g_count = atoi(optarg); break;
    case 'a': check_master = true; break;
    case 'v': g_verbose = true; break;
    case 'h': print_help(); return 1;
    case '?': print_usage(); return 1;
    }
  }
  
  // Create target name and actual (.active) name
  if(strlen(tgt_fname) == 0)
  {
    if (strlen(prefix) == 0)
      strcpy(tgt_fname, name);
    else
      snprintf(tgt_fname, sizeof(tgt_fname), "%s_%s", prefix, name);
  }
  else
    strncpy(tgt_fname+strlen(tgt_fname),".bag",
            sizeof(tgt_fname)-strlen(tgt_fname)-1);
  
  snprintf(fname, sizeof(fname), "%s.active", tgt_fname);
  
  // Logic to make sure count is not specified with automatic topic
  // subscription (implied by no listed topics)
  if ((argc - optind) < 1)
  {
    if (g_count > 0)
    {
      fprintf(stderr, "Specifing a count is not valid with automatic topic subscription.\n");
      return 1;
    }
    if (!check_master)
    {
      ROS_WARN("Running rosrecord with no arguments has been deprecated.  Please use 'rosrecord -a' instead\n");
      check_master = true;
    }
  }

  // Get a node_handle
  ros::NodeHandle node_handle;

  // Only set up recording if we actually got a useful nodehandle.
  if (node_handle.ok())
  {
  // Open our recorder and add available topics
  if (g_recorder.open(std::string(fname)))
  {
    // Every non-processed argument is assumed to be a topic
    for (int i = optind; i < argc; i++)
    {
      boost::shared_ptr<int> count(new int(g_count));
      boost::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
      *sub = node_handle.subscribe<topic_tools::ShapeShifter>(argv[i], 100, boost::bind(&do_queue, _1, argv[i], sub, count));
    }
  } else {
    exit_code = 1;
    ros::shutdown();
  }

  ros::Timer check_master_timer;
  if (check_master)
    check_master_timer = node_handle.createTimer(ros::Duration(1.0), boost::bind(&do_check_master, _1, boost::ref(node_handle)));

  ros::Timer check_subscribers_left_timer;
  if (g_count >= 0)
    check_subscribers_left_timer = node_handle.createTimer(ros::Duration(0.1), &do_check_subscribers_left);

  // Spin up a thread for actually writing to file
  boost::thread record_thread(&do_record);

  ros::MultiThreadedSpinner s(10);
  ros::spin(s);

  g_queue_condition.notify_all();

  record_thread.join();

  // Close the file nicely
  g_recorder.close();

  // Rename the file to the actual target name
  rename(fname,tgt_fname);
  }
  
  // Return our exit code
  return exit_code;
}
