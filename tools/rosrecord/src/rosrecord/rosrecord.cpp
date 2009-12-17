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

#include <sstream>
#include <string>
#include <set>
#include <queue>

#include "std_msgs/Empty.h"

class OutgoingMessage
{
public:
  OutgoingMessage(std::string _topic_name, topic_tools::ShapeShifter::ConstPtr _msg, ros::Time _time) :
    topic_name(_topic_name), msg(_msg), time(_time) {}

  std::string topic_name;
  topic_tools::ShapeShifter::ConstPtr msg;
  ros::Time time;
};

class OutgoingQueue
{
public:
  OutgoingQueue(std::string _fname, std::queue<OutgoingMessage>* _queue, ros::Time _time) :
    fname(_fname), queue(_queue), time(_time) {}

  std::string fname;
  std::queue<OutgoingMessage>* queue;
  ros::Time time;
};


//! Global verbose flag so we can easily use from callback
bool g_verbose = false;

//! Global snapshot flag so we can easily use from callback
bool g_snapshot = false;

// Global Eventual exit code
int  g_exit_code       = 0;

//! Global variable including the set of currenly recording topics
std::set<std::string> g_currently_recording;

//! Global variable used for initialization of counting messages
int g_count = -1;

//! Global variable used for book-keeping of our number of subscribers
int g_num_subscribers = 0;

//! Global queue for storing 
std::queue<OutgoingMessage>* g_queue;

//! Global queue size
uint64_t g_queue_size=0;

//! Global max queue size
uint64_t g_max_queue_size=1048576*256;

//! Queue of queues to be used by the snapshot recorders
std::queue<OutgoingQueue> g_queue_queue;

//! Mutex for global queue
boost::mutex g_queue_mutex;

//! Conditional variable for global queue
boost::condition_variable_any g_queue_condition;

//! Compression mode
static enum {
  COMPRESSION_NONE,
  COMPRESSION_GZIP,
  COMPRESSION_BZIP2
} g_compression = COMPRESSION_NONE;

template <class T>
std::string time_to_str(T ros_t)
{
  char buf[1024]      = "";
  time_t t = ros_t.sec;
  struct tm *tms = localtime(&t);
  strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
  return std::string(buf);
}

//! Helper function to print executable usage
void print_usage() {
  fprintf (stderr, "Usage: rosrecord [options] TOPIC1 [TOPIC2 TOPIC3...]\n"
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
  fprintf(stderr, " -m          : Maximize internal buffer size in MB (Default: 256MB)  0 = infinite.\n");
  fprintf(stderr, " -z          : Compress the messages with gzip\n");
  fprintf(stderr, " -j          : Compress the messages with bzip2\n");
  fprintf(stderr, " -s          : (EXPERIMENTAL) Enable snapshot recording (don't write to file unless triggered)\n");
  fprintf(stderr, " -t          : (EXPERIMENTAL) Trigger snapshot recording\n");
  fprintf(stderr, " -h          : Display this help message\n");
}


//! Callback to be invoked to save messages into a queue
void do_queue(topic_tools::ShapeShifter::ConstPtr msg,
               std::string topic_name,
               boost::shared_ptr<ros::Subscriber> subscriber,
               boost::shared_ptr<int> count)
{
  ros::Time rectime = ros::Time::now();

  if (g_verbose)
    std::cout << "Received message on topic " << subscriber->getTopic() << std::endl;

  OutgoingMessage out(topic_name, msg, rectime);

  {
    boost::mutex::scoped_lock lock(g_queue_mutex);
    g_queue->push(out);
    g_queue_size += out.msg->msgBufUsed;

    while (g_max_queue_size > 0 && g_queue_size > g_max_queue_size)
    {
      OutgoingMessage drop = g_queue->front();
      g_queue->pop();
      g_queue_size -= drop.msg->msgBufUsed;
      if (!g_snapshot)
      {
        static ros::Time last = ros::Time();
        ros::Time now = ros::Time::now();
        if (now > last + ros::Duration(5.0))
        {
          ROS_WARN("Rosrecord buffer exceeded.  Dropping oldest queued message.");
          last = now;
        }
      }
    }
  }
  
  if (!g_snapshot)
    g_queue_condition.notify_all();

  // If we are book-keeping count, decrement and possibly shutdown
  if ((*count) > 0)
  {
    (*count)--;
    if ((*count) == 0)
    {
      subscriber->shutdown();
      
      if (--g_num_subscribers == 0)
        ros::shutdown();
    }
  }
}

//! Callback to be invoked to actually do the recording
void snapshot_trigger(std_msgs::Empty::ConstPtr trigger, std::string prefix, bool add_date)
{
  ros::WallTime rectime = ros::WallTime::now();

  std::vector<std::string> join;

  if (prefix.length() > 0)
    join.push_back(prefix);
  if (add_date)
    join.push_back(time_to_str(rectime));

  std::string tgt_fname = join[0];
  for (size_t i = 1; i < join.size(); i++)
    tgt_fname = tgt_fname + "_" + join[i];

  tgt_fname = tgt_fname + std::string(".bag");


  ROS_INFO("Triggered snapshot recording with name %s.", tgt_fname.c_str());

  {
    boost::mutex::scoped_lock lock(g_queue_mutex);

    OutgoingQueue out(tgt_fname, g_queue, ros::Time::now());

    g_queue = new std::queue<OutgoingMessage>;
    g_queue_size=0;

    g_queue_queue.push(out);
  }
  g_queue_condition.notify_all();
}

//! Thread that actually does writing to file.
void do_record(std::string prefix, bool add_date)
{
  ros::WallTime rectime = ros::WallTime::now();

  std::vector<std::string> join;

  if (prefix.length() > 0)
    join.push_back(prefix);
  if (add_date)
    join.push_back(time_to_str(rectime));

  std::string tgt_fname = join[0];
  for (size_t i = 1; i < join.size(); i++)
    tgt_fname = tgt_fname + "_" + join[i];

  std::string suffix;
  switch (g_compression) {
  case COMPRESSION_GZIP:
    suffix = ".gz";
    break;
  case COMPRESSION_BZIP2:
    suffix = ".bz2";
    break;
  case COMPRESSION_NONE:
    break;
  default:
    ROS_FATAL("Unknown compression method requested: %d", g_compression);
    g_exit_code = 1;
    ros::shutdown();
    break;
  }

  tgt_fname = tgt_fname + std::string(".bag") + suffix;
  std::string fname = tgt_fname + std::string(".active") + suffix;

  ROS_INFO("Recording to %s.", tgt_fname.c_str());

  ros::NodeHandle nh;
  ros::record::Recorder recorder;

  // Open our recorder and add available topics
  if (!recorder.open(std::string(fname)))
  {
    ROS_FATAL("Could not open output file: %s", fname.c_str());
    g_exit_code = 1;
    ros::shutdown();
  }
  
  recorder.writeVersion();
  recorder.writeFileHeader();

  // Technically the g_queue_mutex should be locked while checking empty
  // Except it should only get checked if the node is not ok, and thus
  // it shouldn't be in contention.
  while (nh.ok() || !g_queue->empty())
  {
    boost::unique_lock<boost::mutex> lock(g_queue_mutex);

    bool finished = false;
    while(g_queue->empty())
    {
      if (!nh.ok())
      {
        lock.release()->unlock();
        finished = true;
        break;
      }
      g_queue_condition.wait(lock);
    }
    if (finished)
      break;

    OutgoingMessage out = g_queue->front();
    g_queue->pop();
    g_queue_size -= out.msg->msgBufUsed;

    lock.release()->unlock();

    recorder.record(out.topic_name, out.msg, out.time);
  }

  // Write the index to a file
  recorder.writeIndex();

  // Close the file nicely
  recorder.close();
  rename(fname.c_str(),tgt_fname.c_str());
}

void do_record_bb()
{
  ros::NodeHandle nh;
  
  while (nh.ok() || !g_queue_queue.empty())
  {
    boost::unique_lock<boost::mutex> lock(g_queue_mutex);
    while(g_queue_queue.empty())
    {
      if (!nh.ok())
        return;
      g_queue_condition.wait(lock);
    }

    OutgoingQueue out_queue = g_queue_queue.front();
    g_queue_queue.pop();

    lock.release()->unlock();

    std::string tgt_fname = out_queue.fname;
    std::string fname = tgt_fname + std::string(".active");

    ros::record::Recorder recorder;

    if (recorder.open(fname))
    {
      recorder.writeVersion();

      while (!out_queue.queue->empty())
      {
        OutgoingMessage out = out_queue.queue->front();
        out_queue.queue->pop();
        recorder.record(out.topic_name, out.msg, out.time);
      }

      // Close the file nicely
      recorder.close();

      // Rename the file to the actual target name
      rename(fname.c_str(),tgt_fname.c_str());
    } else {
      ROS_ERROR("Could not open file: %s", out_queue.fname.c_str());
    }
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

void do_trigger()
{
  // Get a node_handle
  ros::NodeHandle node_handle;
  ros::Publisher pub = node_handle.advertise<std_msgs::Empty>("snapshot_trigger", 1, true);
  pub.publish(std_msgs::Empty());
  ros::Timer terminate_timer = node_handle.createTimer(ros::Duration(1.0), boost::bind(&ros::shutdown));
  ros::spin();  
}

//! Main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosrecord", ros::init_options::AnonymousName);

  // Variables
  bool check_master    = false; // Whether master should be checked periodically

  bool add_date = true;;
  std::string prefix("");   // Prefix                                          
  
  // Parse options  
  int option_char;

  while ((option_char = getopt(argc,argv,"f:F:c:m:ajzsthv")) != -1)
  {
    switch (option_char)
    {
    case 'f': prefix = std::string(optarg); break;
    case 'F': prefix = std::string(optarg); add_date = false; break;
    case 'c': g_count = atoi(optarg); break;
    case 'a': check_master = true; break;
    case 's': g_snapshot = true; break;
    case 't': do_trigger(); return 0; break;
    case 'v': g_verbose = true; break;
    case 'z': g_compression = COMPRESSION_GZIP; break;
    case 'j': g_compression = COMPRESSION_BZIP2; break;
    case 'm': 
      {
        int m=0;
        m=atoi(optarg);
        if (m < 0)
        {
          fprintf(stderr, "Buffer size must be 0 or positive.\n");
          return 1;
        }
        g_max_queue_size =  1048576*m;
      }
      break;
    case 'h': print_help(); return 1;
    case '?': print_usage(); return 1;
    }
  }

  if (g_snapshot)
    ROS_WARN("Using snapshot mode in rosrecord is experimental and usage syntax is subject to change");

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

  g_queue = new std::queue<OutgoingMessage>;

  // Get a node_handle
  ros::NodeHandle node_handle;

  // Only set up recording if we actually got a useful nodehandle.
  if (node_handle.ok())
  {
    boost::thread record_thread;

    // Spin up a thread for actually writing to file
    if (!g_snapshot)
      record_thread = boost::thread(boost::bind(&do_record, prefix, add_date));
    else
      record_thread = boost::thread(boost::bind(&do_record_bb));

    ros::Subscriber trigger = node_handle.subscribe<std_msgs::Empty>("snapshot_trigger", 100, boost::bind(&snapshot_trigger, _1, prefix, add_date));

    // Every non-processed argument is assumed to be a topic
    for (int i = optind; i < argc; i++)
    {
      boost::shared_ptr<int> count(new int(g_count));
      boost::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
      g_num_subscribers++;
      *sub = node_handle.subscribe<topic_tools::ShapeShifter>(argv[i], 100, boost::bind(&do_queue, _1, argv[i], sub, count));
    }

    ros::Timer check_master_timer;
    if (check_master)
      check_master_timer = node_handle.createTimer(ros::Duration(1.0), boost::bind(&do_check_master, _1, boost::ref(node_handle)));
    
    ros::MultiThreadedSpinner s(10);
    ros::spin(s);
    
    g_queue_condition.notify_all();
    
    record_thread.join();
        
  }
  
  delete g_queue;

  // Return our exit code
  return g_exit_code;
}
