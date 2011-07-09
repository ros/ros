///////////////////////////////////////////////////////////////////////////////
// throttle will transform a topic to have a limited number of bytes per second
//
// Copyright (C) 2009, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////


// this could be made a lot smarter by trying to analyze and predict the
// message stream density, etc., rather than just being greedy and stuffing
// the output as fast as it can. 

#include <cstdio>
#include <cstdlib>
#include <deque>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using std::deque;
using namespace topic_tools;

// TODO: move all these globals into a reasonable local scope
ros::NodeHandle *g_node = NULL;
uint32_t g_bps = 0; // bytes per second, not bits!
ros::Duration g_period; // minimum inter-message period
double g_window = 1.0; // 1 second window for starters
bool g_advertised = false;
string g_output_topic;
string g_input_topic;
ros::Publisher g_pub;
ros::Subscriber* g_sub;
bool g_use_messages;
ros::Time g_last_time;
bool g_use_wallclock;
bool g_lazy;
ros::TransportHints g_th;

class Sent
{
public:
  double t;
  uint32_t len;
  Sent(double _t, uint32_t _len) : t(_t), len(_len) { }
};
deque<Sent> g_sent;

void conn_cb(const ros::SingleSubscriberPublisher&);
void in_cb(const boost::shared_ptr<ShapeShifter const>& msg);

void conn_cb(const ros::SingleSubscriberPublisher&)
{
  // If we're in lazy subscribe mode, and the first subscriber just
  // connected, then subscribe, #3546
  if(g_lazy && !g_sub)
  {
    ROS_DEBUG("lazy mode; resubscribing");
    g_sub = new ros::Subscriber(g_node->subscribe<ShapeShifter>(g_input_topic, 10, &in_cb, g_th));
  }
}

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg)
{
  if (!g_advertised)
  {
    // If the input topic is latched, make the output topic latched
    bool latch = false;
    ros::M_string::iterator it = msg->__connection_header->find("latching");
    if((it != msg->__connection_header->end()) && (it->second == "1"))
    {
      ROS_DEBUG("input topic is latched; latching output topic to match");
      latch = true;
    }
    g_pub = msg->advertise(*g_node, g_output_topic, 10, latch, conn_cb);
    g_advertised = true;
    printf("advertised as %s\n", g_output_topic.c_str());
  }
  // If we're in lazy subscribe mode, and nobody's listening, 
  // then unsubscribe, #3546.
  if(g_lazy && !g_pub.getNumSubscribers())
  {
    ROS_DEBUG("lazy mode; unsubscribing");
    delete g_sub;
    g_sub = NULL;
  }
  else
  {
    if(g_use_messages)
    {
      ros::Time now;
      if(g_use_wallclock)
        now.fromSec(ros::WallTime::now().toSec());
      else
        now = ros::Time::now();
      if((now - g_last_time) > g_period)
      {
        g_pub.publish(msg);
        g_last_time = now;
      }
    }
    else
    {
      // pop the front of the queue until it's within the window
      ros::Time now;
      if(g_use_wallclock)
        now.fromSec(ros::WallTime::now().toSec());
      else
        now = ros::Time::now();
      const double t = now.toSec();
      while (!g_sent.empty() && g_sent.front().t < t - g_window)
        g_sent.pop_front();
      // sum up how many bytes are in the window
      uint32_t bytes = 0;
      for (deque<Sent>::iterator i = g_sent.begin(); i != g_sent.end(); ++i)
        bytes += i->len;
      if (bytes < g_bps)
      {
        g_pub.publish(msg);
        g_sent.push_back(Sent(t, msg->size()));
      }
    }
  }
}

#define USAGE "\nusage: \n"\
           "  throttle messages IN_TOPIC MSGS_PER_SEC [OUT_TOPIC]]\n"\
           "OR\n"\
           "  throttle bytes IN_TOPIC BYTES_PER_SEC WINDOW [OUT_TOPIC]]\n\n"\
           "  This program will drop messages from IN_TOPIC so that either: the \n"\
           "  average bytes per second on OUT_TOPIC, averaged over WINDOW \n"\
           "  seconds, remains below BYTES_PER_SEC, or: the minimum inter-message\n"\
           "  period is 1/MSGS_PER_SEC. The messages are output \n"\
           "  to OUT_TOPIC, or (if not supplied), to IN_TOPIC_throttle.\n\n"

int main(int argc, char **argv)
{
  if(argc < 3)
  {
    puts(USAGE);
    return 1;
  }

  g_input_topic = string(argv[2]);

  std::string topic_name;
  if(!getBaseName(string(argv[2]), topic_name))
    return 1;

  ros::init(argc, argv, topic_name + string("_throttle"),
            ros::init_options::AnonymousName);
  bool unreliable = false;
  ros::NodeHandle pnh("~");
  pnh.getParam("wall_clock", g_use_wallclock);
  pnh.getParam("unreliable", unreliable);
  pnh.getParam("lazy", g_lazy);

  if (unreliable)
    g_th.unreliable().reliable(); // Prefers unreliable, but will accept reliable.

  if(!strcmp(argv[1], "messages"))
    g_use_messages = true;
  else if(!strcmp(argv[1], "bytes"))
    g_use_messages = false;
  else
  {
    puts(USAGE);
    return 1;
  }

  if(g_use_messages && argc == 5)
    g_output_topic = string(argv[4]);
  else if(!g_use_messages && argc == 6)
    g_output_topic = string(argv[5]);
  else
    g_output_topic = g_input_topic + "_throttle";

  if(g_use_messages)
  {
    if(argc < 4)
    {
      puts(USAGE);
      return 1;
    }
    g_period = ros::Duration(1.0/atof(argv[3]));
  }
  else
  {
    if(argc < 5)
    {
      puts(USAGE);
      return 1;
    }
    g_bps = atoi(argv[3]);
    g_window = atof(argv[4]);
  }

  ros::NodeHandle n;
  g_node = &n;
  g_sub = new ros::Subscriber(n.subscribe<ShapeShifter>(g_input_topic, 10, &in_cb, g_th));
  ros::spin();
  return 0;
}

