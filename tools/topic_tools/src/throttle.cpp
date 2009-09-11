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
#include "ros/node.h"
#include "topic_tools/shape_shifter.h"

using std::string;
using std::vector;
using std::deque;
using namespace topic_tools;

// TODO: move all these globals into a reasonable local scope
static ros::NodeHandle *g_node = NULL;
static uint32_t g_bps = 0; // bytes per second, not bits!
static ros::Duration g_period; // minimum inter-message period
static double g_window = 1.0; // 1 second window for starters
static bool g_advertised = false;
static string g_output_topic;
static ros::Publisher g_pub;
static bool g_use_messages;
static ros::Time g_last_time;
//static ShapeShifter g_in_msg;

class Sent
{
public:
  double t;
  uint32_t len;
  Sent(double _t, uint32_t _len) : t(_t), len(_len) { }
};
deque<Sent> g_sent;

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg)
{
  if (!ShapeShifter::typed)
  {
    ShapeShifter::datatype = (*(msg->__connection_header))["type"];
    ShapeShifter::md5 = (*(msg->__connection_header))["md5sum"];
    ShapeShifter::msg_def = (*(msg->__connection_header))["message_definition"];
    ShapeShifter::typed = true;
    printf("read first message header, using datatype %s\n",
           ShapeShifter::datatype.c_str());
  }
  if (!g_advertised)
  {
    g_pub = g_node->advertise<ShapeShifter>(g_output_topic, 10);
    g_advertised = true;
    printf("advertised as %s\n", g_output_topic.c_str());
  }
  if(g_use_messages)
  {
    ros::Time now = ros::Time::now();
    if((now - g_last_time) > g_period)
    {
      g_pub.publish(msg);
      g_last_time = now;
    }
  }
  else
  {
    // pop the front of the queue until it's within the window
    const double t = ros::Time::now().toSec();
    while (!g_sent.empty() && g_sent.front().t < t - g_window)
      g_sent.pop_front();
    // sum up how many bytes are in the window
    uint32_t bytes = 0;
    for (deque<Sent>::iterator i = g_sent.begin(); i != g_sent.end(); ++i)
      bytes += i->len;
    if (bytes < g_bps)
    {
      g_pub.publish(msg);
      g_sent.push_back(Sent(t, msg->__serialized_length));
    }
  }
}

#define USAGE "\nusage: \n"\
           "  throttle_bandwidth messages IN_TOPIC MSGS_PER_SEC [OUT_TOPIC]]\n"\
           "OR\n"\
           "  throttle_bandwidth bytes IN_TOPIC BYTES_PER_SEC WINDOW [OUT_TOPIC]]\n\n"\
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

  if(!strcmp(argv[1], "messages"))
    g_use_messages = true;
  else if(!strcmp(argv[1], "bytes"))
    g_use_messages = false;
  else
  {
    puts(USAGE);
    return 1;
  }
  
  string intopic = string(argv[2]);

  if(g_use_messages && argc == 5)
    g_output_topic = string(argv[4]);
  else if(!g_use_messages && argc == 6)
    g_output_topic = string(argv[5]);
  else
    g_output_topic = intopic + "_throttle";

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
    g_bps = atoi(argv[3]);
    g_window = atof(argv[4]);
  }

  ros::init(argc, argv, intopic + string("_throttle"),
            ros::init_options::AnonymousName);
  ros::NodeHandle n;
  g_node = &n;
  ros::Subscriber sub = n.subscribe<ShapeShifter>(intopic, 10, &in_cb);
  ros::spin();
  return 0;
}

