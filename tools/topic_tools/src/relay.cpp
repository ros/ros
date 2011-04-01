///////////////////////////////////////////////////////////////////////////////
// relay just passes messages on. it can be useful if you're trying to ensure
// that a message doesn't get sent twice over a wireless link, by having the 
// relay catch the message and then do the fanout on the far side of the 
// wireless link.
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


#include <cstdio>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using namespace topic_tools;

ros::NodeHandle *g_node = NULL;
bool g_advertised = false;
string g_input_topic;
string g_output_topic;
ros::Publisher g_pub;
ros::Subscriber* g_sub;
bool g_lazy;
ros::TransportHints g_th;

void conn_cb(const ros::SingleSubscriberPublisher&);
void in_cb(const boost::shared_ptr<ShapeShifter const>& msg);

void conn_cb(const ros::SingleSubscriberPublisher&)
{
  // If we're in lazy subscribe mode, and the first subscriber just
  // connected, then subscribe, #3389.
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
    // If the input topic is latched, make the output topic latched, #3385.
    bool latch = false;
    ros::M_string::iterator it = msg->__connection_header->find("latching");
    if((it != msg->__connection_header->end()) && (it->second == "1"))
    {
      ROS_DEBUG("input topic is latched; latching output topic to match");
      latch = true;
    }
    g_pub = msg->advertise(*g_node, g_output_topic, 10, latch, conn_cb);
    g_advertised = true;
    ROS_INFO("advertised as %s\n", g_output_topic.c_str());
  }
  // If we're in lazy subscribe mode, and nobody's listening, 
  // then unsubscribe, #3389.
  if(g_lazy && !g_pub.getNumSubscribers())
  {
    ROS_DEBUG("lazy mode; unsubscribing");
    delete g_sub;
    g_sub = NULL;
  }
  else
    g_pub.publish(msg);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("\nusage: relay IN_TOPIC [OUT_TOPIC]\n\n");
    return 1;
  }
  std::string topic_name;
  if(!getBaseName(string(argv[1]), topic_name))
    return 1;
  ros::init(argc, argv, topic_name + string("_relay"),
            ros::init_options::AnonymousName);
  if (argc == 2)
    g_output_topic = string(argv[1]) + string("_relay");
  else // argc == 3
    g_output_topic = string(argv[2]);
  g_input_topic = string(argv[1]);
  ros::NodeHandle n;
  g_node = &n;
  
  ros::NodeHandle pnh("~");
  bool unreliable = false;
  pnh.getParam("unreliable", unreliable);
  pnh.getParam("lazy", g_lazy);
  if (unreliable)
    g_th.unreliable().reliable(); // Prefers unreliable, but will accept reliable.

  g_sub = new ros::Subscriber(n.subscribe<ShapeShifter>(g_input_topic, 10, &in_cb, g_th));
  ros::spin();
  return 0;
}

