///////////////////////////////////////////////////////////////////////////////
// demux is a generic ROS topic demultiplexer: one input topic is fanned out
// to 1 of N output topics. A service is provided to select between the outputs
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
#include "std_msgs/String.h"
#include "topic_tools/MuxSelect.h"
#include "topic_tools/shape_shifter.h"

using std::string;
using std::vector;
using namespace topic_tools;

static ShapeShifter *g_selected = NULL;
static ros::NodeHandle *g_node = NULL;
static bool g_advertised = false;
static string g_output_topic;
static vector<ShapeShifter *> g_in_msgs;
static ros::Publisher g_pub;

bool sel_srv_cb( topic_tools::MuxSelect::Request  &req,
                 topic_tools::MuxSelect::Response &res )
{
  if (g_selected)
    res.prev_topic = g_selected->topic;
  else
    res.prev_topic = string("");
  // see if it's the magical '__none' topic, in which case we open the circuit
  if (req.topic == string("__none"))
  {
    printf("mux selected to no input.\n");
    g_selected = NULL;
    return true;
  }
  printf("trying to switch mux to %s\n", req.topic.c_str());
  // spin through our vector of inputs and find this guy
  for (size_t i = 0; i < g_in_msgs.size(); i++)
  {
    if (g_in_msgs[i]->topic == req.topic)
    {
      g_selected = g_in_msgs[i];
      printf("mux selected input %d: [%s]\n", i, g_in_msgs[i]->topic.c_str());
      return true;
    }
  }
  return false;
}

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg,
           ShapeShifter* s)
{
  if (!ShapeShifter::typed)
  {
    printf("reading first message header, setting datatype\n");
    ShapeShifter::datatype = (*(msg->__connection_header))["type"];
    ShapeShifter::md5 = (*(msg->__connection_header))["md5sum"];
    ShapeShifter::msg_def = (*(msg->__connection_header))["message_definition"];
    ShapeShifter::typed = true;
  }
  if (!g_advertised)
  {
    printf("advertising\n");
    g_pub = g_node->advertise<ShapeShifter>(g_output_topic, 10);
    g_advertised = true;
  }
  if (s == g_selected)
    g_pub.publish(msg);
}

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("\nusage: mux OUT_TOPIC IN_TOPIC1 [IN_TOPIC2 [...]]\n\n");
    return 1;
  }
  ros::init(argc, argv, string(argv[1]) + string("_mux"),
            ros::init_options::AnonymousName);
  vector<string> topics;
  for (int i = 2; i < argc; i++)
    topics.push_back(argv[i]);
  ros::NodeHandle n;
  g_node = &n;
  g_output_topic = string(argv[1]);
  ros::ServiceServer ss = n.advertiseService(g_output_topic + string("_select"), sel_srv_cb);
  vector<ros::Subscriber> subs;
  for (size_t i = 0; i < topics.size(); i++)
  {
    g_in_msgs.push_back(new ShapeShifter);
    g_in_msgs.back()->topic = topics[i];
    subs.push_back(n.subscribe<ShapeShifter>(topics[i], 10, boost::bind(in_cb, _1, g_in_msgs.back())));
  }
  g_selected = g_in_msgs[0]; // select first topic to start
  ros::spin();
  for (size_t i = 0; i < g_in_msgs.size(); i++)
  {
    subs[i].shutdown();
    delete g_in_msgs[i];
  }
  g_in_msgs.clear();
  return 0;
}

