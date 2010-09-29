///////////////////////////////////////////////////////////////////////////////
// demux is a generic ROS topic demultiplexer: one input topic is fanned out
// to 1 of N output topics. A service and topic subscription are provided
// to select between the outputs.
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

#include "ros/node.h"
#include "std_msgs/String.h"
#include "topic_tools/DemuxSelect.h"
#include "topic_tools/shape_shifter.h"

using std::string;
using std::vector;
using std::map;
using namespace topic_tools;

static std_msgs::String g_sel_msg;
static vector<string> g_output_topics, g_selected;
static bool g_advertised = false;
static ros::Node *g_node = NULL;
static ShapeShifter g_in_msg;

bool sel_srv_cb( topic_tools::DemuxSelect::Request  &req,
                 topic_tools::DemuxSelect::Response &res )
{
  res.prev_topics = g_selected;
  g_selected = req.topics;
  printf("enabling demux on these topics:\n");
  for (size_t i = 0; i < g_selected.size(); ++i)
    printf("  %s\n", g_selected[i].c_str());
  return true;
}

void in_cb()
{
  if (!ShapeShifter::typed)
  {
    printf("reading first message header, setting datatype\n");
    ShapeShifter::datatype = (*(g_in_msg.__connection_header))["type"];
    ShapeShifter::md5 = (*(g_in_msg.__connection_header))["md5sum"];
    ShapeShifter::msg_def = (*(g_in_msg.__connection_header))["message_definition"];
    ShapeShifter::typed = true;
  }
  if (!g_advertised)
  {
    printf("advertising\n");
    for (size_t i = 0; i < g_output_topics.size(); i++)
      g_node->advertise<ShapeShifter>(g_output_topics[i], 10);
    g_advertised = true;
  }
  for (size_t i = 0; i < g_selected.size(); i++)
    g_node->publish(g_selected[i], g_in_msg);
}

int main(int argc, char **argv)
{
  if (argc < 4)
  {
    printf("\nusage: demux IN_TOPIC OUT_TOPIC1 OUT_TOPIC2 [...]\n");
    return 1;
  }
  string in_topic = string(argv[1]);
  ros::init(argc, argv, in_topic + string("_demux"));
  for (int i = 2; i < argc; i++)
    g_output_topics.push_back(argv[i]);
  ros::Node n(in_topic + string("_demux"));
  g_node = &n;
  n.advertiseService(in_topic + string("_demux_select"), sel_srv_cb);
  g_in_msg.topic = in_topic;
  n.subscribe(in_topic, g_in_msg, in_cb, 10);
  g_selected.push_back(g_output_topics[0]); // select the first one to start
  ros::spin();
  return 0;
}

