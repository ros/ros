///////////////////////////////////////////////////////////////////////////////
// drop will (intentionally) drop X out of every Y messages that hits it
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

#include <cstdlib>
#include <cstdio>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using namespace topic_tools;

static ros::NodeHandle *g_node = NULL;
static int g_x = 0, g_y = 1;
static bool g_advertised = false;
static string g_output_topic;
static ros::Publisher g_pub;

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg)
{
  static int s_count = 0;
  if (!g_advertised)
  {
    g_pub = msg->advertise(*g_node, g_output_topic, 10);
    g_advertised = true;
    printf("advertised as %s\n", g_output_topic.c_str());
  }
  if (s_count >= g_x)
    g_pub.publish(msg);
  ++s_count;
  if (s_count >= g_y)
    s_count = 0;
}

#define USAGE "\nusage: drop IN_TOPIC X Y [OUT_TOPIC]\n\n" \
              " This program will drop X out of every Y messages from IN_TOPIC,\n" \
              " forwarding the rest to OUT_TOPIC if given, else to a topic \n" \
              " named IN_TOPIC_drop\n\n"
int main(int argc, char **argv)
{
  if(argc < 2)
  {
    puts(USAGE);
    return 1;
  }
  std::string topic_name;
  if(!getBaseName(string(argv[1]), topic_name))
    return 1;
  ros::init(argc, argv, topic_name + string("_drop"),
            ros::init_options::AnonymousName);
  if ((argc != 4 && argc != 5) || atoi(argv[2]) < 0 || atoi(argv[3]) < 1)
  {
    puts(USAGE);
    return 1;
  }
  if (argc == 4)
    g_output_topic = string(argv[1]) + string("_drop");
  else // argc == 5
    g_output_topic = string(argv[4]);
  ros::NodeHandle n;
  g_node = &n;
  g_x = atoi(argv[2]);
  g_y = atoi(argv[3]);
  ros::Subscriber sub = n.subscribe<ShapeShifter>(string(argv[1]), 10, in_cb);
  ros::spin();
  return 0;
}

