///////////////////////////////////////////////////////////////////////////////
// roscpp_tutorials package shows the features of the libros c++ client library
//
// Copyright (C) 2008, Morgan Quigley
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
//////////////////////////////////////////////////////////////////////////////

#include "ros/node.h"
#include "std_msgs/String.h"

class Listener
{
public:
  std_msgs::String msg1_, msg2_, msg3_;

  void chatter1() { ROS_INFO("chatter1: [%s]", msg1_.data.c_str()); }
  void chatter2() { ROS_INFO("chatter2: [%s]", msg2_.data.c_str()); }
  void chatter3() { ROS_INFO("chatter3: [%s]", msg3_.data.c_str()); }
};

std_msgs::String g_msg4;
void chatter4()
{
  ROS_INFO("chatter4: [%s]", g_msg4.data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  Listener l;
  ros::Node n("listener");
  n.subscribe("chatter", l.msg1_, &Listener::chatter1, &l, 10);
  sleep(1);
  n.subscribe("chatter", l.msg2_, &Listener::chatter2, &l, 10);
  sleep(1);
  n.subscribe("chatter", l.msg3_, &Listener::chatter3, &l, 10);
  sleep(1);
  n.subscribe("chatter", g_msg4, chatter4, 10);
  sleep(1);



  n.unsubscribe("chatter", chatter4);
  sleep(1);
  n.unsubscribe("chatter", &Listener::chatter2, &l);
  sleep(1);
  n.unsubscribe("chatter", &Listener::chatter1, &l);
  sleep(1);
  n.unsubscribe("chatter", &Listener::chatter3, &l);
  sleep(1);
  
  return 0;
}

