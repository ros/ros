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
  std_msgs::String msg_;
  ros::Node* node_;

public:

  Listener(ros::Node* node)
  : node_(node)
  {
  }

  void init()
  {
    node_->subscribe("chatter1", msg_, &Listener::chatterCallback, this, (void *)1, 1000);
    node_->subscribe("chatter2", msg_, &Listener::chatterCallback, this, (void *)2, 1000);
    node_->subscribe("chatter3", msg_, &Listener::chatterCallback, this, (void *)3, 1000);
  }

  void chatterCallback(void *userdata)
  {
    int const ud((size_t)userdata & (size_t)0xffffffff); // for 64bit architectures
    ROS_INFO("I heard: [%s] with userdata [%d]\n", msg_.data.c_str(), ud);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("listener_with_userdata");

  Listener l(&n);
  l.init();

  n.spin();
  
  return 0;
}
