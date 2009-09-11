///////////////////////////////////////////////////////////////////////////////
// The test_roscpp package has a few tests of the roscpp c++ client library 
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

#include <ctime>
#include <cstdlib>
#include "ros/node.h"
#include "std_msgs/String.h"
using namespace ros;

class UnsubscribeTest : public Node
{
public:
  std_msgs::String msg;
  UnsubscribeTest() : ros::Node("ilove2unsubscribe")
  {
  }
  void sign_up()
  {
    printf("signing up for chatter\n");
    subscribe("chatter", msg, &UnsubscribeTest::chatter_cb, 0);
  }
  void cancel_by_name()
  {
    printf("unregistering by name from chatter\n");
    unsubscribe("chatter");
  }
  void cancel_by_msgref()
  {
    printf("unregistering by reference\n");
    unsubscribe(msg);
  }
  void chatter_cb()
  {
    printf("I heard: [%s]\n", msg.data.c_str());
  }
};

int main(int argc, char **argv)
{
  srand(::time(NULL));
  ros::init(argc, argv);
  UnsubscribeTest ut;
  for (int i = 0; i < 10; i++)
  {
    usleep(rand() % 500000 + 100000);
    ut.sign_up();
    usleep(rand() % 500000 + 1000000);
    if (rand() % 2 == 0)
      ut.cancel_by_name();
    else
      ut.cancel_by_msgref();
  }
  
  return 0;
}

