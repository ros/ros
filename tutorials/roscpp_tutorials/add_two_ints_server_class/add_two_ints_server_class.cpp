///////////////////////////////////////////////////////////////////////////////
// The roscpp_tutorial package shows off the features of the c++ client library
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
#include "roscpp_tutorials/TwoInts.h"

class AddTwo
{
public:
  void init()
  {
    ros::Node::instance()->advertiseService("add_two_ints", &AddTwo::add, this);
  }

  bool add(roscpp_tutorials::TwoInts::Request  &req,
           roscpp_tutorials::TwoInts::Response &res )
  {
    res.sum = req.a + req.b;
    printf("request: x=%ld, y=%ld\n", (long int)req.a, (long int)req.b);
    printf("  sending back response: [%ld]\n", (long int)res.sum);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("add_two_ints_server");

  AddTwo a;
  a.init();

  n.spin();
  
  return 0;
}

