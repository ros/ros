/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

/*
 * Publish an empty message N times, back to back
 */

#include <string>
#include <cstdio>
#include <time.h>
#include <stdlib.h>

#include "ros/node.h"
#include <test_roscpp/TestEmpty.h>

class Publisher : public ros::Node
{
  public:
    Publisher(std::string n, int _msg_count, uint32_t options) :
            ros::Node(n, options), msg_count(_msg_count)  {}
    void sub_cb(const ros::SingleSubscriberPublisher&)
    {
      test_roscpp::TestEmpty msg;
      for(int i=0; i<msg_count; i++)
      {
        publish("test_roscpp/pubsub_test", msg);
        printf("published message %d\n", i);
      }
    }
    int msg_count;
};

#define USAGE "USAGE: publish_n_fast <count>"

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  if(argc != 2)
  {
    puts(USAGE);
    exit(-1);
  }

  int msg_count = atoi(argv[1]);

  Publisher* p;
  p = new Publisher("publisher",msg_count,0);


  test_roscpp::TestEmpty msg;
  p->advertise("test_roscpp/pubsub_test", msg, &Publisher::sub_cb, msg_count);

  // Loop until Ctrl-C is given (by rostest)
  p->spin();


  delete p;
}
