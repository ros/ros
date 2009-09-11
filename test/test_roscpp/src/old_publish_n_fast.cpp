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
 * Publish a message N times, back to back
 */

#include <string>
#include <cstdio>
#include <time.h>
#include <stdlib.h>

#include "ros/node.h"
#include <test_roscpp/TestArray.h>

class Publisher : public ros::Node
{
  public:
    int msg_count, min_sz, max_sz;

    Publisher(std::string n, int _msg_count, int _min_sz,
              int _max_sz, uint32_t options) :
            ros::Node(n, options),
            msg_count(_msg_count),
            min_sz(_min_sz),
            max_sz(_max_sz) {}

    void sub_cb(const ros::SingleSubscriberPublisher&)
    {
      test_roscpp::TestArray msg;

      for(int i=0; i<msg_count; i++)
      {
        msg.counter=i;
        int j = min_sz + (int) ((max_sz-min_sz) * (rand() / (RAND_MAX + 1.0)));
        msg.set_float_arr_size(j);
        printf("published message %d (%d bytes)\n",
               msg.counter, msg.serializationLength());
        publish("test_roscpp/pubsub_test", msg);
      }
    }
};

#define USAGE "USAGE: publish_n_fast <count> <min_sz> <max_sz>"

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  if(argc != 4)
  {
    puts(USAGE);
    exit(-1);
  }

  int msg_count = atoi(argv[1]);
  int min_sz = atoi(argv[2]);
  int max_sz = atoi(argv[3]);

  Publisher* p;
  p = new Publisher("publisher",msg_count,min_sz,max_sz,0);

  test_roscpp::TestArray msg;
  p->advertise("test_roscpp/pubsub_test", msg, &Publisher::sub_cb, msg_count);

  srand(time(NULL));

  p->spin();

  delete p;
}
