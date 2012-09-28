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

#include "ros/ros.h"
#include <roscpp/TestArray.h>

#define USAGE "USAGE: publish_n_fast <count> <min_size> <max_size>"

void connectCallback(const ros::SingleSubscriberPublisher &pub, int msg_count, int min_size, int max_size)
{
  roscpp::TestArray msg;
  for(int i = 0; i < msg_count; i++)
  {
    msg.counter = i;
    int j = min_size + (int) ((max_size - min_size) * (rand() / (RAND_MAX + 1.0)));
    msg.float_arr.resize(j);
    ROS_INFO("published message %d (%d bytes)\n",
             msg.counter, ros::serialization::Serializer<roscpp::TestArray>::serializedLength(msg));
    pub.publish(msg);
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_n_fast");
  ros::NodeHandle n;

  if(argc != 4)
  {
    puts(USAGE);
    exit(-1);
  }

  int msg_count = atoi(argv[1]);
  int min_size = atoi(argv[2]);
  int max_size = atoi(argv[3]);

  ros::Publisher pub_ = n.advertise<roscpp::TestArray>("roscpp/pubsub_test", msg_count, boost::bind(&connectCallback, _1, msg_count, min_size, max_size));
  ros::spin();

  return 0;
}
