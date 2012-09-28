/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "roscpp/PointCloud.h"
#include <cstdlib>
#include <cstdio>

using namespace ros::serialization;

ros::WallTime t;

inline void tic()
{
  t = ros::WallTime::now();
}

inline double toc()
{
  return (ros::WallTime::now() - t).toSec();
}

int main(int, char **)
{
  roscpp::PointCloud pc;

  const int NUM_ITER = 100;
  const int NUM_PTS = 1000000;
  pc.pts.resize(NUM_PTS);
  pc.chan.resize(2);
  pc.chan[0].vals.resize(NUM_PTS);
  pc.chan[1].vals.resize(NUM_PTS);

  ros::SerializedMessage m;
  m.num_bytes = serializationLength(pc);
  m.buf.reset(new uint8_t[m.num_bytes]);

  tic();
  for (int i = 0; i < NUM_ITER; ++i)
  {
    OStream s(m.buf.get(), m.num_bytes);
    serialize(s, pc);
    m.message_start = m.buf.get();
  }
  printf("avg serialization took %.6f sec\n", toc() / (double)NUM_ITER);

  tic();
  for (int i = 0; i < NUM_ITER; i++)
  {
    roscpp::PointCloud pc2;
    deserializeMessage(m, pc2);
  }
  printf("avg deserization time %.6f sec\n", toc() / (double)NUM_ITER);

  return 0;
}

