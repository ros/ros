/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PERF_ROSCPP_INTRA_H
#define PERF_ROSCPP_INTRA_H

#include <ros/types.h>
#include <ros/time.h>

namespace perf_roscpp
{
namespace intra
{

struct ThroughputResult
{
  double test_duration;
  uint64_t streams;
  uint64_t message_size;
  uint32_t sender_threads;
  uint32_t receiver_threads;

  uint64_t messages_sent;
  uint64_t messages_received;

  uint64_t total_bytes_sent;
  uint64_t total_bytes_received;
  uint64_t bytes_per_second;

  ros::WallTime test_start;
  ros::WallTime test_end;
};

ThroughputResult throughput(double duration, uint32_t streams, uint32_t message_size, uint32_t sender_threads, uint32_t receiver_threads);

struct LatencyResult
{
  uint64_t count_per_stream;
  uint64_t streams;
  uint64_t message_size;
  uint32_t sender_threads;
  uint32_t receiver_threads;

  uint64_t total_message_count;

  double latency_avg;
  double latency_min;
  double latency_max;

  ros::WallTime test_start;
  ros::WallTime test_end;
};
LatencyResult latency(uint32_t count_per_stream, uint32_t streams, uint32_t message_size, uint32_t sender_threads, uint32_t receiver_threads);

struct STLatencyResult
{
  uint64_t total_message_count;

  double latency_avg;
  double latency_min;
  double latency_max;

  ros::WallTime test_start;
  ros::WallTime test_end;
};
STLatencyResult stlatency(uint32_t message_count);

} // namespace intra
} // namespace perf_roscpp

#endif // PERF_ROSCPP_INTRA_H
