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

#include "perf_roscpp/intra.h"

#include <ros/ros.h>

#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>

using namespace perf_roscpp;
using namespace std;

typedef std::vector<intra::ThroughputResult> V_ThroughputResult;
typedef std::vector<intra::LatencyResult> V_LatencyResult;
typedef std::vector<intra::STLatencyResult> V_STLatencyResult;

void printResult(std::ostream& out, uint32_t test_num, intra::ThroughputResult& r)
{

  out << "----------------------------------------------------------\n";
  out << "Throughput Test " << test_num << ": receiver_threads [" << r.receiver_threads << "], sender_threads [" << r.sender_threads << "], streams [" << r.streams << "], test_duration [" << r.test_duration << "], message_size [" << r.message_size << "]\n";
  out << "\tMessages Sent: " << r.messages_sent << endl;
  out << "\tMessages Received: " << r.messages_received << " (" << (double)r.messages_received / (double)r.messages_sent * 100.0 << "%)" << endl;
  out << "\tBytes Sent: " << r.total_bytes_sent << endl;
  out << "\tBytes Received: " << r.total_bytes_received << endl;
  out << "\tBytes Per Second: " << r.bytes_per_second << " (" << r.bytes_per_second / (1024.0 * 1024.0) << " MB/s)" << endl;
}

void printResult(std::ostream& out, uint32_t test_num, intra::LatencyResult& r)
{
  out << "----------------------------------------------------------\n";
  out << "Multi-Threaded Latency Test " << test_num << ": receiver_threads [" << r.receiver_threads << "], sender_threads [" << r.sender_threads << "], streams [" << r.streams << "], count_per_stream [" << r.count_per_stream << "], message_size [" << r.message_size << "]\n";
  out << "\tMessage Count: " << r.total_message_count << endl;
  out << "\tLatency Average: " << r.latency_avg << endl;
  out << "\tLatency Min: " << r.latency_min << endl;
  out << "\tLatency Max: " << r.latency_max << endl;
}

void printResult(std::ostream& out, uint32_t test_num, intra::STLatencyResult& r)
{
  out << "----------------------------------------------------------\n";
  out << "Single-Threaded Latency Test " << test_num << endl;
  out << "\tMessage Count: " << r.total_message_count << endl;
  out << "\tLatency Average: " << r.latency_avg << endl;
  out << "\tLatency Min: " << r.latency_min << endl;
  out << "\tLatency Max: " << r.latency_max << endl;
}

void addResult(V_ThroughputResult& results, intra::ThroughputResult r, std::ostream& out, uint32_t i)
{
  results.push_back(r);
  printResult(out, i, results.back());
}

void addResult(V_LatencyResult& results, intra::LatencyResult r, std::ostream& out, uint32_t i)
{
  results.push_back(r);
  printResult(out, i, results.back());
}

void addResult(V_STLatencyResult& results, intra::STLatencyResult r, std::ostream& out, uint32_t i)
{
  results.push_back(r);
  printResult(out, i, results.back());
}

void runThroughputTests(std::ostream& out, V_ThroughputResult& results)
{
  uint32_t i = 0;
  //                                   test duration, streams, message size , send threads, receive threads
  addResult(results, intra::throughput(1            , 1      , 100          , 1           , 1              ), out, i++);
  addResult(results, intra::throughput(1            , 1      , 1024*1024*10 , 1           , 1              ), out, i++);
  addResult(results, intra::throughput(1            , 1      , 1024*1024*100, 1           , 1              ), out, i++);

  addResult(results, intra::throughput(10           , 1      , 100          , 1           , 1              ), out, i++);
  addResult(results, intra::throughput(10           , 1      , 1024*1024*10 , 1           , 1              ), out, i++);
  addResult(results, intra::throughput(10           , 1      , 1024*1024*100, 1           , 1              ), out, i++);

#if 0
  addResult(results, intra::throughput(10           , 1      , 100          , 1           , 10             ), out, i++);
  addResult(results, intra::throughput(10           , 1      , 1024*1024*10 , 1           , 10             ), out, i++);
  addResult(results, intra::throughput(10           , 1      , 1024*1024*100, 1           , 10             ), out, i++);

  addResult(results, intra::throughput(1            , 10     , 100          , 1           , 1              ), out, i++);
  addResult(results, intra::throughput(1            , 10     , 1024*1024*1  , 1           , 1              ), out, i++);
  addResult(results, intra::throughput(1            , 10     , 1024*1024*10 , 1           , 1              ), out, i++);

  addResult(results, intra::throughput(10           , 10     , 100          , 1           , 10             ), out, i++);
  addResult(results, intra::throughput(10           , 10     , 1024*1024*1  , 1           , 10             ), out, i++);
  addResult(results, intra::throughput(10           , 10     , 1024*1024*10 , 1           , 10             ), out, i++);
#endif
}

void runLatencyTests(std::ostream& out, V_LatencyResult& results)
{
  uint32_t i = 0;
  //                                count per stream, streams, message size , receive threads
  addResult(results, intra::latency(100000          , 1      , 1            , 1           , 1              ), out, i++);
  addResult(results, intra::latency(10000           , 1      , 1024         , 1           , 1              ), out, i++);
  addResult(results, intra::latency(1000            , 1      , 1024*1024    , 1           , 1              ), out, i++);
  addResult(results, intra::latency(100             , 1      , 1024*1024*100, 1           , 1              ), out, i++);

#if 0
  addResult(results, intra::latency(100000          , 1      , 1            , 1           , 10             ), out, i++);
  addResult(results, intra::latency(10000           , 1      , 1024         , 1           , 10             ), out, i++);
  addResult(results, intra::latency(1000            , 1      , 1024*1024    , 1           , 10             ), out, i++);
  addResult(results, intra::latency(100             , 1      , 1024*1024*100, 1           , 10             ), out, i++);

  addResult(results, intra::latency(100000          , 10     , 1            , 1           , 1              ), out, i++);
  addResult(results, intra::latency(10000           , 10     , 1024         , 1           , 1              ), out, i++);
  addResult(results, intra::latency(1000            , 10     , 1024*1024    , 1           , 1              ), out, i++);
  addResult(results, intra::latency(100             , 10     , 1024*1024*100, 1           , 1              ), out, i++);

  addResult(results, intra::latency(10000           , 10     , 1            , 10          , 1              ), out, i++);
  addResult(results, intra::latency(1000            , 10     , 1024         , 10          , 1              ), out, i++);
  addResult(results, intra::latency(100             , 10     , 1024*1024    , 10          , 1              ), out, i++);
  // 100mb test allocates too much memory
#endif
}

void runSTLatencyTests(std::ostream& out, V_STLatencyResult& results)
{
  uint32_t i = 0;
  addResult(results, intra::stlatency(10000), out, i++);
  addResult(results, intra::stlatency(100000), out, i++);
  addResult(results, intra::stlatency(1000000), out, i++);
}

int main(int argc, char** argv)
{
  std::ofstream out("intra_suite_out.txt", std::ios::out);
  out << std::fixed;
  out.precision(10);
  cout << std::fixed;
  cout.precision(10);

  ROS_ASSERT(out.is_open());

  ros::init(argc, argv, "perf_roscpp_intra_suite", ros::init_options::NoSigintHandler|ros::init_options::NoRosout);
  ros::NodeHandle nh;

  V_ThroughputResult throughput_results;
  runThroughputTests(out, throughput_results);

  V_LatencyResult latency_results;
  runLatencyTests(out, latency_results);

  V_STLatencyResult stlatency_results;
  runSTLatencyTests(out, stlatency_results);

  printf("\n\n\n***************************** Results *****************************\n\n");
  uint32_t i = 0;
  {
    V_ThroughputResult::iterator it = throughput_results.begin();
    V_ThroughputResult::iterator end = throughput_results.end();
    for (; it != end; ++it, ++i)
    {
      intra::ThroughputResult& r = *it;
      printResult(cout, i, r);
    }
  }

  i = 0;
  {
    V_LatencyResult::iterator it = latency_results.begin();
    V_LatencyResult::iterator end = latency_results.end();
    for (; it != end; ++it, ++i)
    {
      intra::LatencyResult& r = *it;
      printResult(cout, i, r);
    }
  }

  i = 0;
  {
    V_STLatencyResult::iterator it = stlatency_results.begin();
    V_STLatencyResult::iterator end = stlatency_results.end();
    for (; it != end; ++it, ++i)
    {
      intra::STLatencyResult& r = *it;
      printResult(cout, i, r);
    }
  }
}
