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
#include "roscpp/ThroughputMessage.h"
#include "roscpp/LatencyMessage.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <vector>

namespace perf_roscpp
{
namespace intra
{

class ThroughputTest
{
public:
  ThroughputTest(double test_duration, uint32_t streams, uint32_t message_size, uint32_t sender_threads, uint32_t receiver_threads);

  ThroughputResult run();

private:
  void sendThread(boost::barrier* all_connected);
  void receiveThread(boost::barrier* all_started, boost::barrier* all_start, ros::WallTime* end_time);

  void callback(const roscpp::ThroughputMessageConstPtr& msg);

  boost::mutex mutex_;

  struct ReceiveThreadResult
  {
    uint64_t bytes_received;
    uint64_t messages_received;

    ros::WallTime last_recv_time;
  };
  boost::thread_specific_ptr<ReceiveThreadResult> receive_thread_result_;
  std::vector<boost::shared_ptr<ReceiveThreadResult> > receive_results_;

  struct SendThreadResult
  {
    uint64_t bytes_sent;
    uint64_t messages_sent;

    ros::WallTime first_send_time;
  };
  boost::thread_specific_ptr<SendThreadResult> send_thread_result_;
  std::vector<boost::shared_ptr<SendThreadResult> > send_results_;

  ros::CallbackQueue queue_;
  std::vector<ros::Publisher> pubs_;

  boost::thread_group receive_threads_;
  boost::thread_group send_threads_;

  double test_duration_;
  uint32_t streams_;
  uint32_t message_size_;
  uint32_t sender_threads_;
  uint32_t receiver_threads_;
};

ThroughputTest::ThroughputTest(double test_duration, uint32_t streams, uint32_t message_size, uint32_t sender_threads, uint32_t receiver_threads)
: test_duration_(test_duration)
, streams_(streams)
, message_size_(message_size)
, sender_threads_(sender_threads)
, receiver_threads_(receiver_threads)
{
}

void ThroughputTest::callback(const roscpp::ThroughputMessageConstPtr& msg)
{
  ReceiveThreadResult& r = *receive_thread_result_;

  r.bytes_received += ros::serialization::Serializer<roscpp::ThroughputMessage>::serializedLength(*msg) + 4; // 4 byte message length field
  ++r.messages_received;

  r.last_recv_time = ros::WallTime::now();

  //ROS_INFO_STREAM("Received message " << r.messages_received);
}

void ThroughputTest::receiveThread(boost::barrier* all_ready, boost::barrier* all_start, ros::WallTime* end_time)
{
  receive_thread_result_.reset(new ReceiveThreadResult);

  ReceiveThreadResult& r = *receive_thread_result_;
  r.messages_received = 0;
  r.bytes_received = 0;

  ROS_INFO_STREAM("Receive thread [" << boost::this_thread::get_id() << "] waiting for all threads to start");

  all_ready->wait();
  all_start->wait();
  ros::WallTime local_end_time = *end_time;

  ROS_INFO_STREAM("Receive thread [" << boost::this_thread::get_id() << "] running");

  while (ros::WallTime::now() < local_end_time)
  {
    queue_.callOne(ros::WallDuration(0.01));
  }

  ROS_INFO_STREAM("Receive thread [" << boost::this_thread::get_id() << "] adding results and exiting");

  boost::mutex::scoped_lock lock(mutex_);
  receive_results_.push_back(boost::shared_ptr<ReceiveThreadResult>(receive_thread_result_.release()));
}

void ThroughputTest::sendThread(boost::barrier* all_connected)
{
  send_thread_result_.reset(new SendThreadResult);
  SendThreadResult& r = *send_thread_result_;

  ros::NodeHandle nh;
  nh.setCallbackQueue(&queue_);
  std::vector<ros::Publisher> pubs;
  for (uint32_t i = 0; i < streams_; ++i)
  {
    std::stringstream ss;
    ss << "throughput_perf_test_" << i;
    pubs.push_back(nh.advertise<roscpp::ThroughputMessage>(ss.str(), 1));
  }

  // Need to keep around the publishers so the connections don't go away
  {
    boost::mutex::scoped_lock lock(mutex_);
    pubs_.insert(pubs_.end(), pubs.begin(), pubs.end());
  }

  ROS_INFO_STREAM("Publish thread [" << boost::this_thread::get_id() << "] waiting for connections");
  bool cont = true;
  while (cont)
  {
    cont = false;
    for (uint32_t i = 0; i < streams_; ++i)
    {
      if (pubs[i].getNumSubscribers() == 0)
      {
        cont = true;
      }
    }
  }

  roscpp::ThroughputMessagePtr msg(new roscpp::ThroughputMessage);
  msg->array.resize(message_size_);

  all_connected->wait();

  ROS_INFO_STREAM("Publish thread [" << boost::this_thread::get_id() << "] all connections established, beginning to publish");

  r.first_send_time = ros::WallTime::now();
  r.bytes_sent = 0;
  r.messages_sent = 0;

  try
  {
    const uint32_t streams = streams_;
    while (!boost::this_thread::interruption_requested())
    {
      for (uint32_t j = 0; j < streams; ++j)
      {
        pubs[j].publish(msg);

        ++r.messages_sent;
        r.bytes_sent += ros::serialization::Serializer<roscpp::ThroughputMessage>::serializedLength(*msg) + 4;
      }

      boost::this_thread::yield();
    }
  }
  catch (boost::thread_interrupted&)
  {
  }

  ROS_INFO_STREAM("Publish thread [" << boost::this_thread::get_id() << "] exiting");

  boost::mutex::scoped_lock lock(mutex_);
  send_results_.push_back(boost::shared_ptr<SendThreadResult>(send_thread_result_.release()));
}

ThroughputResult ThroughputTest::run()
{
  ROS_INFO("Starting receive threads");
  ThroughputResult r;
  r.test_start = ros::WallTime::now();

  ros::NodeHandle nh;
  nh.setCallbackQueue(&queue_);

  std::vector<ros::Subscriber> subs;
  for (uint32_t i = 0; i < streams_; ++i)
  {
    std::stringstream ss;
    ss << "throughput_perf_test_" << i;
    subs.push_back(nh.subscribe(ss.str(), 0, &ThroughputTest::callback, this, ros::TransportHints().tcpNoDelay()));
  }

  boost::barrier sender_all_connected(sender_threads_ + 1);
  boost::barrier receiver_all_ready(receiver_threads_ + 1);
  boost::barrier receiver_start(receiver_threads_ + 1);
  ros::WallTime test_end_time;

  for (uint32_t i = 0; i < receiver_threads_; ++i)
  {
    receive_threads_.create_thread(boost::bind(&ThroughputTest::receiveThread, this, &receiver_all_ready, &receiver_start, &test_end_time));
  }

  for (uint32_t i = 0; i < sender_threads_; ++i)
  {
    send_threads_.create_thread(boost::bind(&ThroughputTest::sendThread, this, &sender_all_connected));
  }

  receiver_all_ready.wait();
  test_end_time = ros::WallTime::now() + ros::WallDuration(test_duration_);
  receiver_start.wait();

  ros::WallTime pub_start = ros::WallTime::now();
  sender_all_connected.wait();

  receive_threads_.join_all();
  ROS_INFO("All receive threads done");

  send_threads_.interrupt_all();
  send_threads_.join_all();
  ROS_INFO("All publish threads done");

  ROS_INFO("Collating results");

  r.test_end = ros::WallTime::now();

  r.bytes_per_second = 0;
  r.message_size = message_size_;
  r.messages_received = 0;
  r.messages_sent = 0;
  r.receiver_threads = receiver_threads_;
  r.sender_threads = sender_threads_;
  r.total_bytes_received = 0;
  r.total_bytes_sent = 0;
  r.test_duration = test_duration_;
  r.streams = streams_;

  ros::WallTime rec_end;
  {
    std::vector<boost::shared_ptr<ReceiveThreadResult> >::iterator it = receive_results_.begin();
    std::vector<boost::shared_ptr<ReceiveThreadResult> >::iterator end = receive_results_.end();
    for (; it != end; ++it)
    {
      ReceiveThreadResult& rr = **it;
      r.total_bytes_received += rr.bytes_received;
      r.messages_received += rr.messages_received;

      rec_end = std::max(rec_end, rr.last_recv_time);
    }
  }

  {
    std::vector<boost::shared_ptr<SendThreadResult> >::iterator it = send_results_.begin();
    std::vector<boost::shared_ptr<SendThreadResult> >::iterator end = send_results_.end();
    for (; it != end; ++it)
    {
      SendThreadResult& sr = **it;
      r.total_bytes_sent += sr.bytes_sent;
      r.messages_sent += sr.messages_sent;

      pub_start = std::min(pub_start, sr.first_send_time);
    }
  }

  r.bytes_per_second = (double)r.total_bytes_received / (rec_end - pub_start).toSec();

  ROS_INFO("Done collating results");

  return r;
}

ThroughputResult throughput(double test_duration, uint32_t streams, uint32_t message_size, uint32_t sender_threads, uint32_t receiver_threads)
{
  ROS_INFO_STREAM("*****************************************************");
  ROS_INFO_STREAM("Running throughput test: "<< "receiver_threads [" << receiver_threads << "], sender_threads [" << sender_threads << "], streams [" << streams << "], test_duration [" << test_duration << "], message_size [" << message_size << "]");

  ThroughputTest t(test_duration, streams, message_size, sender_threads, receiver_threads);
  return t.run();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class LatencyTest
{
public:
  LatencyTest(uint32_t count_per_stream, uint32_t streams, uint32_t message_size, uint32_t sender_threads, uint32_t receiver_threads);

  LatencyResult run();

private:
  void sendThread(boost::barrier* b, uint32_t i);
  void receiveThread();

  void receiveCallback(const roscpp::LatencyMessageConstPtr& msg, ros::Publisher& pub);
  void sendCallback(const roscpp::LatencyMessageConstPtr& msg, ros::Publisher& pub, uint32_t thread_index);

  boost::mutex mutex_;

  struct ThreadResult
  {
    uint64_t message_count;

    std::vector<double> latencies;
  };
  boost::thread_specific_ptr<ThreadResult> thread_result_;
  std::vector<boost::shared_ptr<ThreadResult> > results_;

  ros::CallbackQueue receive_queue_;

  boost::thread_group send_threads_;

  uint32_t count_per_stream_;
  uint32_t streams_;
  uint32_t message_size_;
  uint32_t sender_threads_;
  uint32_t receiver_threads_;
};

LatencyTest::LatencyTest(uint32_t count_per_stream, uint32_t streams, uint32_t message_size, uint32_t sender_threads, uint32_t receiver_threads)
: count_per_stream_(count_per_stream)
, streams_(streams)
, message_size_(message_size)
, sender_threads_(sender_threads)
, receiver_threads_(receiver_threads)
{
}

void LatencyTest::receiveCallback(const roscpp::LatencyMessageConstPtr& msg, ros::Publisher& pub)
{
  ros::WallTime receipt_time = ros::WallTime::now();
  roscpp::LatencyMessagePtr reply = boost::const_pointer_cast<roscpp::LatencyMessage>(msg);
  reply->receipt_time = receipt_time.toSec();
  pub.publish(reply);
  //ROS_INFO("Receiver received message %d", msg->count);
}

void LatencyTest::sendCallback(const roscpp::LatencyMessageConstPtr& msg, ros::Publisher& pub, uint32_t thread_index)
{
  if (msg->thread_index != thread_index)
  {
    return;
  }

  thread_result_->latencies.push_back(msg->receipt_time - msg->publish_time);
  ++thread_result_->message_count;

  roscpp::LatencyMessagePtr reply = boost::const_pointer_cast<roscpp::LatencyMessage>(msg);
  reply->publish_time = ros::WallTime::now().toSec();
  ++reply->count;

  //ROS_INFO("Sender received return message %d", msg->count);

  if (reply->count < count_per_stream_ * streams_)
  {
    pub.publish(reply);
  }
}

void LatencyTest::sendThread(boost::barrier* all_connected, uint32_t thread_index)
{
  thread_result_.reset(new ThreadResult);
  ThreadResult& r = *thread_result_;

  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  std::vector<ros::Publisher> pubs;
  std::vector<ros::Subscriber> subs;
  pubs.reserve(streams_);
  for (uint32_t i = 0; i < streams_; ++i)
  {
    std::stringstream ss;
    ss << "latency_perf_test_" << i;
    pubs.push_back(nh.advertise<roscpp::LatencyMessage>(ss.str(), 0));

    ss << "_return";
    subs.push_back(nh.subscribe<roscpp::LatencyMessage>(ss.str(), 0, boost::bind(&LatencyTest::sendCallback, this, _1, boost::ref(pubs[i]), thread_index), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
  }

  bool cont = true;
  while (cont)
  {
    cont = false;
    for (uint32_t i = 0; i < streams_; ++i)
    {
      if (pubs[i].getNumSubscribers() == 0)
      {
        cont = true;
      }
    }
  }

  std::vector<roscpp::LatencyMessagePtr> messages;
  for (uint32_t i = 0; i < streams_; ++i)
  {
    roscpp::LatencyMessagePtr msg(new roscpp::LatencyMessage);
    msg->thread_index = thread_index;
    msg->array.resize(message_size_);
    messages.push_back(msg);
  }

  all_connected->wait();

  r.message_count = 0;

  const uint32_t count = count_per_stream_;
  const uint32_t streams = streams_;
  const uint32_t total_messages = count * streams;
  for (uint32_t j = 0; j < streams; ++j)
  {
    messages[j]->publish_time = ros::WallTime::now().toSec();
    pubs[j].publish(messages[j]);
  }

  while (r.latencies.size() < total_messages)
  {
    queue.callAvailable(ros::WallDuration(0.01));
  }

  ROS_INFO_STREAM("Publish thread [" << boost::this_thread::get_id() << "] exiting");

  boost::mutex::scoped_lock lock(mutex_);
  results_.push_back(boost::shared_ptr<ThreadResult>(thread_result_.release()));
}

LatencyResult LatencyTest::run()
{
  ROS_INFO("Starting receive threads");
  LatencyResult r;
  r.test_start = ros::WallTime::now();

  ros::NodeHandle nh;
  nh.setCallbackQueue(&receive_queue_);

  std::vector<ros::Subscriber> subs;
  std::vector<ros::Publisher> pubs;
  pubs.reserve(streams_ * sender_threads_);
  for (uint32_t i = 0; i < streams_; ++i)
  {
    std::stringstream ss;
    ss << "latency_perf_test_" << i;
    std::string sub_topic = ss.str();
    ss << "_return";
    std::string pub_topic = ss.str();
    pubs.push_back(nh.advertise<roscpp::LatencyMessage>(pub_topic, 0));
    subs.push_back(nh.subscribe<roscpp::LatencyMessage>(sub_topic, 0, boost::bind(&LatencyTest::receiveCallback, this, _1, boost::ref(pubs.back())), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
  }

  boost::barrier all_connected(1 + sender_threads_);

  ros::WallTime pub_start = ros::WallTime::now();
  ROS_INFO("Starting publish threads");
  for (uint32_t i = 0; i < sender_threads_; ++i)
  {
    send_threads_.create_thread(boost::bind(&LatencyTest::sendThread, this, &all_connected, i));
  }

  ROS_INFO("Waiting for all connections to establish");

  bool cont = true;
  while (cont)
  {
    cont = false;
    for (uint32_t i = 0; i < streams_; ++i)
    {
      if (pubs[i].getNumSubscribers() == 0)
      {
        cont = true;
      }
    }
  }

  all_connected.wait();
  ROS_INFO("All connections established");

  ros::AsyncSpinner receive_spinner(receiver_threads_, &receive_queue_);
  receive_spinner.start();

  send_threads_.join_all();
  ROS_INFO("All publish threads done");

  ROS_INFO("Collating results");

  r.test_end = ros::WallTime::now();

  r.latency_avg = 0;
  r.latency_max = 0;
  r.latency_min = 9999999999999ULL;
  r.total_message_count = 0;
  r.message_size = message_size_;
  r.receiver_threads = receiver_threads_;
  r.sender_threads = sender_threads_;
  r.count_per_stream = count_per_stream_;
  r.streams = streams_;

  double latency_total = 0.0;
  uint32_t latency_count = 0;
  {
    std::vector<boost::shared_ptr<ThreadResult> >::iterator it = results_.begin();
    std::vector<boost::shared_ptr<ThreadResult> >::iterator end = results_.end();
    for (; it != end; ++it)
    {
      ThreadResult& tr = **it;
      r.total_message_count += tr.message_count;

      std::vector<double>::iterator lat_it = tr.latencies.begin();
      std::vector<double>::iterator lat_end = tr.latencies.end();
      for (; lat_it != lat_end; ++lat_it)
      {
        double latency = *lat_it;
        r.latency_min = std::min(r.latency_min, latency);
        r.latency_max = std::max(r.latency_max, latency);
        ++latency_count;
        latency_total += latency;
      }
    }
  }

  r.latency_avg = latency_total / latency_count;

  ROS_INFO("Done collating results");

  return r;
}

LatencyResult latency(uint32_t count_per_stream, uint32_t streams, uint32_t message_size, uint32_t sender_threads, uint32_t receiver_threads)
{
  ROS_INFO_STREAM("*****************************************************");
  ROS_INFO_STREAM("Running latency test: "<< "receiver_threads [" << receiver_threads << "], sender_threads [" << sender_threads << "], streams [" << streams << "], count_per_stream [" << count_per_stream << "], message_size [" << message_size << "]");

  LatencyTest t(count_per_stream, streams, message_size, sender_threads, receiver_threads);
  return t.run();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class STLatencyTest
{
public:
  STLatencyTest(uint32_t message_count);

  STLatencyResult run();

private:

  void receiveCallback(const roscpp::LatencyMessageConstPtr& msg, ros::Publisher& pub);
  void sendCallback(const roscpp::LatencyMessageConstPtr& msg, ros::Publisher& pub);

  struct Result
  {
    std::vector<double> latencies;
  };
  Result result_;

  ros::CallbackQueue receive_queue_;

  uint32_t message_count_;
};

STLatencyTest::STLatencyTest(uint32_t message_count)
: message_count_(message_count)
{
}

void STLatencyTest::receiveCallback(const roscpp::LatencyMessageConstPtr& msg, ros::Publisher& pub)
{
  ros::WallTime receipt_time = ros::WallTime::now();
  roscpp::LatencyMessagePtr reply = boost::const_pointer_cast<roscpp::LatencyMessage>(msg);
  reply->receipt_time = receipt_time.toSec();
  pub.publish(reply);
  //ROS_INFO("Receiver received message %d", msg->count);
}

void STLatencyTest::sendCallback(const roscpp::LatencyMessageConstPtr& msg, ros::Publisher& pub)
{
  result_.latencies.push_back(msg->receipt_time - msg->publish_time);

  roscpp::LatencyMessagePtr reply = boost::const_pointer_cast<roscpp::LatencyMessage>(msg);
  reply->publish_time = ros::WallTime::now().toSec();
  ++reply->count;

  //ROS_INFO("Sender received return message %d", msg->count);

  if (reply->count < message_count_)
  {
    pub.publish(reply);
  }
}

STLatencyResult STLatencyTest::run()
{
  ROS_INFO("Starting receive threads");
  STLatencyResult r;
  r.test_start = ros::WallTime::now();

  ros::NodeHandle nh;
  nh.setCallbackQueue(&receive_queue_);

  ros::Publisher recv_pub = nh.advertise<roscpp::LatencyMessage>("stlatency_perf_test_return", 0);
  ros::Subscriber recv_sub = nh.subscribe<roscpp::LatencyMessage>("stlatency_perf_test", 0, boost::bind(&STLatencyTest::receiveCallback, this, _1, boost::ref(recv_pub)), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  ros::Publisher send_pub = nh.advertise<roscpp::LatencyMessage>("stlatency_perf_test", 0);
  ros::Subscriber send_sub = nh.subscribe<roscpp::LatencyMessage>("stlatency_perf_test_return", 0, boost::bind(&STLatencyTest::sendCallback, this, _1, boost::ref(send_pub)), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  ROS_INFO("Waiting for all connections to establish");

  bool cont = true;
  while (cont)
  {
    cont = recv_pub.getNumSubscribers() == 0 || send_pub.getNumSubscribers() == 0;
    ros::WallDuration(0.001).sleep();
  }

  ROS_INFO("All connections established");

  roscpp::LatencyMessagePtr msg(new roscpp::LatencyMessage);
  msg->publish_time = ros::WallTime::now().toSec();
  send_pub.publish(msg);
  while (msg->count < message_count_)
  {
    receive_queue_.callAvailable(ros::WallDuration(0.1));
  }

  r.test_end = ros::WallTime::now();

  r.latency_avg = 0;
  r.latency_max = 0;
  r.latency_min = 9999999999999ULL;
  r.total_message_count = message_count_;

  double latency_total = 0.0;
  uint32_t latency_count = 0;
  {
    std::vector<double>::iterator lat_it = result_.latencies.begin();
    std::vector<double>::iterator lat_end = result_.latencies.end();
    for (; lat_it != lat_end; ++lat_it)
    {
      double latency = *lat_it;
      r.latency_min = std::min(r.latency_min, latency);
      r.latency_max = std::max(r.latency_max, latency);
      ++latency_count;
      latency_total += latency;
    }
  }

  r.latency_avg = latency_total / latency_count;

  ROS_INFO("Done collating results");

  return r;
}

STLatencyResult stlatency(uint32_t message_count)
{
  ROS_INFO_STREAM("*****************************************************");
  ROS_INFO_STREAM("Running single-threaded latency test: message count[" << message_count << "]");

  STLatencyTest t(message_count);
  return t.run();
}

} // namespace intra
} // namespace perf_roscpp
