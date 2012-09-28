/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

/* Author: Josh Faust */

/*
 * Test subscription queue
 */

#include <gtest/gtest.h>
#include "ros/subscription_queue.h"
#include "ros/message_deserializer.h"
#include "ros/callback_queue_interface.h"
#include "ros/subscription_callback_helper.h"
#include "ros/init.h"

#include <boost/shared_array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

using namespace ros;

class FakeMessage
{
public:
  virtual const std::string __getDataType() const { return ""; }
  virtual const std::string __getMD5Sum() const { return ""; }
  virtual const std::string __getMessageDefinition() const { return ""; }
  virtual uint32_t serializationLength() const { return 0; }
  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const { return write_ptr; }
  virtual uint8_t *deserialize(uint8_t *read_ptr) { return read_ptr; }
};

class FakeSubHelper : public SubscriptionCallbackHelper
{
public:
  FakeSubHelper()
  : calls_(0)
  {}

  virtual VoidConstPtr deserialize(const SubscriptionCallbackHelperDeserializeParams&)
  {
    return VoidConstPtr(new FakeMessage);
  }

  virtual std::string getMD5Sum() { return ""; }
  virtual std::string getDataType() { return ""; }

  virtual void call(SubscriptionCallbackHelperCallParams& params)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      ++calls_;
    }

    if (cb_)
    {
      cb_();
    }
  }

  virtual const std::type_info& getTypeInfo() { return typeid(FakeMessage); }
  virtual bool isConst() { return true; }

  boost::mutex mutex_;
  int32_t calls_;

  boost::function<void(void)> cb_;
};
typedef boost::shared_ptr<FakeSubHelper> FakeSubHelperPtr;

TEST(SubscriptionQueue, queueSize)
{
  SubscriptionQueue queue("blah", 1, false);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  ASSERT_FALSE(queue.full());

  queue.push(helper, des, false, VoidConstWPtr(), true);

  ASSERT_TRUE(queue.full());

  ASSERT_EQ(queue.call(), CallbackInterface::Success);

  ASSERT_FALSE(queue.full());

  queue.push(helper, des, false, VoidConstWPtr(), true);

  ASSERT_TRUE(queue.full());

  ASSERT_TRUE(queue.ready());

  queue.push(helper, des, false, VoidConstWPtr(), true);

  ASSERT_TRUE(queue.full());

  ASSERT_EQ(queue.call(), CallbackInterface::Success);
  ASSERT_EQ(queue.call(), CallbackInterface::Invalid);

  ASSERT_EQ(helper->calls_, 2);
}

TEST(SubscriptionQueue, infiniteQueue)
{
  SubscriptionQueue queue("blah", 0, false);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  ASSERT_FALSE(queue.full());

  queue.push(helper, des, false, VoidConstWPtr(), true);
  ASSERT_EQ(queue.call(), CallbackInterface::Success);

  ASSERT_FALSE(queue.full());

  for (int i = 0; i < 10000; ++i)
  {
    queue.push(helper, des, false, VoidConstWPtr(), true);
  }

  ASSERT_FALSE(queue.full());

  for (int i = 0; i < 10000; ++i)
  {
    ASSERT_EQ(queue.call(), CallbackInterface::Success);
  }

  ASSERT_EQ(queue.call(), CallbackInterface::Invalid);

  ASSERT_EQ(helper->calls_, 10001);
}

TEST(SubscriptionQueue, clearCall)
{
  SubscriptionQueue queue("blah", 1, false);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  queue.push(helper, des, false, VoidConstWPtr(), true);
  queue.clear();
  ASSERT_EQ(queue.call(), CallbackInterface::Invalid);
}

TEST(SubscriptionQueue, clearThenAddAndCall)
{
  SubscriptionQueue queue("blah", 1, false);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  queue.push(helper, des, false, VoidConstWPtr(), true);
  queue.clear();
  queue.push(helper, des, false, VoidConstWPtr(), true);
  ASSERT_EQ(queue.call(), CallbackInterface::Success);
}

void clearInCallbackCallback(SubscriptionQueue& queue)
{
  queue.clear();
}

TEST(SubscriptionQueue, clearInCallback)
{
  SubscriptionQueue queue("blah", 1, false);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  helper->cb_ = boost::bind(clearInCallbackCallback, boost::ref(queue));
  queue.push(helper, des, false, VoidConstWPtr(), true);
  queue.call();
}

void clearWhileThreadIsBlockingCallback(bool* done, boost::barrier* barrier)
{
  barrier->wait();
  ros::WallDuration(.1).sleep();
  *done = true;
}

void callThread(SubscriptionQueue& queue)
{
  queue.call();
}

TEST(SubscriptionQueue, clearWhileThreadIsBlocking)
{
  SubscriptionQueue queue("blah", 1, false);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  bool done = false;
  boost::barrier barrier(2);
  helper->cb_ = boost::bind(clearWhileThreadIsBlockingCallback, &done, &barrier);
  queue.push(helper, des, false, VoidConstWPtr(), true);
  boost::thread t(callThread, boost::ref(queue));
  barrier.wait();

  queue.clear();

  ASSERT_TRUE(done);
}

void waitForBarrier(boost::barrier* bar)
{
  bar->wait();
}

TEST(SubscriptionQueue, concurrentCallbacks)
{
  SubscriptionQueue queue("blah", 0, true);
  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  boost::barrier bar(2);
  helper->cb_ = boost::bind(waitForBarrier, &bar);
  queue.push(helper, des, false, VoidConstWPtr(), true);
  queue.push(helper, des, false, VoidConstWPtr(), true);
  boost::thread t1(callThread, boost::ref(queue));
  boost::thread t2(callThread, boost::ref(queue));
  t1.join();
  t2.join();

  ASSERT_EQ(helper->calls_, 2);
}

void waitForASecond()
{
  ros::WallDuration(1.0).sleep();
}

TEST(SubscriptionQueue, nonConcurrentOrdering)
{
  SubscriptionQueue queue("blah", 0, false);
  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  helper->cb_ = waitForASecond;
  queue.push(helper, des, false, VoidConstWPtr(), true);
  queue.push(helper, des, false, VoidConstWPtr(), true);
  boost::thread t1(callThread, boost::ref(queue));
  boost::thread t2(callThread, boost::ref(queue));
  t1.join();
  t2.join();

  ASSERT_EQ(helper->calls_, 1);
  queue.call();
  ASSERT_EQ(helper->calls_, 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "blah");
  return RUN_ALL_TESTS();
}


