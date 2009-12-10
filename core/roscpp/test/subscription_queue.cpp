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
#include "ros/message.h"
#include "ros/callback_queue_interface.h"

#include <boost/shared_array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

using namespace ros;

class FakeMessage : public Message
{
public:
  virtual const std::string __getDataType() const { return ""; }
  virtual const std::string __getMD5Sum() const { return ""; }
  virtual const std::string __getMessageDefinition() const { return ""; }
  virtual uint32_t serializationLength() const { return 0; }
  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const { return write_ptr; }
  virtual uint8_t *deserialize(uint8_t *read_ptr) { return read_ptr; }
};

class FakeSubHelper : public SubscriptionMessageHelper
{
public:
  FakeSubHelper()
  : calls_(0)
  {}

  virtual MessagePtr create()
  {
    return MessagePtr(new FakeMessage);
  }

  virtual std::string getMD5Sum() { return ""; }
  virtual std::string getDataType() { return ""; }

  virtual void call(const MessagePtr& msg)
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

  boost::mutex mutex_;
  int32_t calls_;

  boost::function<void(void)> cb_;
};
typedef boost::shared_ptr<FakeSubHelper> FakeSubHelperPtr;

TEST(SubscriptionQueue, queueSize)
{
  SubscriptionQueue queue("blah", 1);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, boost::shared_array<uint8_t>(), 0, boost::shared_ptr<M_string>()));

  ASSERT_FALSE(queue.full());

  uint64_t id = queue.push(helper, des, false, VoidWPtr());

  ASSERT_TRUE(queue.full());

  ASSERT_EQ(queue.call(id), CallbackInterface::Success);

  ASSERT_FALSE(queue.full());

  id = queue.push(helper, des, false, VoidWPtr());

  ASSERT_TRUE(queue.full());

  ASSERT_TRUE(queue.ready(id));

  uint64_t id2 = queue.push(helper, des, false, VoidWPtr());

  ASSERT_TRUE(queue.full());

  ASSERT_TRUE(queue.ready(id));
  ASSERT_TRUE(queue.ready(id2));

  ASSERT_EQ(queue.call(id), CallbackInterface::Invalid);
  ASSERT_EQ(queue.call(id2), CallbackInterface::Success);

  ASSERT_TRUE(queue.ready(id));
  ASSERT_TRUE(queue.ready(id2));

  ASSERT_EQ(helper->calls_, 2);
}

TEST(SubscriptionQueue, infiniteQueue)
{
  SubscriptionQueue queue("blah", 0);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, boost::shared_array<uint8_t>(), 0, boost::shared_ptr<M_string>()));

  ASSERT_FALSE(queue.full());

  uint64_t id = queue.push(helper, des, false, VoidWPtr());
  ASSERT_EQ(queue.call(id), CallbackInterface::Success);

  id = queue.push(helper, des, false, VoidWPtr());

  ASSERT_TRUE(queue.ready(id));

  ASSERT_FALSE(queue.full());

  uint64_t id2 = queue.push(helper, des, false, VoidWPtr());

  ASSERT_FALSE(queue.full());

  ASSERT_TRUE(queue.ready(id));
  ASSERT_FALSE(queue.ready(id2));

  ASSERT_EQ(queue.call(id2), CallbackInterface::TryAgain);
  ASSERT_EQ(queue.call(id), CallbackInterface::Success);
  ASSERT_TRUE(queue.ready(id2));
  ASSERT_EQ(queue.call(id2), CallbackInterface::Success);

  ASSERT_TRUE(queue.ready(id));
  ASSERT_TRUE(queue.ready(id2));

  ASSERT_EQ(helper->calls_, 3);
}

TEST(SubscriptionQueue, clearCall)
{
  SubscriptionQueue queue("blah", 1);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, boost::shared_array<uint8_t>(), 0, boost::shared_ptr<M_string>()));

  uint64_t id = queue.push(helper, des, false, VoidWPtr());
  queue.clear();
  ASSERT_EQ(queue.call(id), CallbackInterface::Invalid);
}

TEST(SubscriptionQueue, clearThenAddAndCall)
{
  SubscriptionQueue queue("blah", 1);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, boost::shared_array<uint8_t>(), 0, boost::shared_ptr<M_string>()));

  uint64_t id = queue.push(helper, des, false, VoidWPtr());
  queue.clear();
  id = queue.push(helper, des, false, VoidWPtr());
  ASSERT_EQ(queue.call(id), CallbackInterface::Success);
}

TEST(SubscriptionQueue, remove)
{
  SubscriptionQueue queue("blah", 2);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, boost::shared_array<uint8_t>(), 0, boost::shared_ptr<M_string>()));

  uint64_t id = queue.push(helper, des, false, VoidWPtr());
  uint64_t id2 = queue.push(helper, des, false, VoidWPtr());

  queue.remove(id);

  ASSERT_EQ(queue.call(id2), CallbackInterface::Success);
  ASSERT_EQ(queue.call(id), CallbackInterface::Invalid);
}

void clearInCallbackCallback(SubscriptionQueue& queue)
{
  queue.clear();
}

TEST(SubscriptionQueue, clearInCallback)
{
  SubscriptionQueue queue("blah", 1);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, boost::shared_array<uint8_t>(), 0, boost::shared_ptr<M_string>()));

  helper->cb_ = boost::bind(clearInCallbackCallback, boost::ref(queue));
  uint64_t id = queue.push(helper, des, false, VoidWPtr());
  queue.call(id);
}

void clearWhileThreadIsBlockingCallback(bool* done, boost::barrier* barrier)
{
  barrier->wait();
  ros::Duration(.1).sleep();
  *done = true;
}

void callThread(SubscriptionQueue& queue, uint64_t id)
{
  queue.call(id);
}

TEST(SubscriptionQueue, clearWhileThreadIsBlocking)
{
  SubscriptionQueue queue("blah", 1);

  FakeSubHelperPtr helper(new FakeSubHelper);
  MessageDeserializerPtr des(new MessageDeserializer(helper, boost::shared_array<uint8_t>(), 0, boost::shared_ptr<M_string>()));

  bool done = false;
  boost::barrier barrier(2);
  helper->cb_ = boost::bind(clearWhileThreadIsBlockingCallback, &done, &barrier);
  uint64_t id = queue.push(helper, des, false, VoidWPtr());
  boost::thread t(callThread, boost::ref(queue), id);
  barrier.wait();

  queue.clear();

  ASSERT_TRUE(done);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


