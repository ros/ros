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

/* Author: Josh Faust */

/*
 * Test version macros
 */

#include <gtest/gtest.h>
#include "ros/poll_set.h"
#include "ros/transport/transport_tcp.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

using namespace ros;

class Synchronous : public testing::Test
{
public:
  Synchronous()
  {
  }

  ~Synchronous()
  {
  }


protected:

  virtual void SetUp()
  {
    transports_[0] = TransportTCPPtr(new TransportTCP(NULL, TransportTCP::SYNCHRONOUS));
    transports_[1] = TransportTCPPtr(new TransportTCP(NULL, TransportTCP::SYNCHRONOUS));

    if (!transports_[0]->listen(0, 100, TransportTCP::AcceptCallback()))
    {
      FAIL();
    }

    if (!transports_[1]->connect("localhost", transports_[0]->getServerPort()))
    {
      FAIL();
    }

    transports_[2] = transports_[0]->accept();
    if (!transports_[2])
    {
      FAIL();
    }
  }

  virtual void TearDown()
  {
    for (int i = 0; i < 3; ++i)
    {
      if (transports_[i])
      {
        transports_[i]->close();
      }
    }
  }

  TransportTCPPtr transports_[3];
};

TEST_F(Synchronous, writeThenRead)
{
  std::string msg = "test";
  int32_t written = transports_[1]->write((uint8_t*)msg.c_str(), msg.length());
  ASSERT_EQ(written, (int32_t)msg.length());

  uint8_t buf[5];
  memset(buf, 0, sizeof(buf));
  int32_t read = transports_[2]->read(buf, msg.length());
  ASSERT_EQ(read, (int32_t)msg.length());
  ASSERT_STREQ((const char*)buf, msg.c_str());
}

TEST_F(Synchronous, writeThenReadPartial)
{
  std::string msg = "test";
  int32_t written = transports_[1]->write((uint8_t*)msg.c_str(), msg.length());
  ASSERT_EQ(written, (int32_t)msg.length());

  uint8_t buf[5];
  memset(buf, 0, sizeof(buf));
  int32_t read = transports_[2]->read(buf, 1);
  ASSERT_EQ(read, 1);
  ASSERT_STREQ((const char*)buf, msg.substr(0, 1).c_str());
}

void readThread(TransportTCPPtr transport, uint8_t* buf, uint32_t size, volatile int32_t* read_out, volatile bool* done_read)
{
  while (*read_out < (int32_t)size)
  {
    *read_out += transport->read(buf + *read_out, size - *read_out);
  }
  *done_read = true;
}

TEST_F(Synchronous, readWhileWriting)
{
  for (int i = 0; i < 10; ++i)
  {
    const uint32_t buf_size = 1024*1024;
    std::auto_ptr<uint8_t> read_buf(new uint8_t[buf_size]);

    std::stringstream ss;
    for (int i = 0; i < 100000; ++i)
    {
      ss << i;
    }

    std::string msg = ss.str();

    ASSERT_TRUE(msg.size() < buf_size);

    volatile int32_t read_out = 0;
    volatile bool done_read = false;
    boost::thread t(boost::bind(readThread, transports_[2], read_buf.get(), msg.size(), &read_out, &done_read));

    boost::this_thread::sleep(boost::posix_time::milliseconds(50));

    int32_t written = transports_[1]->write((uint8_t*)msg.c_str(), msg.length());
    ASSERT_EQ(written, (int32_t)msg.length());

    while (!done_read)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }

    ASSERT_EQ(done_read, true);
    ASSERT_EQ(read_out, (int32_t)msg.length());
    ASSERT_STREQ((const char*)read_buf.get(), msg.c_str());
  }
}

TEST_F(Synchronous, readAfterClose)
{
  transports_[1]->close();

  uint8_t buf[5];
  int32_t read = transports_[1]->read(buf, 1);
  ASSERT_EQ(read, -1);
}

TEST_F(Synchronous, writeAfterClose)
{
  transports_[1]->close();

  std::string msg = "test";
  int32_t written = transports_[1]->write((uint8_t*)msg.c_str(), msg.length());
  ASSERT_EQ(written, -1);
}

class Polled : public testing::Test
{
public:
  Polled()
  {
  }

  ~Polled()
  {
  }


protected:

  void connectionReceived(const TransportTCPPtr& transport)
  {
    transports_[2] = transport;
  }

  void pollThread()
  {
    while (continue_)
    {
      poll_set_.update(10);
    }
  }

  void onReadable(const TransportPtr& transport, int index)
  {
    ASSERT_EQ(transport, transports_[index]);

    uint8_t b = 0;
    while (transport->read(&b, 1) > 0)
    {
      ++bytes_read_[index];
    }
  }

  void onWriteable(const TransportPtr& transport, int index)
  {
    ASSERT_EQ(transport, transports_[index]);

    uint8_t b = 0;
    transport->write(&b, 1);

    ++bytes_written_[index];
  }

  void onDisconnect(const TransportPtr& transport, int index)
  {
    ASSERT_EQ(transport, transports_[index]);

    disconnected_[index] = true;
  }

  virtual void SetUp()
  {
    bytes_read_[0] = 0;
    bytes_read_[1] = 0;
    bytes_read_[2] = 0;

    bytes_written_[0] = 0;
    bytes_written_[1] = 0;
    bytes_written_[2] = 0;

    disconnected_[0] = false;
    disconnected_[1] = false;
    disconnected_[2] = false;

    transports_[0] = TransportTCPPtr(new TransportTCP(&poll_set_));
    transports_[1] = TransportTCPPtr(new TransportTCP(&poll_set_));

    if (!transports_[0]->listen(0, 100, boost::bind(&Polled::connectionReceived, this, _1)))
    {
      FAIL();
    }

    if (!transports_[1]->connect("localhost", transports_[0]->getServerPort()))
    {
      FAIL();
    }

    continue_ = true;
    poll_thread_ = boost::thread(boost::bind(&Polled::pollThread, this));

    int count = 0;
    while (!transports_[2] && count < 100)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }

    if (!transports_[2])
    {
      FAIL();
    }

    transports_[1]->setReadCallback(boost::bind(&Polled::onReadable, this, _1, 1));
    transports_[2]->setReadCallback(boost::bind(&Polled::onReadable, this, _1, 2));
    transports_[1]->setWriteCallback(boost::bind(&Polled::onWriteable, this, _1, 1));
    transports_[2]->setWriteCallback(boost::bind(&Polled::onWriteable, this, _1, 2));
    transports_[1]->setDisconnectCallback(boost::bind(&Polled::onDisconnect, this, _1, 1));
    transports_[2]->setDisconnectCallback(boost::bind(&Polled::onDisconnect, this, _1, 2));

    transports_[1]->enableRead();
    transports_[2]->enableRead();
  }

  virtual void TearDown()
  {
    for (int i = 0; i < 3; ++i)
    {
      transports_[i]->close();
    }

    continue_ = false;
    poll_thread_.join();
  }

  TransportTCPPtr transports_[3];
  int bytes_read_[3];
  int bytes_written_[3];
  bool disconnected_[3];

  PollSet poll_set_;

  boost::thread poll_thread_;
  volatile bool continue_;
};

TEST_F(Polled, readAndWrite)
{
  transports_[1]->enableWrite();
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  transports_[1]->disableWrite();

  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  ASSERT_GT(bytes_read_[2], 0);
  ASSERT_EQ(bytes_read_[2], bytes_written_[1]);

  int old_read_val = bytes_read_[2];

  transports_[2]->enableWrite();
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  transports_[2]->disableWrite();

  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  ASSERT_EQ(bytes_read_[1], bytes_written_[2]);
  ASSERT_EQ(old_read_val, bytes_read_[2]);

  transports_[1]->enableWrite();
  transports_[2]->enableWrite();
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  transports_[1]->disableWrite();
  transports_[2]->disableWrite();

  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  ASSERT_GT(bytes_read_[2], 0);
  ASSERT_EQ(bytes_read_[2], bytes_written_[1]);
  ASSERT_GT(bytes_read_[1], 0);
  ASSERT_EQ(bytes_read_[1], bytes_written_[2]);
}

TEST_F(Polled, enableDisableWrite)
{
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  ASSERT_EQ(bytes_read_[1], 0);
  ASSERT_EQ(bytes_read_[2], 0);
  ASSERT_EQ(bytes_written_[1], 0);
  ASSERT_EQ(bytes_written_[2], 0);

  transports_[1]->enableWrite();
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  transports_[1]->disableWrite();

  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  ASSERT_GT(bytes_read_[2], 0);
  ASSERT_GT(bytes_written_[1], 0);
  int old_read_val = bytes_read_[2];
  int old_written_val = bytes_written_[1];
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  ASSERT_EQ(bytes_read_[2], old_read_val);
  ASSERT_EQ(bytes_written_[1], old_written_val);
}

TEST_F(Polled, disconnectNoTraffic)
{
  ASSERT_EQ(disconnected_[1], false);
  ASSERT_EQ(disconnected_[2], false);

  transports_[1]->close();
  ASSERT_EQ(disconnected_[1], true);

  boost::this_thread::sleep(boost::posix_time::milliseconds(50));

  ASSERT_EQ(disconnected_[2], true);
}

TEST_F(Polled, disconnectWriter)
{
  ASSERT_EQ(disconnected_[1], false);
  ASSERT_EQ(disconnected_[2], false);

  transports_[1]->enableWrite();
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  ASSERT_GT(bytes_read_[2], 0);

  transports_[1]->close();
  ASSERT_EQ(disconnected_[1], true);

  boost::this_thread::sleep(boost::posix_time::milliseconds(50));

  ASSERT_EQ(disconnected_[2], true);
}

TEST_F(Polled, disconnectReader)
{
  ASSERT_EQ(disconnected_[1], false);
  ASSERT_EQ(disconnected_[2], false);

  transports_[2]->enableWrite();
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  ASSERT_GT(bytes_read_[1], 0);

  transports_[1]->close();
  ASSERT_EQ(disconnected_[1], true);

  boost::this_thread::sleep(boost::posix_time::milliseconds(50));

  ASSERT_EQ(disconnected_[2], true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  signal(SIGPIPE, SIG_IGN);

  return RUN_ALL_TESTS();
}
