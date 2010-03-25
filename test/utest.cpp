// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rosbag/rosbag.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <boost/foreach.hpp>
#include <gtest/gtest.h>

TEST(Rosbag, simplewrite)
{
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);
  
  std_msgs::String str;
  str.data = std::string("foo");

  std_msgs::Int32 i;
  i.data = 42;

  bag.write("chatter", ros::Time::now(), str);
  bag.write("numbers", ros::Time::now(), i);

  bag.close();

  /*
  std::string in("foo");
  std::string expected("foo");
  std::string out;

  ASSERT_TRUE(topic_tools::getBaseName(in, out));
  ASSERT_EQ(expected, out);
  */
}

TEST(Rosbag, simpleread)
{

  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("chatter"));
  topics.push_back(std::string("numbers"));

  rosbag::MessageList messages = bag.getMessageListByTopic(topics);

  BOOST_FOREACH( rosbag::MessageInstance m, messages)
  {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL)
    {
      std::cout << "The string contains: " << s->data << std::endl;
      std::cout << "At time: " << m.getTime() << std::endl;
    }

    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != NULL)
    {
      std::cout << "The int contains: " << i->data << std::endl;
      std::cout << "At time: " << m.getTime() << std::endl;
    }
  }

  bag.close();

  /*
  std::string in("/foo");
  std::string expected("foo");
  std::string out;

  ASSERT_TRUE(topic_tools::getBaseName(in, out));
  ASSERT_EQ(expected, out);
  */
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
