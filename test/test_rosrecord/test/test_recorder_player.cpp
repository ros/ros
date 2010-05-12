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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "std_msgs/String.h"

#include "rosrecord/Recorder.h"
#include "rosrecord/Player.h"
#include "rosrecord/AnyMsg.h"

void string_handler(std::string name,      // Topic name       
                    std_msgs::String* str, // Message pointer  
                    ros::Time t,           // Message timestamp
                    ros::Time t_shift,     // Shifted time     
                    void* n)               // Void pointer     
{
  std_msgs::String* msg_ptr = (std_msgs::String*)(n);
  
  EXPECT_EQ(str->data, msg_ptr->data);
  EXPECT_EQ(str->__getMD5Sum(), msg_ptr->__getMD5Sum());
  EXPECT_EQ(str->__getDataType(), msg_ptr->__getDataType());
  EXPECT_EQ(str->__getMessageDefinition(), msg_ptr->__getMessageDefinition());

  EXPECT_EQ((*(str->__connection_header))["latching"], std::string("0"));
  EXPECT_EQ((*(str->__connection_header))["callerid"], std::string(""));
  EXPECT_EQ((*(str->__connection_header))["md5sum"], msg_ptr->__getMD5Sum());
  EXPECT_EQ((*(str->__connection_header))["type"], msg_ptr->__getDataType());
  EXPECT_EQ((*(str->__connection_header))["message_definition"], msg_ptr->__getMessageDefinition());

}

void any_handler(std::string name,      // Topic name       
                 ros::Message* any, // Message pointer  
                 ros::Time t,           // Message timestamp
                 ros::Time t_shift,     // Shifted time     
                 void* n)               // Void pointer     
{
  std_msgs::String* msg_ptr = (std_msgs::String*)(n);

  EXPECT_EQ(any->__getMD5Sum(), msg_ptr->__getMD5Sum());
  EXPECT_EQ(any->__getDataType(), msg_ptr->__getDataType());
  EXPECT_EQ(any->__getMessageDefinition(), msg_ptr->__getMessageDefinition());

  EXPECT_EQ((*(any->__connection_header))["latching"], std::string("0"));
  EXPECT_EQ((*(any->__connection_header))["callerid"], std::string(""));
  EXPECT_EQ((*(any->__connection_header))["md5sum"], msg_ptr->__getMD5Sum());
  EXPECT_EQ((*(any->__connection_header))["type"], msg_ptr->__getDataType());
  EXPECT_EQ((*(any->__connection_header))["message_definition"], msg_ptr->__getMessageDefinition());

}


TEST(TestSuite, record_play_ptr)
{
  ros::record::Recorder recorder;
  ASSERT_TRUE(recorder.open("test.bag"));

  std_msgs::String::Ptr msg_ptr(new std_msgs::String);
  msg_ptr->data = std::string("abcdefghi");

  EXPECT_TRUE(recorder.record("some_topic", msg_ptr, ros::Time::now()));

  recorder.close();

  ros::record::Player player;

  ASSERT_TRUE(player.open(std::string("test.bag"), ros::Time()));

  player.addHandler<std_msgs::String>(std::string("some_topic"),
                                      &string_handler,
                                      &(*msg_ptr));
  player.addHandler<AnyMsg>(std::string("some_topic"),
                                       &any_handler,
                                       &(*msg_ptr),
                                       false);

  EXPECT_TRUE(player.nextMsg());
  EXPECT_FALSE(player.nextMsg());
}

TEST(TestSuite, record_play_ref)
{
  ros::record::Recorder recorder;
  ASSERT_TRUE(recorder.open("test.bag"));

  std_msgs::String msg;
  msg.data = std::string("jklmnop");
  

  EXPECT_TRUE(recorder.record("some_topic", msg, ros::Time::now()));

  recorder.close();

  ros::record::Player player;

  ASSERT_TRUE(player.open(std::string("test.bag"), ros::Time()));

  player.addHandler<std_msgs::String>(std::string("some_topic"),
                                      &string_handler,
                                      &msg);
  player.addHandler<AnyMsg>(std::string("some_topic"),
                                       &any_handler,
                                       &msg,
                                       false);

  EXPECT_TRUE(player.nextMsg());
  EXPECT_FALSE(player.nextMsg());
}

void string_latching_handler(std::string name,      // Topic name       
                    std_msgs::String* str, // Message pointer  
                    ros::Time t,           // Message timestamp
                    ros::Time t_shift,     // Shifted time     
                    void* n)               // Void pointer     
{
  std_msgs::String* msg_ptr = (std_msgs::String*)(n);
  
  EXPECT_EQ((*(str->__connection_header))["latching"], std::string("1"));
  EXPECT_EQ((*(str->__connection_header))["callerid"], std::string("/my_node"));
  EXPECT_EQ((*(str->__connection_header))["md5sum"], msg_ptr->__getMD5Sum());
  EXPECT_EQ((*(str->__connection_header))["type"], msg_ptr->__getDataType());
  EXPECT_EQ((*(str->__connection_header))["message_definition"], msg_ptr->__getMessageDefinition());
}

void any_latching_handler(std::string name,      // Topic name       
                 ros::Message* any, // Message pointer  
                 ros::Time t,           // Message timestamp
                 ros::Time t_shift,     // Shifted time     
                 void* n)               // Void pointer     
{
  std_msgs::String* msg_ptr = (std_msgs::String*)(n);

  EXPECT_EQ((*(any->__connection_header))["latching"], std::string("1"));
  EXPECT_EQ((*(any->__connection_header))["callerid"], std::string("/my_node"));
  EXPECT_EQ((*(any->__connection_header))["md5sum"], msg_ptr->__getMD5Sum());
  EXPECT_EQ((*(any->__connection_header))["type"], msg_ptr->__getDataType());
  EXPECT_EQ((*(any->__connection_header))["message_definition"], msg_ptr->__getMessageDefinition());
}

TEST(TestSuite, record_play_latching)
{
  ros::record::Recorder recorder;
  ASSERT_TRUE(recorder.open("test.bag"));

  std_msgs::String msg;
  msg.data = std::string("qrestuv");
  msg.__connection_header = boost::shared_ptr<ros::M_string>(new ros::M_string);
  (*msg.__connection_header)["latching"] = std::string("1");
  (*msg.__connection_header)["callerid"] = std::string("/my_node");

  EXPECT_TRUE(recorder.record("some_topic", msg, ros::Time::now()));

  recorder.close();

  ros::record::Player player;

  ASSERT_TRUE(player.open(std::string("test.bag"), ros::Time()));

  player.addHandler<std_msgs::String>(std::string("some_topic"),
                                      &string_latching_handler,
                                      &msg);
  player.addHandler<AnyMsg>(std::string("some_topic"),
                                       &any_latching_handler,
                                       &msg,
                                       false);

  EXPECT_TRUE(player.nextMsg());
  EXPECT_FALSE(player.nextMsg());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_recorder_player");
  return RUN_ALL_TESTS();
}

