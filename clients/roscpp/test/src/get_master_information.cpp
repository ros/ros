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
 * Test getting various information from the master, like the published topics
 */

#include <string>
#include <sstream>
#include <fstream>
#include <set>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "roscpp/TestEmpty.h"

TEST(masterInfo, getPublishedTopics)
{
  ros::NodeHandle nh;

  typedef std::set<std::string> S_string;
  S_string advertised_topics;
  advertised_topics.insert( "/test_topic_1" );
  advertised_topics.insert( "/test_topic_2" );
  advertised_topics.insert( "/test_topic_3" );
  advertised_topics.insert( "/test_topic_4" );
  advertised_topics.insert( "/test_topic_5" );
  advertised_topics.insert( "/test_topic_6" );
  advertised_topics.insert( "/test_topic_7" );
  advertised_topics.insert( "/test_topic_8" );

  std::vector<ros::Publisher> pubs;

  S_string::iterator adv_it = advertised_topics.begin();
  S_string::iterator adv_end = advertised_topics.end();
  for ( ; adv_it != adv_end; ++adv_it )
  {
    const std::string& topic = *adv_it;
    pubs.push_back(nh.advertise<roscpp::TestEmpty>( topic, 0 ));
  }

  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  adv_it = advertised_topics.begin();
  adv_end = advertised_topics.end();
  for ( ; adv_it != adv_end; ++adv_it )
  {
    const std::string& topic = *adv_it;
    bool found = false;

    ros::master::V_TopicInfo::iterator master_it = master_topics.begin();
    ros::master::V_TopicInfo::iterator master_end = master_topics.end();
    for ( ; master_it != master_end; ++master_it )
    {
      const ros::master::TopicInfo& info = *master_it;
      if ( topic == info.name )
      {
        found = true;
        break;
      }
    }

    ASSERT_TRUE( found );
  }
}


int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init( argc, argv, "get_master_information" );
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
