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

#include "ros/node.h"
#include "test_roscpp/TestEmpty.h"

ros::Node* g_node;
const char* g_node_name = "test_node";

TEST(masterInfo, getPublishedTopics)
{
  typedef std::set<std::string> S_string;
  S_string advertised_topics_;
  advertised_topics_.insert( "/test_topic_1" );
  advertised_topics_.insert( "/test_topic_2" );
  advertised_topics_.insert( "/test_topic_3" );
  advertised_topics_.insert( "/test_topic_4" );
  advertised_topics_.insert( "/test_topic_5" );
  advertised_topics_.insert( "/test_topic_6" );
  advertised_topics_.insert( "/test_topic_7" );
  advertised_topics_.insert( "/test_topic_8" );

  S_string::iterator adv_it = advertised_topics_.begin();
  S_string::iterator adv_end = advertised_topics_.end();
  for ( ; adv_it != adv_end; ++adv_it )
  {
    const std::string& topic = *adv_it;
    g_node->advertise<test_roscpp::TestEmpty>( topic, 0 );
  }

  typedef std::vector<std::pair<std::string, std::string> > V_TopicAndType;
  V_TopicAndType master_topics;
  g_node->getPublishedTopics( &master_topics );

  adv_it = advertised_topics_.begin();
  adv_end = advertised_topics_.end();
  for ( ; adv_it != adv_end; ++adv_it )
  {
    const std::string& topic = *adv_it;
    bool found = false;

    V_TopicAndType::iterator master_it = master_topics.begin();
    V_TopicAndType::iterator master_end = master_topics.end();
    for ( ; master_it != master_end; ++master_it )
    {
      const std::string& master_topic = master_it->first;
      if ( topic == master_topic )
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
  ros::init( argc, argv );

  g_node = new ros::Node( g_node_name );

  int ret = RUN_ALL_TESTS();
  

  delete g_node;

  return ret;
}
