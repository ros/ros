// Copyright (c) 2010, Willow Garage, Inc.
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

#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/view.h"

#include <iostream>

#include <set>

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>

#include <gtest/gtest.h>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#define foreach BOOST_FOREACH

struct IndexEntryMultiSetCompare
{
    bool operator()(rosbag::IndexEntry const& a, rosbag::IndexEntry const& b) const { return a.time < b.time; }
};

TEST(rosbag, multiset_vs_vector)
{
    int count = 1000 * 1000;

    std::multiset<rosbag::IndexEntry, IndexEntryMultiSetCompare> s;
    ros::Time start = ros::Time::now();
    for (int i = 0; i < count; i++) {
        rosbag::IndexEntry e;
        e.time = ros::Time::now();
        e.chunk_pos = 0;
        e.offset = 0;
        s.insert(s.end(), e);
    }
    ros::Time end = ros::Time::now();
    std::cout << "multiset: " << end - start << std::endl;

    std::vector<rosbag::IndexEntry> v;
    start = ros::Time::now();
    for (int i = 0; i < count; i++) {
        rosbag::IndexEntry e;
        e.time = ros::Time::now();
        e.chunk_pos = 0;
        e.offset = 0;
        v.push_back(e);
    }
    end = ros::Time::now();
    std::cout << "vector: " << end - start << std::endl;
}

class BagTest : public testing::Test
{
protected:
    std_msgs::String foo_, bar_;
    std_msgs::Int32 i_;
    
    virtual void SetUp() {
        foo_.data = std::string("foo");
        bar_.data = std::string("bar");
        i_.data = 42;
    }

    void dumpContents(std::string const& filename) {
        rosbag::Bag b;
        b.open(filename, rosbag::bagmode::Read);
        dumpContents(b);        
        b.close();
    }

    void dumpContents(rosbag::Bag& b) {
        b.dump();

    	rosbag::View view;
    	view.addQuery(b, rosbag::Query());
        foreach(rosbag::MessageInstance m, view) {
            std::cout << m.getTime() << ": [" << m.getTopic() << "]" << std::endl;
        }
    }

    void checkContents(std::string const& filename) {
        rosbag::Bag b;
        b.open(filename, rosbag::bagmode::Read);

        int message_count = 0;
    	rosbag::View view;
    	view.addQuery(b, rosbag::Query());
        foreach(rosbag::MessageInstance m, view) {
            std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
            if (s != NULL) {
                ASSERT_EQ(s->data, foo_.data);
                message_count++;
            }
            std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
            if (i != NULL) {
                ASSERT_EQ(i->data, i_.data);
                message_count++;
            }
        }
        ASSERT_EQ(message_count, 2);

        b.close();
    }
};

TEST(rosbag, simplewrite)
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
}

TEST(rosbag, simpleread)
{
    rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("numbers"));

    rosbag::View view;
    view.addQuery(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view) {

        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
            ASSERT_EQ(s->data, std::string("foo"));

        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
            ASSERT_EQ(i->data, 42);
    }

    bag.close();
}

TEST(rosbag, timequery)
{
    rosbag::Bag outbag;
    outbag.open("timequery.bag", rosbag::bagmode::Write);

    std_msgs::Int32 imsg;

    for (int i = 0; i < 1000; i++) {
        imsg.data = i;
        switch (rand() % 5) {
        case 0:
            outbag.write("t0", ros::Time(i, 0), imsg);
            break;
        case 1:
            outbag.write("t1", ros::Time(i, 0), imsg);
            break;
        case 2:
            outbag.write("t2", ros::Time(i, 0), imsg);
            break;
        case 3:
            outbag.write("t2", ros::Time(i, 0), imsg);
            break;
        case 4:
            outbag.write("t4", ros::Time(i, 0), imsg);
            break;
        }
    }
    outbag.close();

    rosbag::Bag bag;
    bag.open("timequery.bag", rosbag::bagmode::Read);

    rosbag::View view;
    view.addQuery(bag, rosbag::Query(ros::Time(23, 0), ros::Time(782, 0)));

    int i = 23;

    foreach(rosbag::MessageInstance const m, view) {
        std_msgs::Int32::ConstPtr imsg = m.instantiate<std_msgs::Int32>();
        if (imsg != NULL)
        {
            ASSERT_EQ(imsg->data, i++);
            ASSERT_TRUE(m.getTime() < ros::Time(783,0));
        }
    }

    bag.close();
}

TEST(rosbag, topicquery)
{
    rosbag::Bag outbag;
    outbag.open("topicquery.bag", rosbag::bagmode::Write);

    std_msgs::Int32 imsg;

    int j0 = 0;
    int j1 = 0;

    for (int i = 0; i < 1000; i++) {
        switch (rand() % 5) {
        case 0:
            imsg.data = j0++;
            outbag.write("t0", ros::Time(i, 0), imsg);
            break;
        case 1:
            imsg.data = j0++;
            outbag.write("t1", ros::Time(i, 0), imsg);
            break;
        case 2:
            imsg.data = j1++;
            outbag.write("t2", ros::Time(i, 0), imsg);
            break;
        case 3:
            imsg.data = j1++;
            outbag.write("t3", ros::Time(i, 0), imsg);
            break;
        case 4:
            imsg.data = j1++;
            outbag.write("t4", ros::Time(i, 0), imsg);
            break;
        }
    }
    outbag.close();

    rosbag::Bag bag;
    bag.open("topicquery.bag", rosbag::bagmode::Read);

    std::vector<std::string> t = boost::assign::list_of("t0")("t1");

    rosbag::View view;
    view.addQuery(bag, rosbag::TopicQuery(t));

    int i = 0;

    foreach(rosbag::MessageInstance const m, view) {
        std_msgs::Int32::ConstPtr imsg = m.instantiate<std_msgs::Int32>();
        if (imsg != NULL)
        {
            ASSERT_EQ(imsg->data, i++);
        }
    }

    bag.close();
}

TEST(rosbag, verifymultibag)
{
    rosbag::Bag outbag1;
    outbag1.open("bag1.bag", rosbag::bagmode::Write);

    rosbag::Bag outbag2;
    outbag2.open("bag2.bag", rosbag::bagmode::Write);

    std_msgs::Int32 imsg;
    for (int i = 0; i < 1000; i++) {
        imsg.data = i;
        switch (rand() % 5) {
        case 0:
            outbag1.write("t0", ros::Time::now(), imsg);
            break;
        case 1:
            outbag1.write("t1", ros::Time::now(), imsg);
            break;
        case 2:
            outbag1.write("t2", ros::Time::now(), imsg);
            break;
        case 3:
            outbag2.write("t0", ros::Time::now(), imsg);
            break;
        case 4:
            outbag2.write("t1", ros::Time::now(), imsg);
            break;
        }
    }

    outbag1.close();
    outbag2.close();

    rosbag::Bag bag1;
    bag1.open("bag1.bag", rosbag::bagmode::Read);

    rosbag::Bag bag2;
    bag2.open("bag2.bag", rosbag::bagmode::Read);

    rosbag::View view;
    view.addQuery(bag1, rosbag::Query());
    view.addQuery(bag2, rosbag::Query());

    int i = 0;

    foreach(rosbag::MessageInstance const m, view) {
        std_msgs::Int32::ConstPtr imsg = m.instantiate<std_msgs::Int32>();
        if (imsg != NULL)
            ASSERT_EQ(imsg->data, i++);
    }

    bag1.close();
    bag2.close();
}

TEST(rosbag, modifyview)
{
    rosbag::Bag outbag;
    outbag.open("modify.bag", rosbag::bagmode::Write);

    std_msgs::Int32 imsg;

    int j0 = 0;
    int j1 = 1;

    // Create a bag with 2 interlaced topics
    for (int i = 0; i < 100; i++) {
        imsg.data = j0;
        j0 += 2;
        outbag.write("t0", ros::Time(2 * i, 0), imsg);

        imsg.data = j1;
        j1 += 2;
        outbag.write("t1", ros::Time(2 * i + 1, 0), imsg);
    }
    outbag.close();

    rosbag::Bag bag;
    bag.open("modify.bag", rosbag::bagmode::Read);

    std::vector<std::string> t0 = boost::assign::list_of("t0");
    std::vector<std::string> t1 = boost::assign::list_of("t1");

    // We're going to skip the t1 for the first half
    j0 = 0;
    j1 = 101;

    rosbag::View view;
    view.addQuery(bag, rosbag::TopicQuery(t0));

    rosbag::View::iterator iter = view.begin();

    for (int i = 0; i < 50; i++) {
        std_msgs::Int32::ConstPtr imsg = iter->instantiate<std_msgs::Int32> ();

        if (imsg != NULL) {
            ASSERT_EQ(imsg->data, j0);
            j0+=2;
        }
        iter++;
    }

    // We now add our query, and expect it to show up
    view.addQuery(bag, rosbag::TopicQuery(t1));

    for (int i = 0; i < 50; i++)
    {
        std_msgs::Int32::ConstPtr imsg = iter->instantiate<std_msgs::Int32>();

        if (imsg != NULL)
        {
            ASSERT_EQ(imsg->data, j0);
            j0+=2;
        }

        iter++;
        imsg = iter->instantiate<std_msgs::Int32>();

        if (imsg != NULL)
        {
            ASSERT_EQ(imsg->data, j1);
            j1+=2;
        }
        iter++;
    }

    bag.close();
}



TEST(rosbag, modifybag)
{
    rosbag::Bag rwbag;
    // Looks like mode is largely being ignored at the moment.
    rwbag.open("rwbag.bag", rosbag::bagmode::Write);// | rosbag::bagmode::Read);

    std::vector<std::string> t0 = boost::assign::list_of("t0");

    rosbag::View view;
    view.addQuery(rwbag, rosbag::TopicQuery(t0));

    std_msgs::Int32 omsg;

    // Put a message at time 5
    omsg.data = 5;
    rwbag.write("t0", ros::Time(5 + 1, 0), omsg);
    
    // Verify begin gets us to 5
    rosbag::View::iterator iter1 = view.begin();
    std_msgs::Int32::ConstPtr imsg = iter1->instantiate<std_msgs::Int32> ();
    ASSERT_EQ(imsg->data, 5);

    for (int i = 0; i < 5; i++)
    {
        omsg.data = i;
        rwbag.write("t0", ros::Time(i + 1, 0), omsg);
    }

    // New iterator should be at 0
    rosbag::View::iterator iter2 = view.begin();
    imsg = iter2->instantiate<std_msgs::Int32> ();
    ASSERT_EQ(imsg->data, 0);

    // Increment it once
    iter2++;
    
    // Output additional messages after time 5
    for (int i = 6; i < 10; i++)
    {
        omsg.data = i;
        rwbag.write("t0", ros::Time(i + 1, 0), omsg);
    }

    // Iter2 should contain 1->10
    for (int i = 1; i < 10; i++) {
        imsg = iter2->instantiate<std_msgs::Int32> ();
        ASSERT_EQ(imsg->data, i);
        iter2++;
    }

    // Iter1 should contain 5->10
    for (int i = 5; i < 10; i++) {
        imsg = iter1->instantiate<std_msgs::Int32> ();
        ASSERT_EQ(imsg->data, i);
        iter1++;
    }
    
    rwbag.close();
}



TEST_F(BagTest, WriteThenReadWorks) {
    std::string filename("test/WriteThenRead.bag");

    rosbag::Bag b1;
    b1.open(filename, rosbag::bagmode::Write);
    b1.write("chatter", ros::Time::now(), foo_);
    b1.write("numbers", ros::Time::now(), i_);
    b1.close();

    checkContents(filename);
}

TEST_F(BagTest, AppendWorks) {
    std::string filename("test/Append.bag");

    rosbag::Bag b1;
    b1.open(filename, rosbag::bagmode::Write);
    b1.write("chatter", ros::Time::now(), foo_);
    b1.close();

    rosbag::Bag b2;
    b2.open(filename, rosbag::bagmode::Append);
    b2.write("numbers", ros::Time::now(), i_);
    b2.close();

    checkContents(filename);
}

/*
TEST_F(BagTest, ReadAppendWorks) {
    std::string filename("test/ReadAppend.bag");

    rosbag::Bag b;
    b.open(filename, rosbag::bagmode::Write);
    b.write("chatter", ros::Time::now(), foo_);
    b.close();

    rosbag::Bag b2;
    b2.open(filename, rosbag::bagmode::Append);
    b2.write("numbers", ros::Time::now(), i_);
    b2.close();

    checkContents(filename);
}


TEST_F(BagTest, ChunkedFileWorks) {
    std::string filename("test/ChunkedFile.bag");

    std::string s1("aaaa");
    std::string s2("bbbb");
    std::string s3("cccc");

    rosbag::ChunkedFile f;

    f.openWrite(filename);
    f.setWriteMode(rosbag::compression::BZ2);
    f.write(s1);
    f.setWriteMode(rosbag::compression::None);
    f.write(s2);
    
    uint64_t offset1 = f.getOffset();
    std::cout << "offset = " << offset1 << std::endl;

    f.setWriteMode(rosbag::compression::BZ2);
    f.write(s3);

    f.close();

    //

    int length = 4;
    char buffer[length];

    f.openRead(filename);
    
    f.setReadMode(rosbag::compression::BZ2);
    f.read((void*) buffer, length);
    std::cout << std::string(buffer, length) << std::endl;
    for (int i = 0; i < length; i++) buffer[i] = ' ';

    f.setReadMode(rosbag::compression::None);
    f.read((void*) buffer, length);
    std::cout << std::string(buffer, length) << std::endl;
    for (int i = 0; i < length; i++) buffer[i] = ' ';

    f.seek(offset1);
    f.setReadMode(rosbag::compression::BZ2);
    f.read((void*) buffer, length);
    std::cout << std::string(buffer, length) << std::endl;
    for (int i = 0; i < length; i++) buffer[i] = ' ';

    f.seek(0);
    f.setReadMode(rosbag::compression::BZ2);

    f.read((void*) buffer, length);
    std::cout << f.getOffset() << std::endl;
    std::cout << std::string(buffer, length) << std::endl;
    for (int i = 0; i < length; i++) buffer[i] = ' ';
    
    f.close();
}

TEST_F(BagTest, ChunkedFileReadWriteWorks) {
    std::string filename("test/ChunkedFileReadWrite.bag");

    std::string s1("aaaa");
    std::string s2("bbbb");
    std::string s3("cccc");

    int length = 4;
    char buffer[length];

    rosbag::ChunkedFile f;

    f.openReadWrite(filename);

    // 0: write "cccc" (compressed) and close (remember file pos in offset0)
    f.setWriteMode(rosbag::compression::BZ2);
    f.write(s3);
    f.setWriteMode(rosbag::compression::None);
    uint64_t offset0 = f.getOffset();
    ROS_INFO("offset0: %llu", (unsigned long long) offset0);

    // 4: write "aaaa" (compressed) and keep open (remember file pos in offset1)
    f.setWriteMode(rosbag::compression::BZ2);
    f.write(s1);
    uint64_t offset1 = f.getOffset();

    f.seek(0);
    f.setReadMode(rosbag::compression::BZ2);
    f.read((void*) buffer, length);
    std::cout << std::string(buffer, length) << std::endl;
    for (int i = 0; i < length; i++) buffer[i] = ' ';
    f.setReadMode(rosbag::compression::None);

    f.seek(offset1);
    f.setWriteMode(rosbag::compression::None);
    f.setWriteMode(rosbag::compression::BZ2);
    f.write(s2);
    f.close();

    f.openRead(filename);
    f.seek(offset1);
    f.setReadMode(rosbag::compression::BZ2);
    f.read((void*) buffer, length);
    std::cout << std::string(buffer, length) << std::endl;
    for (int i = 0; i < length; i++) buffer[i] = ' ';
    f.read((void*) buffer, length);
    std::cout << std::string(buffer, length) << std::endl;
    for (int i = 0; i < length; i++) buffer[i] = ' ';
    f.close();
}

  TEST_F(BagTest, TestChunkSizes) {
  uint32_t chunk_threshold_lo =  700 * 1024;
  uint32_t chunk_threshold_hi = 1000 * 1024;

  for (uint32_t t = chunk_threshold_lo; t <= chunk_threshold_hi; t += (25 * 1024)) {
  rosbag::Bag b;
  b.setChunkThreshold(t);
  b.rewrite("test/diag_test.bag", "test/diag_test_" + boost::lexical_cast<std::string>(t) + ".bag");
  }
  }

  TEST_F(BagTest, Read102IndexedWorks) {
  rosbag::Bag b;
  b.open("test/sample_1.2_indexed.bag", rosbag::bagmode::Read);
  b.close();
  }

  TEST_F(BagTest, Convert102To103Works) {
  rosbag::Bag in, out;
  in.open("test/sample_1.2_indexed.bag", rosbag::bagmode::Read);
  out.open("test/Convert102To103Works.bag", rosbag::bagmode::Write);
  rosbag::View view;
  view.addQuery(bag, rosbag::Query());
  foreach(rosbag::MessageInstance m, view) {
  m.instantiateMessage();
  out.write(m.getTopic(), m.getTime(), m);
  }
  in.close();
  out.close();
  }

  TEST_F(BagTest, RewriteWorks) {
  rosbag::Bag in, out;
  in.open("test/sample_1.3.bag", rosbag::bagmode::Read);
  out.open("test/RewriteWorks.bag", rosbag::bagmode::Write);
  rosbag::View view;
  view.addQuery(bag, rosbag::Query());
  foreach(rosbag::MessageInstance m, view) {
  m.instantiateMessage();
  out.write(m.getTopic(), m.getTime(), m);
  }
  in.close();
  out.close();
  }

  TEST_F(BagTest, Read102UnindexedWorks) {
  rosbag::Bag b;
  b.open("test/sample_1.2_unindexed.bag", rosbag::bagmode::Read);
  b.close();
  }
*/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
