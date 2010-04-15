// utest.cpp

#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/message_instance.h"

#include <iostream>

#include <boost/foreach.hpp>
#include <gtest/gtest.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "topic_tools/shape_shifter.h"

#define foreach BOOST_FOREACH

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
        foreach(rosbag::MessageInstance const& m, b.getMessageList()) {
            std::cout << m.getTime() << ": [" << m.getTopic() << "]" << std::endl;
        }
    }

    void checkContents(std::string const& filename) {
        rosbag::Bag b;
        b.open(filename, rosbag::bagmode::Read);

        int message_count = 0;
        foreach(rosbag::MessageInstance const& m, b.getMessageList()) {
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

TEST_F(BagTest, ReadAppendWorks) {
    std::string filename("test/ReadAppend.bag");

    rosbag::Bag b;
    b.open(filename, rosbag::bagmode::Write);
    b.write("chatter", ros::Time::now(), foo_);
	b.close();

    rosbag::Bag b2;
    b2.open(filename, rosbag::bagmode::ReadAppend);
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
    ROS_INFO("offset0: %llu", offset0);

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

/*
TEST_F(BagTest, TestChunkSizes) {
    uint32_t chunk_threshold_lo =  700 * 1024;
    uint32_t chunk_threshold_hi = 1000 * 1024;

    for (uint32_t t = chunk_threshold_lo; t <= chunk_threshold_hi; t += (25 * 1024)) {
        rosbag::Bag b;
        b.setChunkThreshold(t);
        b.rewrite("test/diag_test.bag", "test/diag_test_" + boost::lexical_cast<std::string>(t) + ".bag");
    }
}
*/
/*
TEST_F(BagTest, Read102IndexedWorks) {
	rosbag::Bag b;
	b.open("test/sample_1.2_indexed.bag", rosbag::bagmode::Read);
	b.close();
}

TEST_F(BagTest, Convert102To103Works) {
	rosbag::Bag in, out;
	in.open("test/sample_1.2_indexed.bag", rosbag::bagmode::Read);
	out.open("test/Convert102To103Works.bag", rosbag::bagmode::Write);
    foreach(rosbag::MessageInstance const& m, in.getMessageList()) {
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
    foreach(rosbag::MessageInstance const& m, in.getMessageList()) {
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
