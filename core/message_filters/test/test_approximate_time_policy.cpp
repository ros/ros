
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include <gtest/gtest.h>
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <vector>
#include <ros/ros.h>
//#include <pair>

using namespace message_filters;
using namespace message_filters::sync_policies;

struct Header
{
  ros::Time stamp;
};


struct Msg
{
  Header header;
  int data;
};
typedef boost::shared_ptr<Msg> MsgPtr;
typedef boost::shared_ptr<Msg const> MsgConstPtr;

namespace ros
{
namespace message_traits
{
template<>
struct TimeStamp<Msg>
{
  static ros::Time value(const Msg& m)
  {
    return m.header.stamp;
  }
};
}
}

typedef std::pair<ros::Time, ros::Time> TimePair;
typedef std::pair<ros::Time, unsigned int> TimeAndTopic;


//----------------------------------------------------------
//                   Test Class
//----------------------------------------------------------
class ApproximateTimeSynchronizerTest
{
public:

  ApproximateTimeSynchronizerTest(const std::vector<TimeAndTopic> &input,
      const std::vector<TimePair> &output) :
    input_(input), output_(output), output_position_(0)
  {
  }

  void callback(const MsgConstPtr& p, const MsgConstPtr& q)
  {
    //printf("Call_back called\n");
    //printf("Call back: <%f, %f>\n", p->header.stamp.toSec(), q->header.stamp.toSec());
    ASSERT_TRUE(p);
    ASSERT_TRUE(q);
    ASSERT_LT(output_position_, output_.size());
    EXPECT_EQ(output_[output_position_].first, p->header.stamp);
    EXPECT_EQ(output_[output_position_].second, q->header.stamp);
    ++output_position_;
  }

  void run()
  {
    Synchronizer<ApproximateTime<Msg, Msg> > sync(ApproximateTime<Msg, Msg>(1));
    sync.registerCallback(boost::bind(&ApproximateTimeSynchronizerTest::callback, this, _1, _2));
    for (unsigned int i = 0; i < input_.size(); i++)
    {
      if (input_[i].second == 0)
      {
        MsgPtr p(new Msg);
        p->header.stamp = input_[i].first;
        sync.add<0>(p);
      }
      else
      {
        MsgPtr q(new Msg);
        q->header.stamp = input_[i].first;
        sync.add<1>(q);
      }
    }
    //printf("Done running test\n");
    EXPECT_EQ(output_.size(), output_position_);
  }

private:
  const std::vector<TimeAndTopic> &input_;
  const std::vector<TimePair> &output_;
  unsigned int output_position_;

};


//----------------------------------------------------------
//                   Test Suite
//----------------------------------------------------------
TEST(ApproxTimeSync, PerfectMatch) {
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   ...a..b.
  //           ...A..B.
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*4,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*4));

  ApproximateTimeSynchronizerTest sync_test(input, output);
  sync_test.run();
}


TEST(ApproxTimeSync, ImperfectMatch) {
  // Input A:  a.xb..c.
  // Input B:  .A...B.C
  // Output:   ..a...c.
  //           ..A...B.
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+s*2,0)); // x
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*5,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*6, t+s*5));

  ApproximateTimeSynchronizerTest sync_test(input, output);
  sync_test.run();
}


TEST(ApproxTimeSync, Acceleration) {
  // Time:     0123456789012345678
  // Input A:  a...........b....c.
  // Input B:  .......A.......B..C
  // Output:   ............b.....c
  //           ............A.....C
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));      // a
  input.push_back(TimeAndTopic(t+s*7,1));  // A
  input.push_back(TimeAndTopic(t+s*12,0)); // b
  input.push_back(TimeAndTopic(t+s*15,1)); // B
  input.push_back(TimeAndTopic(t+s*17,0)); // c
  input.push_back(TimeAndTopic(t+s*18,1)); // C
  output.push_back(TimePair(t+s*12, t+s*7));
  output.push_back(TimePair(t+s*17, t+s*18));

  ApproximateTimeSynchronizerTest sync_test(input, output);
  sync_test.run();
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  printf("Preparing to test\n");


  return RUN_ALL_TESTS();
}
