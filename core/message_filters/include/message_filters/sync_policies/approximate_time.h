/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef MESSAGE_FILTERS_SYNC_APPROXIMATE_TIME_H
#define MESSAGE_FILTERS_SYNC_APPROXIMATE_TIME_H

#include "message_filters/synchronizer.h"
#include "message_filters/connection.h"
#include "message_filters/null_types.h"
#include "message_filters/signal9.h"

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/signals.hpp>
#include <boost/bind.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/noncopyable.hpp>
#include <boost/mpl/or.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/vector.hpp>

#include <roslib/Header.h>

#include <ros/message_traits.h>
#include <ros/message_event.h>

#include <deque>
#include <vector>
#include <string>

namespace message_filters
{
namespace sync_policies
{

namespace mpl = boost::mpl;

template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct ApproximateTime : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<ApproximateTime> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;
  typedef typename Super::M0Event M0Event;
  typedef typename Super::M1Event M1Event;
  typedef typename Super::M2Event M2Event;
  typedef typename Super::M3Event M3Event;
  typedef typename Super::M4Event M4Event;
  typedef typename Super::M5Event M5Event;
  typedef typename Super::M6Event M6Event;
  typedef typename Super::M7Event M7Event;
  typedef typename Super::M8Event M8Event;
  typedef std::deque<M0Event> M0Deque;
  typedef std::deque<M1Event> M1Deque;
  typedef std::deque<M2Event> M2Deque;
  typedef std::deque<M3Event> M3Deque;
  typedef std::deque<M4Event> M4Deque;
  typedef std::deque<M5Event> M5Deque;
  typedef std::deque<M6Event> M6Deque;
  typedef std::deque<M7Event> M7Deque;
  typedef std::deque<M8Event> M8Deque;
  typedef std::vector<M0Event> M0Vector;
  typedef std::vector<M1Event> M1Vector;
  typedef std::vector<M2Event> M2Vector;
  typedef std::vector<M3Event> M3Vector;
  typedef std::vector<M4Event> M4Vector;
  typedef std::vector<M5Event> M5Vector;
  typedef std::vector<M6Event> M6Vector;
  typedef std::vector<M7Event> M7Vector;
  typedef std::vector<M8Event> M8Vector;
  typedef boost::tuple<M0Event, M1Event, M2Event, M3Event, M4Event, M5Event, M6Event, M7Event, M8Event> Tuple;
  typedef boost::tuple<M0Deque, M1Deque, M2Deque, M3Deque, M4Deque, M5Deque, M6Deque, M7Deque, M8Deque> DequeTuple;
  typedef boost::tuple<M0Vector, M1Vector, M2Vector, M3Vector, M4Vector, M5Vector, M6Vector, M7Vector, M8Vector> VectorTuple;

  ApproximateTime(uint32_t queue_size)
  : parent_(0)
  , queue_size_(queue_size)
  , num_non_empty_deques_(0)
  , pivot_(NO_PIVOT)
  , max_interval_duration_(ros::DURATION_MAX)
  , epsilon_(0.1)
  {
  }

  ApproximateTime(const ApproximateTime& e)
  {
    *this = e;
  }

  ApproximateTime& operator=(const ApproximateTime& rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    num_non_empty_deques_ = rhs.num_non_empty_deques_;
    pivot_ = rhs.pivot_;
    max_interval_duration_ = rhs.max_interval_duration_;
    epsilon_ = rhs.epsilon_;
    candidate_start_ = rhs.candidate_start_;
    candidate_end_ = rhs.candidate_end_;
    deques_ = rhs.deques_;
    past_ = rhs.past_;

    return *this;
  }

  void initParent(Sync* parent)
  {
    parent_ = parent;
  }

  template<int i>
  void add(const typename mpl::at_c<Events, i>::type& evt)
  {
    boost::mutex::scoped_lock lock(data_mutex_);

    std::deque<typename mpl::at_c<Events, i>::type>& deque = boost::get<i>(deques_);
    deque.push_back(evt);
    if (deque.size() == (size_t)1) {
      // We have just added the first message, so it was empty before
      ++num_non_empty_deques_;
      if (num_non_empty_deques_ == (uint32_t)RealTypeCount::value)
      {
        // All deques have messages
        process();
      }
    }
  }

  void setEpsilon(double epsilon)
  {
    // For correctness we only need epsilon > -1.0, but most likely a negative epsilon is a mistake.
    ROS_ASSERT(epsilon >= 0);
    epsilon_ = epsilon;
  }

  void setMaxIntervalDuration(ros::Duration max_interval_duration) {
    // For correctness we only need epsilon > -1.0, but most likely a negative epsilon is a mistake.
    ROS_ASSERT(max_interval_duration >= ros::Duration(0,0));
    max_interval_duration_ = max_interval_duration;
  }

private:
  // Assumes that deque number <index> is non empty
  template<int i>
  void dequeDeleteFront()
  {
    std::deque<typename mpl::at_c<Events, i>::type>& deque = boost::get<i>(deques_);
    ROS_ASSERT(!deque.empty());
    deque.pop_front();
    if (deque.empty())
    {
      --num_non_empty_deques_;
    }
  }

  // Assumes that deque number <index> is non empty
  void dequeDeleteFront(uint32_t index)
  {
    switch (index)
    {
    case 0:
      dequeDeleteFront<0>();
      break;
    case 1:
      dequeDeleteFront<1>();
      break;
    case 2:
      dequeDeleteFront<2>();
      break;
    case 3:
      dequeDeleteFront<3>();
      break;
    case 4:
      dequeDeleteFront<4>();
      break;
    case 5:
      dequeDeleteFront<5>();
      break;
    case 6:
      dequeDeleteFront<6>();
      break;
    case 7:
      dequeDeleteFront<7>();
      break;
    case 8:
      dequeDeleteFront<8>();
      break;
    default:
      ROS_BREAK();
    }
  }

  // Assumes that deque number <index> is non empty
  template<int i>
  void dequeMoveFrontToPast()
  {
    std::deque<typename mpl::at_c<Events, i>::type>& deque = boost::get<i>(deques_);
    std::vector<typename mpl::at_c<Events, i>::type>& vector = boost::get<i>(past_);
    ROS_ASSERT(!deque.empty());
    vector.push_back(deque.front());
    deque.pop_front();
    if (deque.empty())
    {
      --num_non_empty_deques_;
    }
  }
  // Assumes that deque number <index> is non empty
  void dequeMoveFrontToPast(uint32_t index)
  {
    switch (index)
    {
    case 0:
      dequeMoveFrontToPast<0>();
      break;
    case 1:
      dequeMoveFrontToPast<1>();
      break;
    case 2:
      dequeMoveFrontToPast<2>();
      break;
    case 3:
      dequeMoveFrontToPast<3>();
      break;
    case 4:
      dequeMoveFrontToPast<4>();
      break;
    case 5:
      dequeMoveFrontToPast<5>();
      break;
    case 6:
      dequeMoveFrontToPast<6>();
      break;
    case 7:
      dequeMoveFrontToPast<7>();
      break;
    case 8:
      dequeMoveFrontToPast<8>();
      break;
    default:
      ROS_BREAK();
    }
  }

  void makeCandidate()
  {
    //printf("Creating candidate\n");
    // Create candidate tuple
    candidate_ = Tuple(); // Discards old one if any
    boost::get<0>(candidate_) = boost::get<0>(deques_).front();
    boost::get<1>(candidate_) = boost::get<1>(deques_).front();
    if (RealTypeCount::value > 2)
    {
      boost::get<2>(candidate_) = boost::get<2>(deques_).front();
      if (RealTypeCount::value > 3)
      {
	boost::get<3>(candidate_) = boost::get<3>(deques_).front();
	if (RealTypeCount::value > 4)
	{
	  boost::get<4>(candidate_) = boost::get<4>(deques_).front();
	  if (RealTypeCount::value > 5)
	  {
	    boost::get<5>(candidate_) = boost::get<5>(deques_).front();
	    if (RealTypeCount::value > 6)
	    {
	      boost::get<6>(candidate_) = boost::get<6>(deques_).front();
	      if (RealTypeCount::value > 7)
	      {
		boost::get<7>(candidate_) = boost::get<7>(deques_).front();
		if (RealTypeCount::value > 8)
		{
		  boost::get<8>(candidate_) = boost::get<8>(deques_).front();
		}
	      }
	    }
	  }
	}
      }
    }
    // Delete all past messages, since we have found a better candidate
    boost::get<0>(past_).clear();
    boost::get<1>(past_).clear();
    boost::get<2>(past_).clear();
    boost::get<3>(past_).clear();
    boost::get<4>(past_).clear();
    boost::get<5>(past_).clear();
    boost::get<6>(past_).clear();
    boost::get<7>(past_).clear();
    boost::get<8>(past_).clear();
    //printf("Candidate created\n");
  }

  template<int i>
  void recoverAndDelete()
  {
    if (i >= RealTypeCount::value)
    {
      return;
    }

    std::vector<typename mpl::at_c<Events, i>::type>& v = boost::get<i>(past_);
    std::deque<typename mpl::at_c<Events, i>::type>& q = boost::get<i>(deques_);
    while (!v.empty())
    {
      q.push_front(v.back());
      v.pop_back();
    }

    ROS_ASSERT(!q.empty());

    q.pop_front();
    if (!q.empty())
    {
      ++num_non_empty_deques_;
    }
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == RealTypeCount::value
  void publishCandidate()
  {
    //printf("Publishing candidate\n");
    // Publish
    parent_->signal(boost::get<0>(candidate_), boost::get<1>(candidate_), boost::get<2>(candidate_), boost::get<3>(candidate_),
                    boost::get<4>(candidate_), boost::get<5>(candidate_), boost::get<6>(candidate_), boost::get<7>(candidate_),
                    boost::get<8>(candidate_));
    // Delete this candidate
    candidate_ = Tuple();
    pivot_ = NO_PIVOT;
    num_non_empty_deques_ = 0; // We will recompute it from scratch

    // Recover hidden messages, and delete the ones corresponding to the candidate
    recoverAndDelete<0>();
    recoverAndDelete<1>();
    recoverAndDelete<2>();
    recoverAndDelete<3>();
    recoverAndDelete<4>();
    recoverAndDelete<5>();
    recoverAndDelete<6>();
    recoverAndDelete<7>();
    recoverAndDelete<8>();
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == RealTypeCount::value
  // Returns: the oldest message on the deques
  void getCandidateStart(uint32_t &start_index, ros::Time &start_time)
  {
    return getCandidateBoundary(start_index, start_time, false);
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == RealTypeCount::value
  // Returns: the latest message among the heads of the deques, i.e. the minimum
  //          time to end an interval started at getCandidateStart_index()
  void getCandidateEnd(uint32_t &end_index, ros::Time &end_time)
  {
    return getCandidateBoundary(end_index, end_time, true);
  }

  // end = true: look for the latest head of deque
  //       false: look for the earliest head of deque
  void getCandidateBoundary(uint32_t &index, ros::Time &time, bool end)
  {
    namespace mt = ros::message_traits;

    M0Event& m0 = boost::get<0>(deques_).front();
    M1Event& m1 = boost::get<1>(deques_).front();
    M2Event& m2 = boost::get<2>(deques_).front();
    M3Event& m3 = boost::get<3>(deques_).front();
    M4Event& m4 = boost::get<4>(deques_).front();
    M5Event& m5 = boost::get<5>(deques_).front();
    M6Event& m6 = boost::get<6>(deques_).front();
    M7Event& m7 = boost::get<7>(deques_).front();
    M8Event& m8 = boost::get<8>(deques_).front();
    time = mt::TimeStamp<M0>::value(*m0.getMessage());
    index = 0;
    if ((RealTypeCount::value > 1) && ((mt::TimeStamp<M1>::value(*m1.getMessage()) < time) ^ end))
    {
      time = mt::TimeStamp<M1>::value(*m1.getMessage());
      index = 1;
    }
    if ((RealTypeCount::value > 2) && ((mt::TimeStamp<M2>::value(*m2.getMessage()) < time) ^ end))
    {
      time = mt::TimeStamp<M2>::value(*m2.getMessage());
      index = 2;
    }
    if ((RealTypeCount::value > 3) && ((mt::TimeStamp<M3>::value(*m3.getMessage()) < time) ^ end))
    {
      time = mt::TimeStamp<M3>::value(*m3.getMessage());
      index = 3;
    }
    if ((RealTypeCount::value > 4) && ((mt::TimeStamp<M4>::value(*m4.getMessage()) < time) ^ end))
    {
      time = mt::TimeStamp<M4>::value(*m4.getMessage());
      index = 4;
    }
    if ((RealTypeCount::value > 5) && ((mt::TimeStamp<M5>::value(*m5.getMessage()) < time) ^ end))
    {
      time = mt::TimeStamp<M5>::value(*m5.getMessage());
      index = 5;
    }
    if ((RealTypeCount::value > 6) && ((mt::TimeStamp<M6>::value(*m6.getMessage()) < time) ^ end))
    {
      time = mt::TimeStamp<M6>::value(*m6.getMessage());
      index = 6;
    }
    if ((RealTypeCount::value > 7) && ((mt::TimeStamp<M7>::value(*m7.getMessage()) < time) ^ end))
    {
      time = mt::TimeStamp<M7>::value(*m7.getMessage());
      index = 7;
    }
    if ((RealTypeCount::value > 8) && ((mt::TimeStamp<M8>::value(*m8.getMessage()) < time) ^ end))
    {
      time = mt::TimeStamp<M8>::value(*m8.getMessage());
      index = 8;
    }
  }

  // assumes data_mutex_ is already locked
  void process()
  {
    // While no deque is empty
    while (num_non_empty_deques_ == (uint32_t)RealTypeCount::value)
    {
      // Find the start and end of the current interval
      //printf("Entering while loop in this state [\n");
      //show_internal_state();
      //printf("]\n");
      ros::Time end_time, start_time;
      uint32_t end_index;
      uint32_t start_index;
      getCandidateEnd(end_index, end_time);
      getCandidateStart(start_index, start_time);
      if (pivot_ == NO_PIVOT)
      {
        // We do not have a candidate
        // INVARIANT: the past_ vectors are empty
        // INVARIANT: (candidate_ has no filled members)
        if (end_time - start_time > max_interval_duration_)
        {
          // This interval is too big to be a valid candidate, move to the next
          dequeDeleteFront(start_index);
          continue;
        }
        else
        {
          // This is a valid candidate, and we don't have any, so take it
          makeCandidate();
          candidate_start_ = start_time;
          candidate_end_ = end_time;
          pivot_ = end_index;
          dequeMoveFrontToPast(start_index);
          //intf("After dequeMoveFrontToPast\n");
        }
      }
      else
      {
        // We already have a candidate
        // Is this one better than the current candidate?
        if ((end_time - candidate_end_) * (1 + epsilon_) - (start_time
            - candidate_start_) > ros::Duration(0))
        {
          // This is not a better candidate, move to the next
          dequeMoveFrontToPast(start_index);
        }
        else
        {
          // This is a better candidate
          makeCandidate();
          candidate_start_ = start_time;
          candidate_end_ = end_time;
          dequeMoveFrontToPast(start_index);
          // Keep the same pivot
        }
      }
      // INVARIANT: we have a candidate and pivot
      //printf("start_index == %d, pivot_ == %d\n", start_index, pivot_);
      if (start_index == pivot_)
      {
        // We have exhausted all possible candidates for this pivot, we now can output the best one
        publishCandidate();
      }

    }
  }

  Sync* parent_;
  uint32_t queue_size_;

  static const uint32_t NO_PIVOT = 9;  // Special value for the pivot indicating that no pivot has been selected

  DequeTuple deques_;
  uint32_t num_non_empty_deques_;
  VectorTuple past_;
  Tuple candidate_;  // NULL if there is no candidate, in which case there is no pivot.
  ros::Time candidate_start_;
  ros::Time candidate_end_;
  uint32_t pivot_;  // Equal to NO_PIVOT if there is no candidate
  boost::mutex data_mutex_;  // Protects all of the above

  ros::Duration max_interval_duration_; // TODO: initialize with a parameter
  double epsilon_;
};

} // namespace sync
} // namespace message_filters

#endif // MESSAGE_FILTERS_SYNC_APPROXIMATE_TIME_H

