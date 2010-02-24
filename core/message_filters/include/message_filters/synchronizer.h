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

#ifndef MESSAGE_FILTERS_STATIC_TIME_SYNCHRONIZER_H
#define MESSAGE_FILTERS_STATIC_TIME_SYNCHRONIZER_H

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
#include <boost/function_types/function_arity.hpp>
#include <boost/function_types/is_nonmember_callable_builtin.hpp>

#include <roslib/Header.h>

#include "connection.h"
#include "null_types.h"
#include "signal9.h"
#include <ros/message_traits.h>
#include <ros/message_event.h>

#include <deque>
#include <vector>
#include <string>

namespace message_filters
{

namespace mpl = boost::mpl;

template<class Policy>
class Synchronizer : public boost::noncopyable, public Policy
{
public:
  typedef typename Policy::Messages Messages;
  typedef typename Policy::Events Events;
  typedef typename Policy::Signal Signal;
  typedef typename mpl::at_c<Messages, 0>::type M0;
  typedef typename mpl::at_c<Messages, 1>::type M1;
  typedef typename mpl::at_c<Messages, 2>::type M2;
  typedef typename mpl::at_c<Messages, 3>::type M3;
  typedef typename mpl::at_c<Messages, 4>::type M4;
  typedef typename mpl::at_c<Messages, 5>::type M5;
  typedef typename mpl::at_c<Messages, 6>::type M6;
  typedef typename mpl::at_c<Messages, 7>::type M7;
  typedef typename mpl::at_c<Messages, 8>::type M8;
  typedef typename mpl::at_c<Events, 0>::type M0Event;
  typedef typename mpl::at_c<Events, 1>::type M1Event;
  typedef typename mpl::at_c<Events, 2>::type M2Event;
  typedef typename mpl::at_c<Events, 3>::type M3Event;
  typedef typename mpl::at_c<Events, 4>::type M4Event;
  typedef typename mpl::at_c<Events, 5>::type M5Event;
  typedef typename mpl::at_c<Events, 6>::type M6Event;
  typedef typename mpl::at_c<Events, 7>::type M7Event;
  typedef typename mpl::at_c<Events, 8>::type M8Event;

  static const uint8_t MAX_MESSAGES = 9;

  template<class F0, class F1>
  Synchronizer(F0& f0, F1& f1)
  {
    connectInput(f0, f1);
    init();
  }

  template<class F0, class F1, class F2>
  Synchronizer(F0& f0, F1& f1, F2& f2)
  {
    connectInput(f0, f1, f2);
    init();
  }

  template<class F0, class F1, class F2, class F3>
  Synchronizer(F0& f0, F1& f1, F2& f2, F3& f3)
  {
    connectInput(f0, f1, f2, f3);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4>
  Synchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4)
  {
    connectInput(f0, f1, f2, f3, f4);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5>
  Synchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5)
  {
    connectInput(f0, f1, f2, f3, f4, f5);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  Synchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  Synchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  Synchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, F8& f8)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7, f8);
    init();
  }

  Synchronizer()
  {
    init();
  }

  template<class F0, class F1>
  Synchronizer(const Policy& policy, F0& f0, F1& f1)
  : Policy(policy)
  {
    connectInput(f0, f1);
    init();
  }

  template<class F0, class F1, class F2>
  Synchronizer(const Policy& policy, F0& f0, F1& f1, F2& f2)
  : Policy(policy)
  {
    connectInput(f0, f1, f2);
    init();
  }

  template<class F0, class F1, class F2, class F3>
  Synchronizer(const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4>
  Synchronizer(const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5>
  Synchronizer(const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4, f5);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  Synchronizer(const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  Synchronizer(const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7);
    init();
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  Synchronizer(const Policy& policy, F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, F8& f8)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7, f8);
    init();
  }

  Synchronizer(const Policy& policy)
  : Policy(policy)
  {
    init();
  }

  ~Synchronizer()
  {
    disconnectAll();
  }

  void init()
  {
    Policy::initParent(this);
  }

  template<class F0, class F1>
  void connectInput(F0& f0, F1& f1)
  {
    NullFilter<M2> f2;
    connectInput(f0, f1, f2);
  }

  template<class F0, class F1, class F2>
  void connectInput(F0& f0, F1& f1, F2& f2)
  {
    NullFilter<M3> f3;
    connectInput(f0, f1, f2, f3);
  }

  template<class F0, class F1, class F2, class F3>
  void connectInput(F0& f0, F1& f1, F2& f2, F3& f3)
  {
    NullFilter<M4> f4;
    connectInput(f0, f1, f2, f3, f4);
  }

  template<class F0, class F1, class F2, class F3, class F4>
  void connectInput(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4)
  {
    NullFilter<M5> f5;
    connectInput(f0, f1, f2, f3, f4, f5);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5>
  void connectInput(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5)
  {
    NullFilter<M6> f6;
    connectInput(f0, f1, f2, f3, f4, f5, f6);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  void connectInput(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6)
  {
    NullFilter<M7> f7;
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  void connectInput(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7)
  {
    NullFilter<M8> f8;
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7, f8);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  void connectInput(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, F8& f8)
  {
    disconnectAll();

    input_connections_[0] = f0.registerCallback(boost::function<void(const M0Event&)>(boost::bind(&Synchronizer::cb<0>, this, _1)));
    input_connections_[1] = f1.registerCallback(boost::function<void(const M1Event&)>(boost::bind(&Synchronizer::cb<1>, this, _1)));
    input_connections_[2] = f2.registerCallback(boost::function<void(const M2Event&)>(boost::bind(&Synchronizer::cb<2>, this, _1)));
    input_connections_[3] = f3.registerCallback(boost::function<void(const M3Event&)>(boost::bind(&Synchronizer::cb<3>, this, _1)));
    input_connections_[4] = f4.registerCallback(boost::function<void(const M4Event&)>(boost::bind(&Synchronizer::cb<4>, this, _1)));
    input_connections_[5] = f5.registerCallback(boost::function<void(const M5Event&)>(boost::bind(&Synchronizer::cb<5>, this, _1)));
    input_connections_[6] = f6.registerCallback(boost::function<void(const M6Event&)>(boost::bind(&Synchronizer::cb<6>, this, _1)));
    input_connections_[7] = f7.registerCallback(boost::function<void(const M7Event&)>(boost::bind(&Synchronizer::cb<7>, this, _1)));
    input_connections_[8] = f8.registerCallback(boost::function<void(const M8Event&)>(boost::bind(&Synchronizer::cb<8>, this, _1)));
  }

  template<class C>
  Connection registerCallback(const C& callback)
  {
    return signal_.template addCallback(callback);
  }

  template<class C, typename T>
  Connection registerCallback(const C& callback, T* t)
  {
    return signal_.template addCallback(callback, t);
  }

  template<class C>
  Connection registerDropCallback(const C& callback)
  {
    return drop_signal_.template addCallback(callback);
  }

  template<class C, typename T>
  Connection registerDropCallback(const C& callback, T* t)
  {
    return drop_signal_.template addCallback(callback, t);
  }

  void setName(const std::string& name) { name_ = name; }
  const std::string& getName() { return name_; }


  void signal(const M0Event& e0, const M1Event& e1, const M2Event& e2, const M3Event& e3, const M4Event& e4,
              const M5Event& e5, const M6Event& e6, const M7Event& e7, const M8Event& e8)
  {
    signal_.call(e0, e1, e2, e3, e4, e5, e6, e7, e8);
  }

  void signalDrop(const M0Event& e0, const M1Event& e1, const M2Event& e2, const M3Event& e3, const M4Event& e4,
              const M5Event& e5, const M6Event& e6, const M7Event& e7, const M8Event& e8)
  {
    drop_signal_.call(e0, e1, e2, e3, e4, e5, e6, e7, e8);
  }

  Policy* getPolicy() { return static_cast<Policy*>(this); }

  using Policy::add;

  template<int i>
  void add(const boost::shared_ptr<typename mpl::at_c<Messages, i>::type const>& msg)
  {
    this->template add<i>(typename mpl::at_c<Events, i>::type(msg));
  }

private:

  void disconnectAll()
  {
    for (int i = 0; i < MAX_MESSAGES; ++i)
    {
      input_connections_[i].disconnect();
    }
  }

  template<int i>
  void cb(const typename mpl::at_c<Events, i>::type& evt)
  {
    add<i>(evt);
  }

  uint32_t queue_size_;

  Signal signal_;
  Signal drop_signal_;

  Connection input_connections_[MAX_MESSAGES];

  std::string name_;
};

template<typename M0, typename M1, typename M2, typename M3, typename M4,
         typename M5, typename M6, typename M7, typename M8>
struct StaticTimePolicyBase
{
  typedef mpl::vector<M0, M1, M2, M3, M4, M5, M6, M7, M8> Messages;
  typedef Signal9<M0, M1, M2, M3, M4, M5, M6, M7, M8> Signal;
  typedef mpl::vector<ros::MessageEvent<M0 const>, ros::MessageEvent<M1 const>, ros::MessageEvent<M2 const>, ros::MessageEvent<M3 const>,
                      ros::MessageEvent<M4 const>, ros::MessageEvent<M5 const>, ros::MessageEvent<M6 const>, ros::MessageEvent<M7 const>,
                      ros::MessageEvent<M8 const> > Events;
  typedef typename mpl::fold<Messages, mpl::int_<0>, mpl::if_<mpl::not_<boost::is_same<mpl::_2, NullType> >, mpl::next<mpl::_1>, mpl::_1> >::type RealTypeCount;
  typedef typename mpl::at_c<Events, 0>::type M0Event;
  typedef typename mpl::at_c<Events, 1>::type M1Event;
  typedef typename mpl::at_c<Events, 2>::type M2Event;
  typedef typename mpl::at_c<Events, 3>::type M3Event;
  typedef typename mpl::at_c<Events, 4>::type M4Event;
  typedef typename mpl::at_c<Events, 5>::type M5Event;
  typedef typename mpl::at_c<Events, 6>::type M6Event;
  typedef typename mpl::at_c<Events, 7>::type M7Event;
  typedef typename mpl::at_c<Events, 8>::type M8Event;
};

template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct ExactTime : public StaticTimePolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<ExactTime> Sync;
  typedef StaticTimePolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
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
  typedef boost::tuple<M0Event, M1Event, M2Event, M3Event, M4Event, M5Event, M6Event, M7Event, M8Event> Tuple;

  ExactTime(uint32_t queue_size)
  : parent_(0)
  , queue_size_(queue_size)
  {
  }

  ExactTime(const ExactTime& e)
  {
    *this = e;
  }

  ExactTime& operator=(const ExactTime& rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    last_signal_time_ = rhs.last_signal_time_;
    tuples_ = rhs.tuples_;

    return *this;
  }

  void initParent(Sync* parent)
  {
    parent_ = parent;
  }

  template<int i>
  void add(const typename mpl::at_c<Events, i>::type& evt)
  {
    ROS_ASSERT(parent_);

    namespace mt = ros::message_traits;

    boost::mutex::scoped_lock lock(mutex_);

    Tuple& t = tuples_[mt::TimeStamp<typename mpl::at_c<Messages, i>::type>::value(*evt.getMessage())];
    boost::get<i>(t) = evt;

    checkTuple(t);
  }

private:

  // assumes mutex_ is already locked
  void checkTuple(Tuple& t)
  {
    namespace mt = ros::message_traits;

    bool full = true;
    full = full && (bool)boost::get<0>(t).getMessage();
    full = full && (bool)boost::get<1>(t).getMessage();
    full = full && (RealTypeCount::value > 2 ? (bool)boost::get<2>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 3 ? (bool)boost::get<3>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 4 ? (bool)boost::get<4>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 5 ? (bool)boost::get<5>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 6 ? (bool)boost::get<6>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 7 ? (bool)boost::get<7>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 8 ? (bool)boost::get<8>(t).getMessage() : true);

    if (full)
    {
      parent_->signal(boost::get<0>(t), boost::get<1>(t), boost::get<2>(t),
                       boost::get<3>(t), boost::get<4>(t), boost::get<5>(t),
                       boost::get<6>(t), boost::get<7>(t), boost::get<8>(t));

      last_signal_time_ = mt::TimeStamp<M0>::value(*boost::get<0>(t).getMessage());

      tuples_.erase(last_signal_time_);

      clearOldTuples();
    }

    if (queue_size_ > 0)
    {
      while (tuples_.size() > queue_size_)
      {
        Tuple& t2 = tuples_.begin()->second;
        parent_->signalDrop(boost::get<0>(t2), boost::get<1>(t2), boost::get<2>(t2),
                            boost::get<3>(t2), boost::get<4>(t2), boost::get<5>(t2),
                            boost::get<6>(t2), boost::get<7>(t2), boost::get<8>(t2));
        tuples_.erase(tuples_.begin());
      }
    }
  }

  // assumes mutex_ is already locked
  void clearOldTuples()
  {
    typename M_TimeToTuple::iterator it = tuples_.begin();
    typename M_TimeToTuple::iterator end = tuples_.end();
    for (; it != end;)
    {
      if (it->first <= last_signal_time_)
      {
        typename M_TimeToTuple::iterator old = it;
        ++it;

        Tuple& t = old->second;
        parent_->signalDrop(boost::get<0>(t), boost::get<1>(t), boost::get<2>(t),
                            boost::get<3>(t), boost::get<4>(t), boost::get<5>(t),
                            boost::get<6>(t), boost::get<7>(t), boost::get<8>(t));
        tuples_.erase(old);
      }
      else
      {
        // the map is sorted by time, so we can ignore anything after this if this one's time is ok
        break;
      }
    }
  }

private:
  Sync* parent_;

  uint32_t queue_size_;
  typedef std::map<ros::Time, Tuple> M_TimeToTuple;
  M_TimeToTuple tuples_;
  ros::Time last_signal_time_;

  boost::mutex mutex_;
};

template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct ApproximateTime : public StaticTimePolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<ApproximateTime> Sync;
  typedef StaticTimePolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
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
    }
    else if (RealTypeCount::value > 3)
    {
      boost::get<3>(candidate_) = boost::get<3>(deques_).front();
    }
    else if (RealTypeCount::value > 4)
    {
      boost::get<4>(candidate_) = boost::get<4>(deques_).front();
    }
    else if (RealTypeCount::value > 5)
    {
      boost::get<5>(candidate_) = boost::get<5>(deques_).front();
    }
    else if (RealTypeCount::value > 6)
    {
      boost::get<6>(candidate_) = boost::get<6>(deques_).front();
    }
    else if (RealTypeCount::value > 7)
    {
      boost::get<7>(candidate_) = boost::get<7>(deques_).front();
    }
    else if (RealTypeCount::value > 8)
    {
      boost::get<8>(candidate_) = boost::get<8>(deques_).front();
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
    if (RealTypeCount::value >= i)
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

}

#endif // MESSAGE_FILTERS_TIME_SYNCHRONIZER_H
