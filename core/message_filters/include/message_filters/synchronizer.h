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

#ifndef MESSAGE_FILTERS_SYNCHRONIZER_H
#define MESSAGE_FILTERS_SYNCHRONIZER_H

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

    input_connections_[0] = f0.registerCallback(boost::function<void(const M0Event&)>(boost::bind(&Synchronizer::template cb<0>, this, _1)));
    input_connections_[1] = f1.registerCallback(boost::function<void(const M1Event&)>(boost::bind(&Synchronizer::template cb<1>, this, _1)));
    input_connections_[2] = f2.registerCallback(boost::function<void(const M2Event&)>(boost::bind(&Synchronizer::template cb<2>, this, _1)));
    input_connections_[3] = f3.registerCallback(boost::function<void(const M3Event&)>(boost::bind(&Synchronizer::template cb<3>, this, _1)));
    input_connections_[4] = f4.registerCallback(boost::function<void(const M4Event&)>(boost::bind(&Synchronizer::template cb<4>, this, _1)));
    input_connections_[5] = f5.registerCallback(boost::function<void(const M5Event&)>(boost::bind(&Synchronizer::template cb<5>, this, _1)));
    input_connections_[6] = f6.registerCallback(boost::function<void(const M6Event&)>(boost::bind(&Synchronizer::template cb<6>, this, _1)));
    input_connections_[7] = f7.registerCallback(boost::function<void(const M7Event&)>(boost::bind(&Synchronizer::template cb<7>, this, _1)));
    input_connections_[8] = f8.registerCallback(boost::function<void(const M8Event&)>(boost::bind(&Synchronizer::template cb<8>, this, _1)));
  }

  template<class C>
  Connection registerCallback(C& callback)
  {
    return signal_.template addCallback(callback);
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

  template<class C, typename T>
  Connection registerCallback(C& callback, T* t)
  {
    return signal_.template addCallback(callback, t);
  }

  void setName(const std::string& name) { name_ = name; }
  const std::string& getName() { return name_; }


  void signal(const M0Event& e0, const M1Event& e1, const M2Event& e2, const M3Event& e3, const M4Event& e4,
              const M5Event& e5, const M6Event& e6, const M7Event& e7, const M8Event& e8)
  {
    signal_.call(e0, e1, e2, e3, e4, e5, e6, e7, e8);
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

  Connection input_connections_[MAX_MESSAGES];

  std::string name_;
};

template<typename M0, typename M1, typename M2, typename M3, typename M4,
         typename M5, typename M6, typename M7, typename M8>
struct PolicyBase
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

} // namespace message_filters

#endif // MESSAGE_FILTERS_SYNCHRONIZER_H
