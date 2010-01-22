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

#ifndef MESSAGE_FILTERS_TIME_SYNCHRONIZER_H
#define MESSAGE_FILTERS_TIME_SYNCHRONIZER_H

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/signals.hpp>
#include <boost/bind.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/noncopyable.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/mpl/and.hpp>
#include <boost/mpl/equal_to.hpp>
#include <boost/function_types/function_arity.hpp>
#include <boost/function_types/is_nonmember_callable_builtin.hpp>

#include <roslib/Header.h>

#include "connection.h"

#define TIME_SYNCHRONIZER_MAX_MESSAGES 9

namespace message_filters
{

namespace ft = boost::function_types;
namespace mpl = boost::mpl;

class NullType
{
public:
  roslib::Header header;
};
typedef boost::shared_ptr<NullType const> NullTypeConstPtr;

template<class M>
class NullFilter
{
public:
  typedef boost::shared_ptr<M const> MConstPtr;
  typedef boost::function<void(const MConstPtr&)> Callback;
  Connection registerCallback(const Callback& cb)
  {
    return Connection();
  }
};

/**
 * \brief Synchronizes up to 9 messages by their timestamps.
 *
 * TimeSynchronizer synchronizes up to 9 incoming channels by the timestamps contained in their messages' headers.
 * TimeSynchronizer takes anywhere from 2 to 9 message types as template parameters, and passes them through to a
 * callback which takes a shared pointer of each.
 *
 * The required queue size parameter when constructing the TimeSynchronizer tells it how many sets of messages it should
 * store (by timestamp) while waiting for messages to arrive and complete their "set"
 *
 * \section connections CONNECTIONS
 *
 * The input connections for the TimeSynchronizer object is the same signature as for roscpp subscription callbacks, ie.
\verbatim
void callback(const boost::shared_ptr<M const>&);
\endverbatim
 * The output connection for the TimeSynchronizer object is dependent on the number of messages being synchronized.  For
 * a 3-message synchronizer for example, it would be:
\verbatim
void callback(const boost::shared_ptr<M0 const>&, const boost::shared_ptr<M1 const>&, const boost::shared_ptr<M2 const>&);
\endverbatim
 * \section usage USAGE
 * Example usage would be:
\verbatim
TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image> sync(caminfo_sub, limage_sub, rimage_sub, 3);
sync.registerCallback(callback);
\endverbatim

 * The callback is then of the form:
\verbatim
void callback(const sensor_msgs::CameraInfo::ConstPtr&, const sensor_msgs::Image::ConstPtr&, const sensor_msgs::Image::ConstPtr&);
\endverbatim
 *
 */
template<class M0, class M1, class M2 = NullType, class M3 = NullType, class M4 = NullType,
         class M5 = NullType, class M6 = NullType, class M7 = NullType, class M8 = NullType>
class TimeSynchronizer : public boost::noncopyable
{
public:
  typedef boost::shared_ptr<M0 const> M0ConstPtr;
  typedef boost::shared_ptr<M1 const> M1ConstPtr;
  typedef boost::shared_ptr<M2 const> M2ConstPtr;
  typedef boost::shared_ptr<M3 const> M3ConstPtr;
  typedef boost::shared_ptr<M4 const> M4ConstPtr;
  typedef boost::shared_ptr<M5 const> M5ConstPtr;
  typedef boost::shared_ptr<M6 const> M6ConstPtr;
  typedef boost::shared_ptr<M7 const> M7ConstPtr;
  typedef boost::shared_ptr<M8 const> M8ConstPtr;
  typedef boost::tuple<M0ConstPtr, M1ConstPtr, M2ConstPtr, M3ConstPtr, M4ConstPtr, M5ConstPtr, M6ConstPtr, M7ConstPtr, M8ConstPtr> Tuple;
  typedef boost::signal<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&, const M3ConstPtr&, const M4ConstPtr&, const M5ConstPtr&, const M6ConstPtr&, const M7ConstPtr&, const M8ConstPtr&)> Signal;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&)> Callback2;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&)> Callback3;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&, const M3ConstPtr&)> Callback4;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&, const M3ConstPtr&, const M4ConstPtr&)> Callback5;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&, const M3ConstPtr&, const M4ConstPtr&, const M5ConstPtr&)> Callback6;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&, const M3ConstPtr&, const M4ConstPtr&, const M5ConstPtr&, const M6ConstPtr&)> Callback7;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&, const M3ConstPtr&, const M4ConstPtr&, const M5ConstPtr&, const M6ConstPtr&, const M7ConstPtr&)> Callback8;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&, const M3ConstPtr&, const M4ConstPtr&, const M5ConstPtr&, const M6ConstPtr&, const M7ConstPtr&, const M8ConstPtr&)> Callback9;

  template<class F0, class F1>
  TimeSynchronizer(F0& f0, F1& f1, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectInput(f0, f1);
  }

  template<class F0, class F1, class F2>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectInput(f0, f1, f2);
  }

  template<class F0, class F1, class F2, class F3>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectInput(f0, f1, f2, f3);
  }

  template<class F0, class F1, class F2, class F3, class F4>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectInput(f0, f1, f2, f3, f4);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectInput(f0, f1, f2, f3, f4, f5);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectInput(f0, f1, f2, f3, f4, f5, f6);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, F8& f8, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7, f8);
  }

  TimeSynchronizer(uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
  }

  ~TimeSynchronizer()
  {
    disconnectAll();
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

    input_connections_[0] = f0.registerCallback(boost::bind(&TimeSynchronizer::cb0, this, _1));
    input_connections_[1] = f1.registerCallback(boost::bind(&TimeSynchronizer::cb1, this, _1));
    input_connections_[2] = f2.registerCallback(boost::bind(&TimeSynchronizer::cb2, this, _1));
    input_connections_[3] = f3.registerCallback(boost::bind(&TimeSynchronizer::cb3, this, _1));
    input_connections_[4] = f4.registerCallback(boost::bind(&TimeSynchronizer::cb4, this, _1));
    input_connections_[5] = f5.registerCallback(boost::bind(&TimeSynchronizer::cb5, this, _1));
    input_connections_[6] = f6.registerCallback(boost::bind(&TimeSynchronizer::cb6, this, _1));
    input_connections_[7] = f7.registerCallback(boost::bind(&TimeSynchronizer::cb7, this, _1));
    input_connections_[8] = f8.registerCallback(boost::bind(&TimeSynchronizer::cb8, this, _1));
  }

  template<class C>
  typename boost::enable_if<mpl::and_<ft::is_nonmember_callable_builtin<C>,
                                      mpl::equal_to<ft::function_arity<C>, mpl::integral_c<size_t, 2> > >, Connection >::type registerCallback(const C& callback)
  {
    return registerCallback(boost::bind(Callback2(callback), _1, _2));
  }

  template<class C>
  typename boost::enable_if<mpl::and_<ft::is_nonmember_callable_builtin<C>,
                                      mpl::equal_to<ft::function_arity<C>, mpl::integral_c<size_t, 3> > >, Connection >::type registerCallback(const C& callback)
  {
    return registerCallback(boost::bind(Callback3(callback), _1, _2, _3));
  }

  template<class C>
  typename boost::enable_if<mpl::and_<ft::is_nonmember_callable_builtin<C>,
                                      mpl::equal_to<ft::function_arity<C>, mpl::integral_c<size_t, 4> > >, Connection >::type registerCallback(const C& callback)
  {
    return registerCallback(boost::bind(Callback4(callback), _1, _2, _3, _4));
  }

  template<class C>
  typename boost::enable_if<mpl::and_<ft::is_nonmember_callable_builtin<C>,
                                      mpl::equal_to<ft::function_arity<C>, mpl::integral_c<size_t, 5> > >, Connection >::type registerCallback(const C& callback)
  {
    return registerCallback(boost::bind(Callback5(callback), _1, _2, _3, _4, _5));
  }

  template<class C>
  typename boost::enable_if<mpl::and_<ft::is_nonmember_callable_builtin<C>,
                                      mpl::equal_to<ft::function_arity<C>, mpl::integral_c<size_t, 6> > >, Connection >::type registerCallback(const C& callback)
  {
    return registerCallback(boost::bind(Callback6(callback), _1, _2, _3, _4, _5, _6));
  }

  template<class C>
  typename boost::enable_if<mpl::and_<ft::is_nonmember_callable_builtin<C>,
                                      mpl::equal_to<ft::function_arity<C>, mpl::integral_c<size_t, 7> > >, Connection >::type registerCallback(const C& callback)
  {
    return registerCallback(boost::bind(Callback7(callback), _1, _2, _3, _4, _5, _6, _7));
  }

  template<class C>
  typename boost::enable_if<mpl::and_<ft::is_nonmember_callable_builtin<C>,
                                      mpl::equal_to<ft::function_arity<C>, mpl::integral_c<size_t, 8> > >, Connection >::type registerCallback(const C& callback)
  {
    return registerCallback(boost::bind(Callback8(callback), _1, _2, _3, _4, _5, _6, _7, _8));
  }

  template<class C>
  typename boost::enable_if<mpl::and_<ft::is_nonmember_callable_builtin<C>,
                                      mpl::equal_to<ft::function_arity<C>, mpl::integral_c<size_t, 9> > >, Connection >::type registerCallback(const C& callback)
  {
    return registerCallback(boost::bind(Callback9(callback), _1, _2, _3, _4, _5, _6, _7, _8, _9));
  }

  template<class C>
  typename boost::disable_if<ft::is_nonmember_callable_builtin<C>, Connection >::type registerCallback(const C& callback)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    return Connection(boost::bind(&TimeSynchronizer::disconnect, this, _1), signal_.connect(boost::bind(callback, _1, _2, _3, _4, _5, _6, _7, _8, _9)));
  }

  void add0(const M0ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<0>(t) = msg;

    checkTuple(t);
  }

  void add1(const M1ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<1>(t) = msg;

    checkTuple(t);
  }

  void add2(const M2ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<2>(t) = msg;

    checkTuple(t);
  }

  void add3(const M3ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<3>(t) = msg;

    checkTuple(t);
  }

  void add4(const M4ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<4>(t) = msg;

    checkTuple(t);
  }

  void add5(const M5ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<5>(t) = msg;

    checkTuple(t);
  }

  void add6(const M6ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<6>(t) = msg;

    checkTuple(t);
  }

  void add7(const M7ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<7>(t) = msg;

    checkTuple(t);
  }

  void add8(const M8ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<8>(t) = msg;

    checkTuple(t);
  }

  void setName(const std::string& name) { name_ = name; }
  const std::string& getName() { return name_; }

private:

  void disconnectAll()
  {
    for (int i = 0; i < TIME_SYNCHRONIZER_MAX_MESSAGES; ++i)
    {
      input_connections_[i].disconnect();
    }
  }

  void determineRealTypeCount()
  {
    real_type_count_ = 2;

    if (!boost::is_same<M2, NullType>::value)
    {
      ++real_type_count_;

      if (!boost::is_same<M3, NullType>::value)
      {
        ++real_type_count_;

        if (!boost::is_same<M4, NullType>::value)
        {
          ++real_type_count_;

          if (!boost::is_same<M5, NullType>::value)
          {
            ++real_type_count_;

            if (!boost::is_same<M6, NullType>::value)
            {
              ++real_type_count_;

              if (!boost::is_same<M7, NullType>::value)
              {
                ++real_type_count_;

                if (!boost::is_same<M8, NullType>::value)
                {
                  ++real_type_count_;
                }
              }
            }
          }
        }
      }
    }
  }

  void cb0(const M0ConstPtr& msg)
  {
    add0(msg);
  }

  void cb1(const M1ConstPtr& msg)
  {
    add1(msg);
  }

  void cb2(const M2ConstPtr& msg)
  {
    add2(msg);
  }

  void cb3(const M3ConstPtr& msg)
  {
    add3(msg);
  }

  void cb4(const M4ConstPtr& msg)
  {
    add4(msg);
  }

  void cb5(const M5ConstPtr& msg)
  {
    add5(msg);
  }

  void cb6(const M6ConstPtr& msg)
  {
    add6(msg);
  }

  void cb7(const M7ConstPtr& msg)
  {
    add7(msg);
  }

  void cb8(const M8ConstPtr& msg)
  {
    add8(msg);
  }

  // assumes tuples_mutex_ is already locked
  void checkTuple(Tuple& t)
  {
    bool full = true;
    full &= (bool)boost::get<0>(t);
    full &= (bool)boost::get<1>(t);
    full &= real_type_count_ > 2 ? (bool)boost::get<2>(t) : true;
    full &= real_type_count_ > 3 ? (bool)boost::get<3>(t) : true;
    full &= real_type_count_ > 4 ? (bool)boost::get<4>(t) : true;
    full &= real_type_count_ > 5 ? (bool)boost::get<5>(t) : true;
    full &= real_type_count_ > 6 ? (bool)boost::get<6>(t) : true;
    full &= real_type_count_ > 7 ? (bool)boost::get<7>(t) : true;
    full &= real_type_count_ > 8 ? (bool)boost::get<8>(t) : true;

    if (full)
    {
      {
        boost::mutex::scoped_lock lock(signal_mutex_);
        signal_(boost::get<0>(t), boost::get<1>(t), boost::get<2>(t), boost::get<3>(t), boost::get<4>(t), boost::get<5>(t), boost::get<6>(t), boost::get<7>(t), boost::get<8>(t));

        last_signal_time_ = boost::get<0>(t)->header.stamp;
      }

      tuples_.erase(last_signal_time_);

      clearOldTuples();
    }

    if (queue_size_ > 0)
    {
      while (tuples_.size() > queue_size_)
      {
        tuples_.erase(tuples_.begin());
      }
    }
  }

  // assumes tuples_mutex_ is already locked
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
        tuples_.erase(old);
      }
      else
      {
        // the map is sorted by time, so we can ignore anything after this if this one's time is ok
        break;
      }
    }
  }

  void disconnect(const Connection& c)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    c.getBoostConnection().disconnect();
  }

  uint32_t queue_size_;

  typedef std::map<ros::Time, Tuple> M_TimeToTuple;
  M_TimeToTuple tuples_;
  boost::mutex tuples_mutex_;

  Signal signal_;
  boost::mutex signal_mutex_;
  ros::Time last_signal_time_;

  Connection input_connections_[TIME_SYNCHRONIZER_MAX_MESSAGES];

  uint32_t real_type_count_;

  std::string name_;
};

}

#endif // MESSAGE_FILTERS_TIME_SYNCHRONIZER_H
