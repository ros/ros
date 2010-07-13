/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef ROS_TIME_H
#define ROS_TIME_H

#include "duration.h"

#include <iostream>
#include <math.h>

//Ros types also needs to be moved out of roscpp...
//#include "ros/types.h"

#ifdef WIN32
#include <windows.h>
#endif

namespace ros
{

inline void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
{
  uint64_t nsec_part = nsec % 1000000000UL;
  uint64_t sec_part = nsec / 1000000000UL;

  if (sec_part > UINT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec += sec_part;
  nsec = nsec_part;
}

inline void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
{
  uint64_t sec64 = sec;
  uint64_t nsec64 = nsec;

  normalizeSecNSec(sec64, nsec64);

  sec = (uint32_t)sec64;
  nsec = (uint32_t)nsec64;
}

inline void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
{
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part >= 1000000000L)
  {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0)
  {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < 0 || sec_part > INT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

/**
 * \brief Base class for Time implementations.  Provides storage, common functions and operator overloads.
 * This should not need to be used directly.
 */
template<class T, class D>
class TimeBase
{
public:
  uint32_t sec, nsec;

  TimeBase() : sec(0), nsec(0) { }
  TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
  {
    normalizeSecNSec(sec, nsec);
  }
  explicit TimeBase(double t) { fromSec(t); }
  ~TimeBase() {}
  D operator-(const T &rhs) const;
  T operator+(const D &rhs) const;
  T operator-(const D &rhs) const;
  T& operator+=(const D &rhs);
  T& operator-=(const D &rhs);
  bool operator==(const T &rhs) const;
  inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
  bool operator>(const T &rhs) const;
  bool operator<(const T &rhs) const;
  bool operator>=(const T &rhs) const;
  bool operator<=(const T &rhs) const;

  double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
  T& fromSec(double t) { sec = (uint32_t)floor(t); nsec = (uint32_t)round((t-sec) * 1e9);  return *static_cast<T*>(this);}

  uint64_t toNSec() const {return (uint64_t)sec*1000000000ull + (uint64_t)nsec;  }
  T& fromNSec(uint64_t t);

  inline bool isZero() const { return sec == 0 && nsec == 0; }
  inline bool is_zero() const { return isZero(); }
};

/**
 * \brief Time representation.  May either represent wall clock time or ROS clock time.
 *
 * ros::TimeBase provides most of its functionality.
 */
class Time : public TimeBase<Time, Duration>
{
public:
  Time()
  : TimeBase<Time, Duration>()
  {}

  Time(uint32_t _sec, uint32_t _nsec)
  : TimeBase<Time, Duration>(_sec, _nsec)
  {}

  explicit Time(double t) { fromSec(t); }

  /**
   * \brief Retrieve the current time.  If ROS clock time is in use, this returns the time according to the
   * ROS clock.  Otherwise returns the current wall clock time.
   */
  static Time now();
  /**
   * \brief Sleep until a specific time has been reached.
   */
  static bool sleepUntil(const Time& end);

  static void init();
  static void shutdown();
  static void setNow(const Time& new_now);
  static bool useSystemTime() { return use_system_time_; }
private:
  static Time sim_time_;
  static bool use_system_time_;
};

extern const Time TIME_MAX;
extern const Time TIME_MIN;

/**
 * \brief Time representation.  Always wall-clock time.
 *
 * ros::TimeBase provides most of its functionality.
 */
class WallTime : public TimeBase<WallTime, WallDuration>
{
public:
  WallTime()
  : TimeBase<WallTime, WallDuration>()
  {}

  WallTime(uint32_t _sec, uint32_t _nsec)
  : TimeBase<WallTime, WallDuration>(_sec, _nsec)
  {}

  explicit WallTime(double t) { fromSec(t); }

  /**
   * \brief Returns the current wall clock time.
   */
  static WallTime now();

  /**
   * \brief Sleep until a specific time has been reached.
   */
  static bool sleepUntil(const WallTime& end);
};

std::ostream &operator <<(std::ostream &os, const Time &rhs);
std::ostream &operator <<(std::ostream &os, const WallTime &rhs);

template<class T, class D>
T& TimeBase<T, D>::fromNSec(uint64_t t)
{
  sec  = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSec(sec, nsec);

  return *static_cast<T*>(this);
}

template<class T, class D>
D TimeBase<T, D>::operator-(const T &rhs) const
{
  return D((int32_t)sec -  (int32_t)rhs.sec,
                  (int32_t)nsec - (int32_t)rhs.nsec); // carry handled in ctor
}

template<class T, class D>
T TimeBase<T, D>::operator-(const D &rhs) const
{
  return *static_cast<const T*>(this) + ( -rhs);
}

template<class T, class D>
T TimeBase<T, D>::operator+(const D &rhs) const
{
  int64_t sec_sum  = (int64_t)sec  + (int64_t)rhs.sec;
  int64_t nsec_sum = (int64_t)nsec + (int64_t)rhs.nsec;

  // Throws an exception if we go out of 32-bit range
  normalizeSecNSecUnsigned(sec_sum, nsec_sum);

  // now, it's safe to downcast back to uint32 bits
  return T((uint32_t)sec_sum, (uint32_t)nsec_sum);
}

template<class T, class D>
T& TimeBase<T, D>::operator+=(const D &rhs)
{
  *this = *this + rhs;
  return *static_cast<T*>(this);
}

template<class T, class D>
T& TimeBase<T, D>::operator-=(const D &rhs)
{
  *this += (-rhs);
  return *static_cast<T*>(this);
}

template<class T, class D>
bool TimeBase<T, D>::operator==(const T &rhs) const
{
  return sec == rhs.sec && nsec == rhs.nsec;
}

template<class T, class D>
bool TimeBase<T, D>::operator<(const T &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator>(const T &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator<=(const T &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator>=(const T &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

}

#endif // ROS_TIME_H

