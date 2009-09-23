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

#ifndef ROS_DURATION_H
#define ROS_DURATION_H

#include <iostream>
#include <math.h>
#include <stdexcept>
#include <climits>
#include <stdint.h>

#ifdef WIN32
#include <windows.h>
#endif

namespace ros
{

inline void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec)
{
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part > 1000000000L)
  {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0)
  {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < INT_MIN || sec_part > INT_MAX)
    throw std::runtime_error("Duration is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

inline void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec)
{
  int64_t sec64 = sec;
  int64_t nsec64 = nsec;

  normalizeSecNSecSigned(sec64, nsec64);

  sec = (int32_t)sec64;
  nsec = (int32_t)nsec64;
}

template<class T>
class DurationBase
{
public:
  int32_t sec, nsec;
  DurationBase() : sec(0), nsec(0) { }
  DurationBase(int32_t _sec, int32_t _nsec);
  explicit DurationBase(double t){fromSec(t);};
  ~DurationBase() {}
  T operator+(const T &rhs) const;
  T operator-(const T &rhs) const;
  T operator-() const;
  T operator*(double scale) const;
  T& operator+=(const T &rhs);
  T& operator-=(const T &rhs);
  T& operator*=(double scale);
  bool operator==(const T &rhs) const;
  inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
  bool operator>(const T &rhs) const;
  bool operator<(const T &rhs) const;
  bool operator>=(const T &rhs) const;
  bool operator<=(const T &rhs) const;
  double toSec() const { return (double)sec + 1e-9*(double)nsec; };
  int64_t toNSec() const {return (int64_t)sec*1000000000ll + (int64_t)nsec;  };
  T& fromSec(double t);
  T& fromNSec(int64_t t);
  bool isZero();
};

class Duration : public DurationBase<Duration>
{
public:
  Duration()
  : DurationBase<Duration>()
  { }

  Duration(int32_t _sec, int32_t _nsec)
  : DurationBase<Duration>(_sec, _nsec)
  {}

  explicit Duration(double t) { fromSec(t); }

  /**
   * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for the time remaining.
   */
  bool sleep() const;
};

extern const Duration DURATION_MAX;
extern const Duration DURATION_MIN;

class WallDuration : public DurationBase<WallDuration>
{
public:
  WallDuration()
  : DurationBase<WallDuration>()
  { }

  WallDuration(int32_t _sec, int32_t _nsec)
  : DurationBase<WallDuration>(_sec, _nsec)
  {}

  explicit WallDuration(double t) { fromSec(t); }

  /**
   * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for the time remaining.
   */
  bool sleep() const;
};

std::ostream &operator <<(std::ostream &os, const Duration &rhs);
std::ostream &operator <<(std::ostream &os, const WallDuration &rhs);

//
// DurationBase template member function implementation
//
template<class T>
DurationBase<T>::DurationBase(int32_t _sec, int32_t _nsec)
: sec(_sec), nsec(_nsec)
{
  normalizeSecNSecSigned(sec, nsec);
}

template<class T>
T& DurationBase<T>::fromSec(double d)
{
#ifdef HAVE_TRUNC
  sec  = (int32_t)trunc(d);
#else
  // (morgan: why doesn't win32 provide trunc? argh. hacked this together
  // without much thought. need to test this conversion.
  if (d >= 0.0)
    sec = (int32_t)floor(d);
  else
    sec = (int32_t)floor(d) + 1;
#endif
  nsec = (int32_t)((d - (double)sec)*1000000000);
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::fromNSec(int64_t t)
{
  sec  = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSecSigned(sec, nsec);

  return *static_cast<T*>(this);
}

template<class T>
T DurationBase<T>::operator+(const T &rhs) const
{
  return T(sec + rhs.sec, nsec + rhs.nsec);
}

template<class T>
T DurationBase<T>::operator*(double scale) const
{
  return T(toSec() * scale);
}

template<class T>
T DurationBase<T>::operator-(const T &rhs) const
{
  return T(sec - rhs.sec, nsec - rhs.nsec);
}

template<class T>
T DurationBase<T>::operator-() const
{
  return T(-sec , -nsec);
}

template<class T>
T& DurationBase<T>::operator+=(const T &rhs)
{
  *this = *this + rhs;
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::operator-=(const T &rhs)
{
  *this += (-rhs);
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::operator*=(double scale)
{
  fromSec(toSec() * scale);
  return *static_cast<T*>(this);
}

template<class T>
bool DurationBase<T>::operator<(const T &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator>(const T &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator<=(const T &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator>=(const T &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator==(const T &rhs) const
{
  return sec == rhs.sec && nsec == rhs.nsec;
}

template<class T>
bool DurationBase<T>::isZero()
{
  return sec == 0 && nsec == 0;
}

}

#endif // ROS_DURATION_H


