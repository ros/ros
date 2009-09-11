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

#include "ros/time.h"
#include <cmath>
#include <time.h>
#include <iomanip>
#include <stdexcept>

#include <config.h>

#include <boost/thread/mutex.hpp>

#ifndef WIN32
  #if POSIX_TIMERS <= 0
  #include <sys/time.h>
  #endif
#else
  #include <sys/timeb.h>
  ros::Time ros::Time::start_time;
#endif

using namespace ros;
using namespace std;


ros::Time ros::Time::sim_time_(0, 0);
bool ros::Time::use_system_time_(true);

boost::mutex g_sim_time_mutex;

Duration::Duration(int32_t _sec, int32_t _nsec)
: sec(_sec), nsec(_nsec)
{
  while (nsec > 1000000000)
  {
    nsec -= 1000000000;
    sec++;
  }
  while (nsec < 0)
  {
    nsec += 1000000000;
    sec--;
  }
}


Duration& Duration::fromSec(double d)
{
#ifdef HAVE_TRUNC
  sec  = (int32_t)trunc(d);
#else
  // (morgan: why doesn't win32 provide trunc? argh. hacked this together
  // without much thought. need to test this conversion.
  if (d > 0)
    sec = (int32_t)floor(d);
  else
    sec = (int32_t)floor(d) + 1;
#endif
  nsec = (int32_t)((d - (double)sec)*1000000000);
  return *this;
}


Duration& Duration::fromNSec(int64_t t)
{
  sec  = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);
  while (nsec > 1000000000)
  {
    nsec -= 1000000000;
    sec++;
  }
  while (nsec < 0)
  {
    nsec += 1000000000;
    sec--;
  }
  return *this;
}


Duration Time::operator-(const Time &rhs) const
{
  return Duration((int32_t)sec -  (int32_t)rhs.sec,
                  (int32_t)nsec - (int32_t)rhs.nsec); // carry handled in ctor
}

Time Time::operator-(const Duration &rhs) const
{
  return *this + ( -rhs);
}

Time Time::operator+(const Duration &rhs) const
{
  long long sec_sum  = (long long)sec  + (long long)rhs.sec;
  long long nsec_sum = (long long)nsec + (long long)rhs.nsec;
  while (nsec_sum < 0)
  {
    nsec_sum += 1000000000;
    sec_sum--;
  }
  while (nsec_sum > 1000000000)
  {
    nsec_sum -= 1000000000;
    sec_sum++;
  }
  if (sec_sum < 0 || sec_sum > 4294967295U)
    throw std::runtime_error("time + duration was out of dual 32-bit range");
  // now, it's safe to downcast back to uint32 bits
  return Time((uint32_t)sec_sum, (uint32_t)nsec_sum);
}

Time& Time::operator+=(const Duration &rhs)
{
  sec += rhs.sec;
  nsec += rhs.nsec;
  return *this;
};

Time& Time::operator-=(const Duration &rhs)
{ *this += (-rhs); 
  return *this;};

Duration Duration::operator+(const Duration &rhs) const
{
  return Duration(sec + rhs.sec, nsec + rhs.nsec);
}

Duration Duration::operator*(double scale) const
{
  return Duration(toSec() * scale);
}

Duration Duration::operator-(const Duration &rhs) const
{
  return Duration(sec - rhs.sec, nsec - rhs.nsec);
}
Duration Duration::operator-() const
{
  return Duration(-sec , -nsec);
}

Duration& Duration::operator+=(const Duration &rhs)
{
  sec += rhs.sec;
  nsec += rhs.nsec;
  return *this;
};

Duration& Duration::operator-=(const Duration &rhs)
{ *this += (-rhs); 
  return *this;};

Duration& Duration::operator*=(double scale)
{
  fromSec(toSec() * scale);
  return *this;
};


bool Duration::operator<(const Duration &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

bool Duration::operator>(const Duration &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

bool Duration::operator<=(const Duration &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

bool Duration::operator>=(const Duration &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

Time Time::now()
{
  if (!use_system_time_)
  {
    boost::mutex::scoped_lock lock(g_sim_time_mutex);
    Time t = sim_time_;
    return t;
  }


  Time t;
#ifndef WIN32
#if POSIX_TIMERS > 0
  struct timespec start;
  clock_gettime(CLOCK_REALTIME, &start);
  t.sec  = start.tv_sec;
  t.nsec = start.tv_nsec;
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  t.sec  = timeofday.tv_sec;
  t.nsec = timeofday.tv_usec * 1000;
#endif
  return t;
#else
  if (start_time.is_zero())
  {
    QueryPerformanceFrequency(&cpu_freq);
    if (cpu_freq.QuadPart == 0)
    {
      ROS_INFO("woah! this system (for whatever reason) does not support the "
             "high-performance timing API. ur done.\n");
      abort();
    }
    QueryPerformanceCounter(&init_cpu_time);
    // compute an offset from the Epoch using the lower-performance timer API
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    LARGE_INTEGER start_li;
    start_li.LowPart = ft.dwLowDateTime;
    start_li.HighPart = ft.dwHighDateTime;
    // why did they choose 1601 as the time zero, instead of 1970?
    // there were no outstanding hard rock bands in 1601.
#ifdef _MSC_VER
    start_li.QuadPart -= 116444736000000000Ui64;
#else
    start_li.QuadPart -= 116444736000000000ULL;
#endif
    start_time.sec = (uint32_t)(start_li.QuadPart / 10000000); // 100-ns units. odd.
    start_time.nsec = (start_li.LowPart % 10000000) * 100;
  }
  LARGE_INTEGER cur_time;
  QueryPerformanceCounter(&cur_time);
  LARGE_INTEGER delta_cpu_time;
  delta_cpu_time.QuadPart = cur_time.QuadPart - init_cpu_time.QuadPart;
  // todo: how to handle cpu clock drift. not sure it's a big deal for us.
  // also, think about clock wraparound. seems extremely unlikey, but possible
  double d_delta_cpu_time = delta_cpu_time.QuadPart / (double)cpu_freq.QuadPart;
  return Time(start_time + Duration(d_delta_cpu_time));
#endif
}

bool Duration::operator==(const Duration &rhs) const
{
  return sec == rhs.sec && nsec == rhs.nsec;
}

bool Time::operator==(const Time &rhs) const
{
  return sec == rhs.sec && nsec == rhs.nsec;
}

bool Time::operator<(const Time &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

bool Time::operator>(const Time &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}


bool Time::operator<=(const Time &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

bool Time::operator>=(const Time &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

void Time::setNow(const Time& new_now)
{
  boost::mutex::scoped_lock lock(g_sim_time_mutex);

  sim_time_ = new_now;
  use_system_time_ = false;
}




ostream &ros::operator<<(ostream &os, const Time &rhs)
{
  os << rhs.sec << "." << setw(9) << setfill('0') << rhs.nsec;
  return os;
}

ostream &ros::operator<<(ostream &os, const Duration &rhs)
{
  os << rhs.sec << "." << setw(9) << setfill('0') << rhs.nsec;
  return os;
}

bool Duration::sleep()
{
  struct timespec ts = {sec, nsec};
  return nanosleep(&ts, NULL) ? false : true;
}

