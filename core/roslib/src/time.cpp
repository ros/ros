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
#include <ctime>
#include <iomanip>
#include <stdexcept>
#include <limits>

#include <config.h>

#include <boost/thread/mutex.hpp>

#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

#ifndef WIN32
  #if !HAS_CLOCK_GETTIME
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

const Duration ros::DURATION_MAX(std::numeric_limits<int32_t>::max(), 999999999);
const Duration ros::DURATION_MIN(std::numeric_limits<int32_t>::min(), 0);

const Time ros::TIME_MAX(std::numeric_limits<uint32_t>::max(), 999999999);
const Time ros::TIME_MIN(0, 0);

// This is declared here because it's set from the Time class but read from
// the Duration class, and need not be exported to users of either.
static bool g_stopped(false);

// I assume that this is declared here, instead of time.h, to keep users
// of time.h from including boost/thread/mutex.hpp
static boost::mutex g_sim_time_mutex;

void getWallTime(uint32_t& sec, uint32_t& nsec)
{
#ifndef WIN32
#if HAS_CLOCK_GETTIME
  struct timespec start;
  clock_gettime(CLOCK_REALTIME, &start);
  sec  = start.tv_sec;
  nsec = start.tv_nsec;
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  sec  = timeofday.tv_sec;
  nsec = timeofday.tv_usec * 1000;
#endif
#else
  // unless I've missed something obvious, the only way to get high-precision
  // time on Windows is via the QueryPerformanceCounter() call. However,
  // this is somewhat problematic in Windows XP on some processors, especially
  // AMD, because the Windows implementation can freak out when the CPU clocks
  // down to save power. Time can jump or even go backwards. Microsoft has
  // fixed this bug for most systems now, but it can still show up if you have
  // not installed the latest CPU drivers (an oxymoron). They fixed all these
  // problems in Windows Vista, and this API is by far the most accurate that
  // I know of in Windows, so I'll use it here despite all these caveats
  static LARGE_INTEGER cpu_freq, init_cpu_time;
  static Time start_time;
  if (start_time.isZero())
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
  Time t(start_time + Duration(d_delta_cpu_time));

  sec = t.sec;
  nsec = t.nsec;
#endif
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
  getWallTime(t.sec, t.nsec);

  return t;
}

void Time::setNow(const Time& new_now)
{
  boost::mutex::scoped_lock lock(g_sim_time_mutex);

  sim_time_ = new_now;
  use_system_time_ = false;
}

void Time::init()
{
  g_stopped = false;
  use_system_time_ = true;
}

void Time::shutdown()
{
  g_stopped = true;
}

ostream &ros::operator<<(ostream& os, const Time &rhs)
{
  os << rhs.sec << "." << setw(9) << setfill('0') << rhs.nsec;
  return os;
}

ostream &ros::operator<<(ostream& os, const Duration& rhs)
{
  os << rhs.sec << "." << setw(9) << setfill('0') << rhs.nsec;
  return os;
}

bool Time::sleepUntil(const Time& end)
{
  if (Time::useSystemTime())
  {
    Duration d(end - Time::now());
    if (d > Duration(0))
    {
      return d.sleep();
    }

    return true;
  }
  else
  {
    Time start = Time::now();
    struct timespec ts = {0, 1000000};
    while (!g_stopped && (Time::now() < end))
    {
      if (nanosleep(&ts, NULL))
      {
       return false;
      }

      if (Time::now() < start)
      {
        return false;
      }
    }

    return true;
  }
}

bool WallTime::sleepUntil(const WallTime& end)
{
  WallDuration d(end - WallTime::now());
  if (d > WallDuration(0))
  {
    return d.sleep();
  }

  return true;
}

bool wallSleep(uint32_t sec, uint32_t nsec)
{
  struct timespec ts = {sec, nsec};
  struct timespec rem;

  while (nanosleep(&ts, &rem) && !g_stopped)
  {
    ts = rem;
  }

  return !g_stopped;
}

bool Duration::sleep() const
{
  if (Time::useSystemTime())
  {
    return wallSleep(sec, nsec);
  }
  else
  {
    Time start = Time::now();
    Time end = start + *this;
    while (!g_stopped && (Time::now() < end))
    {
      wallSleep(0, 1000000);

      if (Time::now() < start)
      {
        return false;
      }
    }

    return true;
  }
}

ostream &ros::operator<<(ostream& os, const WallTime &rhs)
{
  os << rhs.sec << "." << setw(9) << setfill('0') << rhs.nsec;
  return os;
}

WallTime WallTime::now()
{
  WallTime t;
  getWallTime(t.sec, t.nsec);

  return t;
}

ostream &ros::operator<<(ostream& os, const WallDuration& rhs)
{
  os << rhs.sec << "." << setw(9) << setfill('0') << rhs.nsec;
  return os;
}

bool WallDuration::sleep() const
{
  return wallSleep(sec, nsec);
}

