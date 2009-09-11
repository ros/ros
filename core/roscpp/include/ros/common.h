/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCPP_COMMON_H
#define ROSCPP_COMMON_H

#include <stdint.h>
#include <assert.h>
#include <stddef.h>
#include <string>

#include "ros/assert.h"
#include "ros/forwards.h"

#include <boost/shared_array.hpp>

#define ROS_VERSION_MAJOR 0
#define ROS_VERSION_MINOR 7
#define ROS_VERSION_PATCH 2
#define ROS_VERSION_COMBINED(major, minor, patch) (((major) << 20) | ((minor) << 10) | (patch))
#define ROS_VERSION ROS_VERSION_COMBINED(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH)

#define ROS_VERSION_GE(major1, minor1, patch1, major2, minor2, patch2) (ROS_VERSION_COMBINED(major1, minor1, patch1) >= ROS_VERSION_COMBINED(major2, minor2, patch2))
#define ROS_VERSION_MINIMUM(major, minor, patch) ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, major, minor, patch)

namespace ros
{
class Node;
extern Node *g_node;

/**
 * \deprecated Use ros::package::getPath() in the roslib package instead
 */
ROSCPP_DEPRECATED std::string getPackagePath(const std::string &package_name);

void disableAllSignalsInThisThread();

class SerializedMessage
{
public:
  boost::shared_array<uint8_t> buf;
  size_t num_bytes;

  SerializedMessage()
  : buf(boost::shared_array<uint8_t>())
  , num_bytes(0)
  {}

  SerializedMessage(boost::shared_array<uint8_t> buf, size_t num_bytes)
  : buf(buf), num_bytes(num_bytes) { }
};

class AbstractFunctor
{
public:
  virtual void call() = 0;
  virtual ~AbstractFunctor() { }
  virtual bool operator==(const AbstractFunctor &rhs) { return equals(&rhs); }
private:
  virtual bool equals(const AbstractFunctor *) = 0;
};

template<class T>
class MethodFunctor : public AbstractFunctor
{
public:
  MethodFunctor(T *_obj, void (T::*_mp)())
  : mp_(_mp)
  , mp_data_(NULL)
  , obj_(_obj)
  , user_data_(NULL)
  { }

  MethodFunctor(T *_obj, void (T::*_mp_data)(void *), void *_user_data_)
  : mp_(NULL)
  , mp_data_(_mp_data)
  , obj_(_obj)
  , user_data_(_user_data_)
  { }

  virtual void call()
  {
    if (mp_)
    {
      (*obj_.*mp_)();
    }
    else if (mp_data_)
    {
      (*obj_.*mp_data_)(user_data_);
    }
    else
    {
      ROS_BREAK();
      (void)0;
    }
  }
private:
  void (T::*mp_)();
  void (T::*mp_data_)(void *);
  T* obj_;

  void* user_data_;

  virtual bool equals(const AbstractFunctor *_rhs)
  {
    const MethodFunctor<T> *rhs = dynamic_cast<const MethodFunctor<T> *>(_rhs);
    if (!rhs)
      return false;
    return rhs->mp_ == mp_ && rhs->mp_data_ == mp_data_ &&
           rhs->obj_ == obj_ && rhs->user_data_ == user_data_;
  }
};

class FunctionFunctor : public AbstractFunctor
{
public:
  FunctionFunctor(void (*_fp)())
  : fp_(_fp)
  , fp_data_(NULL)
  , user_data_(NULL)
  { }

  FunctionFunctor(void (*_fp_data)(void *), void *_user_data_)
  : fp_(NULL)
  , fp_data_(_fp_data)
  , user_data_(_user_data_)
  { }

  virtual void call()
  {
    if (fp_)
    {
      (*fp_)();
    }
    else if (fp_data_)
    {
      (*fp_data_)(user_data_);
    }
    else
    {
      ROS_BREAK();
      (void)0;
    }
  }
private:
  void (*fp_)();
  void (*fp_data_)(void *);

  void* user_data_;

  virtual bool equals(const AbstractFunctor *_rhs)
  {
    const FunctionFunctor* rhs = dynamic_cast<const FunctionFunctor*>(_rhs);
    if (!rhs)
    {
      return false;
    }

    return rhs->fp_ == fp_ && rhs->fp_data_ == fp_data_ && rhs->user_data_ == user_data_;
  }
};

}

#endif

