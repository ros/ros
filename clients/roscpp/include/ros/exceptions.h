/*
 * Copyright (C) 2009, Willow Garage, Inc.
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

#ifndef ROSCPP_EXCEPTIONS_H
#define ROSCPP_EXCEPTIONS_H

#include <ros/exception.h>

namespace ros
{

/**
 * \brief Thrown when an invalid node name is specified to ros::init()
 */
class InvalidNodeNameException : public ros::Exception
{
public:
  InvalidNodeNameException(const std::string& name, const std::string& reason)
  : Exception("Invalid node name [" + name + "]: " + reason)
  {}
};

/**
 * \brief Thrown when an invalid graph resource name is specified to any roscpp
 * function.
 */
class InvalidNameException : public ros::Exception
{
public:
  InvalidNameException(const std::string& msg)
  : Exception(msg)
  {}
};

/**
 * \brief Thrown when a second (third,...) subscription is attempted with conflicting
 * arguments.
 */
class ConflictingSubscriptionException : public ros::Exception
{
public:
  ConflictingSubscriptionException(const std::string& msg)
  : Exception(msg)
  {}
};

/**
 * \brief Thrown when an invalid parameter is passed to a method
 */
class InvalidParameterException : public ros::Exception
{
public:
  InvalidParameterException(const std::string& msg)
  : Exception(msg)
  {}
};

/**
 * \brief Thrown when an invalid port is specified
 */
class InvalidPortException : public ros::Exception
{
public:
  InvalidPortException(const std::string& msg)
  : Exception(msg)
  {}
};

} // namespace ros

#endif

