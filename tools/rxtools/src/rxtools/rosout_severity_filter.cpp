/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include "rosout_severity_filter.h"

namespace rxtools
{

RosoutSeverityFilter::RosoutSeverityFilter()
: severity_mask_(Default)
{

}

void RosoutSeverityFilter::setSeverityMask(uint32_t field_mask)
{
  severity_mask_ = field_mask;

  changed();
}

bool RosoutSeverityFilter::doFilter(const roslib::LogConstPtr& msg) const
{
  bool match = false;
  switch (msg->level)
  {
  case roslib::Log::DEBUG:
    match = severity_mask_ & Debug;
    break;
  case roslib::Log::INFO:
    match = severity_mask_ & Info;
    break;
  case roslib::Log::WARN:
    match = severity_mask_ & Warn;
    break;
  case roslib::Log::ERROR:
    match = severity_mask_ & Error;
    break;
  case roslib::Log::FATAL:
    match = severity_mask_ & Fatal;
    break;
  }

  return match;
}

bool RosoutSeverityFilter::doIsValid() const
{
  return true;
}

}
