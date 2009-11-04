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

#include "rosout_text_filter.h"

#include <boost/algorithm/string/case_conv.hpp>

namespace rxtools
{

int wildcmp(const char* wild, const char* string)
{
  // Written by Jack Handy - jakkhandy@hotmail.com
  // found at http://www.codeproject.com/KB/string/wildcmp.aspx

  const char *cp = NULL, *mp = NULL;

  while ((*string) && (*wild != '*'))
  {
    if ((*wild != *string) && (*wild != '?'))
    {
      return 0;
    }
    wild++;
    string++;
  }

  while (*string)
  {
    if (*wild == '*')
    {
      if (!*++wild)
      {
        return 1;
      }
      mp = wild;
      cp = string+1;
    }
    else if ((*wild == *string) || (*wild == '?'))
    {
      wild++;
      string++;
    }
    else
    {
      wild = mp;
      string = cp++;
    }
  }

  while (*wild == '*')
  {
    wild++;
  }
  return !*wild;
}

RosoutTextFilter::RosoutTextFilter()
: field_mask_(Default)
, use_regex_(false)
, type_(Include)
, regex_valid_(true)
{

}

void RosoutTextFilter::setFieldMask(uint32_t field_mask)
{
  field_mask_ = field_mask;

  changed();
}

void RosoutTextFilter::setText(const std::string& text)
{
  text_ = text;

  if (use_regex_)
  {
    regex_valid_ = true;
    if (!text_.empty())
    {
      try
      {
        regex_ = boost::regex(text_, boost::regbase::normal | boost::regbase::icase);
      }
      catch (std::runtime_error&)
      {
        regex_valid_ = false;
      }
    }
  }

  changed();
}

void RosoutTextFilter::setUseRegex(bool use)
{
  use_regex_ = use;

  // setText will call changed(), so we don't do it here explicitly
  setText(text_);
}

void RosoutTextFilter::setFilterType(FilterType type)
{
  type_ = type;

  changed();
}

bool RosoutTextFilter::filterString(const std::string& str) const
{
  bool match = false;
  if (use_regex_)
  {
    if (regex_valid_)
    {
      match = boost::regex_match(str, regex_);
    }
  }
  else
  {
    std::string lower_str = boost::algorithm::to_lower_copy(str);
    std::string lower_text = "*" + boost::algorithm::to_lower_copy(text_) + "*";
    match = wildcmp(lower_text.c_str(), lower_str.c_str());
  }

  return match;
}

bool RosoutTextFilter::filterVector(const std::vector<std::string>& strs) const
{
  std::vector<std::string>::const_iterator it = strs.begin();
  std::vector<std::string>::const_iterator end = strs.end();
  for (; it != end; ++it)
  {
    if (filterString(*it))
    {
      return true;
    }
  }

  return false;
}

bool RosoutTextFilter::doFilter(const roslib::LogConstPtr& msg) const
{
  if (text_.empty())
  {
    return true;
  }

  bool match = false;

  if (field_mask_ & Message)
  {
    match = match || filterString(msg->msg);
  }

  if (field_mask_ & Node)
  {
    match = match || filterString(msg->name);
  }

  if (field_mask_ & File)
  {
    match = match || filterString(msg->file);
  }

  if (field_mask_ & Function)
  {
    match = match || filterString(msg->function);
  }

  if (field_mask_ & Topics)
  {
    match = match || filterVector(msg->topics);
  }

  return type_ == Include ? match : !match;
}

bool RosoutTextFilter::doIsValid() const
{
  return !use_regex_ || regex_valid_;
}

}
