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

#ifndef RXTOOLS_ROSOUT_TEXT_FILTER_H
#define RXTOOLS_ROSOUT_TEXT_FILTER_H

#include <stdint.h>

#include "rosout_filter.h"
#include <boost/shared_ptr.hpp>
#include <boost/regex.hpp>

namespace rxtools
{

class RosoutTextFilter;
typedef boost::shared_ptr<RosoutTextFilter> RosoutTextFilterPtr;

class RosoutTextFilter : public RosoutFilter
{
public:
  RosoutTextFilter();

  enum Field
  {
    Message = 1<<0,
    Node = 1<<1,
    File = 1<<2,
    Function = 1<<3,
    Topics = 1<<4,
    Default = Message | Node | File | Function | Topics,
  };
  void setFieldMask(uint32_t field_mask);
  uint32_t getFieldMask() { return field_mask_; }
  void addField(Field field)
  {
    setFieldMask(field_mask_ | field);
  }

  void removeField(Field field)
  {
    setFieldMask(field_mask_ & ~field);
  }

  void setText(const std::string& text);
  const std::string& getText() { return text_; }

  void setUseRegex(bool use);
  bool getUseRegex() { return use_regex_; }

  enum FilterType
  {
    Include,
    Exclude
  };
  void setFilterType(FilterType type);
  FilterType getFilterType() { return type_; }

protected:
  virtual bool doFilter(const roslib::LogConstPtr&) const;
  virtual bool doIsValid() const;

private:
  bool filterString(const std::string& str) const;
  bool filterVector(const std::vector<std::string>& strs) const;

  uint32_t field_mask_;
  std::string text_;
  bool use_regex_;
  FilterType type_;

  boost::regex regex_;
  bool regex_valid_;
};

}

#endif // RXTOOLS_ROSOUT_TEXT_FILTER_H

