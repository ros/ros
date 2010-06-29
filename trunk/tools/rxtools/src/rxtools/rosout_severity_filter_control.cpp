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

#include "rosout_severity_filter_control.h"
#include "rosout_severity_filter.h"

namespace rxtools
{

RosoutSeverityFilterControl::RosoutSeverityFilterControl(wxWindow* parent, const RosoutSeverityFilterPtr& filter)
: RosoutSeverityFilterControlBase(parent, wxID_ANY)
, filter_(filter)
{
  uint32_t mask = filter_->getSeverityMask();
  fatal_->SetValue(mask & RosoutSeverityFilter::Fatal);
  error_->SetValue(mask & RosoutSeverityFilter::Error);
  warn_->SetValue(mask & RosoutSeverityFilter::Warn);
  info_->SetValue(mask & RosoutSeverityFilter::Info);
  debug_->SetValue(mask & RosoutSeverityFilter::Debug);
}

void RosoutSeverityFilterControl::onFatal( wxCommandEvent& event )
{
  if (event.IsChecked())
  {
    filter_->addSeverity(RosoutSeverityFilter::Fatal);
  }
  else
  {
    filter_->removeSeverity(RosoutSeverityFilter::Fatal);
  }
}

void RosoutSeverityFilterControl::onError( wxCommandEvent& event )
{
  if (event.IsChecked())
  {
    filter_->addSeverity(RosoutSeverityFilter::Error);
  }
  else
  {
    filter_->removeSeverity(RosoutSeverityFilter::Error);
  }
}

void RosoutSeverityFilterControl::onWarn( wxCommandEvent& event )
{
  if (event.IsChecked())
  {
    filter_->addSeverity(RosoutSeverityFilter::Warn);
  }
  else
  {
    filter_->removeSeverity(RosoutSeverityFilter::Warn);
  }
}

void RosoutSeverityFilterControl::onInfo( wxCommandEvent& event )
{
  if (event.IsChecked())
  {
    filter_->addSeverity(RosoutSeverityFilter::Info);
  }
  else
  {
    filter_->removeSeverity(RosoutSeverityFilter::Info);
  }
}

void RosoutSeverityFilterControl::onDebug( wxCommandEvent& event )
{
  if (event.IsChecked())
  {
    filter_->addSeverity(RosoutSeverityFilter::Debug);
  }
  else
  {
    filter_->removeSeverity(RosoutSeverityFilter::Debug);
  }
}

}
