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

#include "rosout_text_filter_control.h"
#include "rosout_text_filter.h"

#include <boost/bind.hpp>

namespace rxtools
{

RosoutTextFilterControl::RosoutTextFilterControl(wxWindow* parent, const RosoutTextFilterPtr& filter)
: RosoutTextFilterControlBase(parent, wxID_ANY)
, filter_(filter)
{
  filter_changed_connection_ = filter_->getChangedSignal().connect(boost::bind(&RosoutTextFilterControl::read, this));

  read();
}

void RosoutTextFilterControl::read()
{
  regex_->SetValue(filter_->getUseRegex());

  uint32_t field_mask = filter_->getFieldMask();
  message_->SetValue(field_mask & RosoutTextFilter::Message);
  node_->SetValue(field_mask & RosoutTextFilter::Node);
  location_->SetValue(field_mask & RosoutTextFilter::Location);
  topics_->SetValue(field_mask & RosoutTextFilter::Topics);

  include_exclude_->SetSelection(filter_->getFilterType());

  text_->ChangeValue(wxString::FromAscii(filter_->getText().c_str()));

  setIncludeExcludeColor();
}

void RosoutTextFilterControl::setIncludeExcludeColor()
{
  if (include_exclude_->GetSelection() == 0)
  {
    include_exclude_->SetBackgroundColour(wxColour(255, 238, 176));
  }
  else
  {
    include_exclude_->SetBackgroundColour(wxColour(198, 203, 255));
  }
}

void RosoutTextFilterControl::checkValid()
{
  text_->SetBackgroundColour(wxNullColour);
  if (!filter_->isValid())
  {
    text_->SetBackgroundColour(wxColour(255, 99, 78));
  }
}

void RosoutTextFilterControl::onIncludeExclude( wxCommandEvent& event )
{
  filter_changed_connection_.block();
  filter_->setFilterType((RosoutTextFilter::FilterType)include_exclude_->GetSelection());
  filter_changed_connection_.unblock();

  checkValid();
  setIncludeExcludeColor();
}

void RosoutTextFilterControl::onRegex( wxCommandEvent& event )
{
  filter_changed_connection_.block();
  filter_->setUseRegex(event.IsChecked());
  filter_changed_connection_.unblock();

  checkValid();
}

void RosoutTextFilterControl::onText( wxCommandEvent& event )
{
  filter_changed_connection_.block();
  filter_->setText((const char*)text_->GetValue().char_str());
  filter_changed_connection_.unblock();

  checkValid();
}

void RosoutTextFilterControl::onMessage( wxCommandEvent& event )
{
  filter_changed_connection_.block();
  if (event.IsChecked())
  {
    filter_->addField(RosoutTextFilter::Message);
  }
  else
  {
    filter_->removeField(RosoutTextFilter::Message);
  }
  filter_changed_connection_.unblock();

  checkValid();
}

void RosoutTextFilterControl::onNode( wxCommandEvent& event )
{
  filter_changed_connection_.block();
  if (event.IsChecked())
  {
    filter_->addField(RosoutTextFilter::Node);
  }
  else
  {
    filter_->removeField(RosoutTextFilter::Node);
  }
  filter_changed_connection_.unblock();

  checkValid();
}

void RosoutTextFilterControl::onLocation( wxCommandEvent& event )
{
  filter_changed_connection_.block();
  if (event.IsChecked())
  {
    filter_->addField(RosoutTextFilter::Location);
  }
  else
  {
    filter_->removeField(RosoutTextFilter::Location);
  }
  filter_changed_connection_.unblock();

  checkValid();
}

void RosoutTextFilterControl::onTopics( wxCommandEvent& event )
{
  filter_changed_connection_.block();
  if (event.IsChecked())
  {
    filter_->addField(RosoutTextFilter::Topics);
  }
  else
  {
    filter_->removeField(RosoutTextFilter::Topics);
  }
  filter_changed_connection_.unblock();

  checkValid();
}

}
