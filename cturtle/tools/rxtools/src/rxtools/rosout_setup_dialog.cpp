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
********************************************************************/

#include <stdint.h>
#include "rosout_setup_dialog.h"

#include "topic_display_dialog.h"

#include "roslib/Log.h"

namespace rxtools
{

RosoutSetupDialog::RosoutSetupDialog(wxWindow* parent, const std::string& topic, uint32_t buffer_size) :
  RosoutSetupDialogBase(parent)
{
  topic_->SetValue(wxString::FromAscii(topic.c_str()));
  buffer_size_spinner_->SetValue(buffer_size);
}

void RosoutSetupDialog::onCancel(wxCommandEvent& event)
{
  EndModal(wxCANCEL);
}

void RosoutSetupDialog::onOk(wxCommandEvent& event)
{
  EndModal(wxOK);
}

std::string RosoutSetupDialog::getTopic()
{
  return (const char*) topic_->GetValue().mb_str();
}

uint32_t RosoutSetupDialog::getBufferSize()
{
  return buffer_size_spinner_->GetValue();
}

void RosoutSetupDialog::onTopicBrowse(wxCommandEvent& event)
{
  TopicDisplayDialog dialog(this, false, roslib::Log::__s_getDataType());
  if (dialog.ShowModal() == wxID_OK)
  {
    std::vector<std::string> selection;
    dialog.getSelection(selection);
    if (!selection.empty())
    {
      topic_->SetValue(wxString::FromAscii(selection[0].c_str()));
    }
  }
}

} // namespace rxtools
