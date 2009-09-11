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

#include "topic_display_dialog.h"
#include "topic_display.h"

// wx includes
#include <wx/msgdlg.h>

namespace rxtools
{

TopicDisplayDialog::TopicDisplayDialog(wxWindow* parent, ros::Node*, bool multiselect, const std::string& message_type)
: GenTopicDisplayDialog(parent)
{
  topic_display_panel_ = new TopicDisplay(tree_panel_, message_type, false, tree_panel_->GetSize());
  topic_display_panel_->setMultiselectAllowed(multiselect);

  topic_display_panel_->Connect(wxEVT_COMMAND_TREE_ITEM_ACTIVATED, wxTreeEventHandler( TopicDisplayDialog::onTreeItemActivated ), NULL, this );
}

TopicDisplayDialog::TopicDisplayDialog(wxWindow* parent, bool multiselect, const std::string& message_type)
: GenTopicDisplayDialog(parent)
{
  topic_display_panel_ = new TopicDisplay(tree_panel_, message_type, false, tree_panel_->GetSize());
  topic_display_panel_->setMultiselectAllowed(multiselect);

  topic_display_panel_->Connect(wxEVT_COMMAND_TREE_ITEM_ACTIVATED, wxTreeEventHandler( TopicDisplayDialog::onTreeItemActivated ), NULL, this );
}

TopicDisplayDialog::~TopicDisplayDialog()
{
}

void TopicDisplayDialog::onOK(wxCommandEvent& event)
{
  V_string selection;
  topic_display_panel_->getSelectedTopics(selection);
  if (!selection.empty())
  {
    EndModal(wxID_OK);
  }
  else
  {
    wxMessageBox(wxT("Please select a topic!"), wxT("No topic selected"), wxOK | wxCENTRE | wxICON_ERROR, this);
  }
}

void TopicDisplayDialog::onCancel(wxCommandEvent& event)
{
  EndModal(wxID_CANCEL);
}

void TopicDisplayDialog::getSelection(V_string& selection)
{
  topic_display_panel_->getSelectedTopics(selection);
}

void TopicDisplayDialog::onTreeItemActivated(wxTreeEvent& event)
{
  V_string selection;
  topic_display_panel_->getSelectedTopics(selection);
  if (!selection.empty())
  {
    EndModal(wxID_OK);
  }
}

} // namespace rxtools
