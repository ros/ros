/*
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
 */

#ifndef RXTOOLS_TOPIC_DISPLAY_DIALOG_H
#define RXTOOLS_TOPIC_DISPLAY_DIALOG_H

#include "topic_display_generated.h"

#include <vector>

namespace ros
{
class Node;
}

namespace rxtools
{

/**
 * \brief Dialog which provides the tree of topics and OK and Cancel buttons
 */
class TopicDisplay;

class TopicDisplayDialog : public GenTopicDisplayDialog
{
public:
  /**
   * \deprecated
   */
  __attribute__((deprecated)) TopicDisplayDialog(wxWindow* parent, ros::Node*, bool multiselect, const std::string& message_type = "");

  /**
   * \brief Constructor
   * \param parent The parent window, or NULL
   * \param node The ROS node
   * \param multiselect Whether multiple selection is valid or not.  If not, getSelection() will only ever return vectors of length 0 and 1
   * \param message_type The message type of the topics you'd like to display, or empty for all topics
   */
  TopicDisplayDialog(wxWindow* parent, bool multiselect, const std::string& message_type = "");
  ~TopicDisplayDialog();

  /**
   * \brief Returns the selected topics, or an empty vector if "Cancel" was pressed
   */
  void getSelection(std::vector<std::string>& selection);

private:
  virtual void onOK(wxCommandEvent& event);
  virtual void onCancel(wxCommandEvent& event);
  void onTreeItemActivated(wxTreeEvent& event);

  TopicDisplay* topic_display_panel_;
};

} // namespace rxtools

#endif // RXTOOLS_TOPIC_DISPLAY_DIALOG_H
