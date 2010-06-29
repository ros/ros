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
 *********************************************************************/

#ifndef RXTOOLS_TOPIC_DISPLAY_H
#define RXTOOLS_TOPIC_DISPLAY_H

#include "topic_display_generated.h"
#include <wx/timer.h>

#include <ros/ros.h>

namespace rxtools
{

typedef std::vector<std::string> V_string;

/**
 * \brief The TopicDisplay class is an embeddable panel encapsulating a treeview that displays list of
 * topics advertised on the master.  It is also possible to filter this list to only show topics of a
 * specific message type.
 */
class TopicDisplay : public GenTopicDisplay
{
public:
  /**
   * \brief Constructor
   * \param parent The parent window for this panel
   * \param node The ROS node
   * \param message_type The message type to filter by, or empty string to show all types.  If this is set, the panel will only display topics of this type
   * \param size The size of the panel
   */
  TopicDisplay(wxWindow* parent, const std::string& message_type = "", bool auto_refresh = true, const wxSize& size = wxSize(500, 300));
  virtual ~TopicDisplay();

  /**
   * \brief Get the topics that have been selected in the list.  If multiple selection is disallowed, this will return a 0 or 1 length vector
   */
  void getSelectedTopics(V_string& topics);
  /**
   * \brief Set whether multiple selection of topics is allowed by the treeview
   */
  void setMultiselectAllowed(bool allowed);

private:
  /**
   * \brief Structure used to store information about a topic we've received
   */
  struct TopicInfo
  {
    bool save_; ///< Used when refreshing the topic list to determine if we should delete this topic from the tree or not
    wxTreeItemId item_; ///< Tree item associated with this topic
    std::string type_; ///< Topic data type (eg. "std_msgs/Empty")
  };

  typedef std::map<std::string, TopicInfo> M_TopicInfo;

  wxTimer* timer_; ///< Timer controlling our refresh rate
  ros::NodeHandle nh_;
  M_TopicInfo topic_cache_; ///< Cache of topic info, used when refreshing

  wxTreeItemId root_id_; ///< Tree item id for the root of the tree

  std::string message_type_; ///< Message type to filter by (blank == show all)

  virtual void checkIsTopic(wxTreeEvent& event);
  virtual void onItemActivated(wxTreeEvent& event);
  virtual void tick(wxTimerEvent& event);
  virtual void refreshTopics();
};

} // namespace rxtools

#endif // RXTOOLS_TOPIC_DISPLAY_H
