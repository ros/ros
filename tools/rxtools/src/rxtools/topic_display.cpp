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

#include "topic_display.h"

namespace rxtools
{

class TopicNameData : public wxTreeItemData
{
public:
  std::string name;
};

TopicDisplay::TopicDisplay(wxWindow* parent, const std::string& message_type, bool auto_refresh, const wxSize& size)
: GenTopicDisplay(parent, wxID_ANY, wxDefaultPosition, size)
, message_type_(message_type)
{
  timer_ = new wxTimer(this);

  Connect(wxEVT_TIMER, wxTimerEventHandler(TopicDisplay::tick), NULL, this);

  if (auto_refresh)
  {
    timer_->Start(1000);
  }

  root_id_ = topic_tree_->AddRoot(wxT("/"));

  refreshTopics();
}

TopicDisplay::~TopicDisplay()
{
  delete timer_;
}

void TopicDisplay::checkIsTopic(wxTreeEvent& event)
{
  if (topic_tree_->GetItemData(event.GetItem()) == NULL)
  {
    event.Veto();
  }
}

void TopicDisplay::onItemActivated(wxTreeEvent& event)
{
  if (topic_tree_->GetItemData(event.GetItem()) == NULL)
  {
    event.Skip();
  }
  else
  {
    wxPostEvent(this, event);
  }
}

void TopicDisplay::refreshTopics()
{
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);

  // Set all items in cache to tentatively delete
  for (M_TopicInfo::iterator i = topic_cache_.begin(); i != topic_cache_.end(); ++i)
  {
    i->second.save_ = false;
  }

  // Loop through all published topics
  ros::master::V_TopicInfo::iterator it = topics.begin();
  ros::master::V_TopicInfo::iterator end = topics.end();
  for (; it != end; ++it)
  {
    const ros::master::TopicInfo& topic = *it;
    if (!message_type_.empty() && topic.datatype != message_type_)
    {
      continue;
    }

    M_TopicInfo::iterator j = topic_cache_.find(topic.name);
    if (j == topic_cache_.end())
    {
      // Topic not in cache yet.  Find right place to put it in the tree
      std::istringstream iss(topic.name);
      std::string token;

      wxTreeItemId id = root_id_;

      while (getline(iss, token, '/'))
      {
        if (!token.empty())
        {
          wxTreeItemIdValue cookie;

          wxTreeItemId child;

          child = topic_tree_->GetFirstChild(id, cookie);

          bool exists = false;
          while (child.IsOk())
          {
            if (topic_tree_->GetItemText(child) == wxString::FromAscii(token.c_str()))
            {
              exists = true;
              break;
            }

            child = topic_tree_->GetNextChild(id, cookie);
          }

          if (exists)
          {
            id = child;
          }
          else
          {
            id = topic_tree_->AppendItem(id, wxString::FromAscii(token.c_str()));
          }
        }
      }

      // Add to Cache
      TopicInfo cache_item;
      cache_item.item_ = id;
      cache_item.save_ = true;
      cache_item.type_ = topic.datatype;
      topic_cache_[topic.name] = cache_item;

      // Put data in tree item and rename
      TopicNameData* data = new TopicNameData();
      data->name = topic.name;
      topic_tree_->SetItemText(id, wxString::FromAscii(token.c_str()) + wxT(" (") + wxString::FromAscii(topic.datatype.c_str()) + wxT(")"));
      topic_tree_->SetItemData(id, data);
      topic_tree_->SetItemBold(id, true);
    }
    else
    {
      // Topic already in cache -- keep it there.
      j->second.save_ = true;
    }
  }

  std::vector<M_TopicInfo::iterator> to_erase;

  // Tentatively delete all items in cache which should be removed
  for (M_TopicInfo::iterator i = topic_cache_.begin(); i != topic_cache_.end(); i++)
  {
    if (i->second.save_ == false)
    {
      to_erase.push_back(i);
    }
  }

  // Actually delete all items and purge parents as necessary
  for (std::vector<M_TopicInfo::iterator>::iterator i = to_erase.begin(); i != to_erase.end(); i++)
  {
    // Delete item
    wxTreeItemId id = (*i)->second.item_;
    wxTreeItemId parent_id = topic_tree_->GetItemParent(id);
    topic_tree_->Delete(id);

    // Delete all childless parents
    while (parent_id != root_id_)
    {
      if (topic_tree_->HasChildren(parent_id))
      {
        break;
      }
      else
      {
        id = parent_id;
        parent_id = topic_tree_->GetItemParent(id);
        topic_tree_->Delete(id);
      }
    }

    // Erase item from cache
    topic_cache_.erase(*i);
  }

  // Refresh the display
  Refresh();
}

void TopicDisplay::tick(wxTimerEvent& event)
{
  refreshTopics();
}

void TopicDisplay::getSelectedTopics(V_string& topics)
{
  wxArrayTreeItemIds selections;

  topic_tree_->GetSelections(selections);

  for (unsigned int i = 0; i < selections.GetCount(); i++)
  {
    wxTreeItemId id = selections.Item(i);
    if (topic_tree_->GetItemData(id) != NULL)
    {
      TopicNameData* s = (TopicNameData*) topic_tree_->GetItemData(id);
      topics.push_back(s->name);
    }
  }
}

void TopicDisplay::setMultiselectAllowed(bool allowed)
{
  long tree_style = topic_tree_->GetWindowStyle();

  if (allowed)
  {
    tree_style &= ~wxTR_SINGLE;
    tree_style |= wxTR_MULTIPLE;
  }
  else
  {
    tree_style &= ~wxTR_MULTIPLE;
    tree_style |= wxTR_SINGLE;
  }

  topic_tree_->SetWindowStyle(tree_style);
  topic_tree_->Refresh();
}

} // namespace rxtools
