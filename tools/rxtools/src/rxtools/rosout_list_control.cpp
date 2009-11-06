/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rosout_list_control.h"
#include "rosout_generated.h"
#include "rosout_panel.h"
#include "rosout_text_filter.h"

#include <ros/assert.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/regex.hpp>

#include <wx/imaglist.h>
#include <wx/artprov.h>

#include <sstream>

namespace rxtools
{

RosoutListControl::RosoutListControl(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style, const wxValidator& validator,
                                     const wxString& name)
: wxListCtrl(parent, id, pos, size, style, validator, name), selection_(-1)
, scrollbar_at_bottom_(true)
, disable_scroll_to_bottom_(false)
{
  wxListItem item;
  item.SetText(wxT("Message"));
  item.SetWidth(600);
  InsertColumn(columns::Message, item);
  item.SetText(wxT("Severity"));
  item.SetWidth(100);
  InsertColumn(columns::Severity, item);
  item.SetText(wxT("Node"));
  item.SetWidth(200);
  InsertColumn(columns::Node, item);
  item.SetText(wxT("Time"));
  item.SetWidth(200);
  InsertColumn(columns::Time, item);
  item.SetText(wxT("Topics"));
  item.SetWidth(200);
  InsertColumn(columns::Topics, item);
  item.SetText(wxT("Location"));
  item.SetWidth(600);
  InsertColumn(columns::Location, item);


  wxImageList* image_list = new wxImageList(16, 16);
  fatal_image_id_ = image_list->Add(wxArtProvider::GetIcon(wxART_CROSS_MARK, wxART_OTHER, wxSize(16, 16)));
  error_image_id_ = image_list->Add(wxArtProvider::GetIcon(wxART_ERROR, wxART_OTHER, wxSize(16, 16)));
  warning_image_id_ = image_list->Add(wxArtProvider::GetIcon(wxART_WARNING, wxART_OTHER, wxSize(16, 16)));
  info_image_id_ = image_list->Add(wxArtProvider::GetIcon(wxART_INFORMATION, wxART_OTHER, wxSize(16, 16)));
  debug_image_id_ = image_list->Add(wxArtProvider::GetIcon(wxART_HELP, wxART_OTHER, wxSize(16, 16)));

  AssignImageList(image_list, wxIMAGE_LIST_SMALL);

  Connect(wxEVT_COMMAND_LIST_ITEM_ACTIVATED, wxListEventHandler(RosoutListControl::onItemActivated), NULL, this);
  Connect(wxEVT_COMMAND_LIST_ITEM_RIGHT_CLICK, wxListEventHandler(RosoutListControl::onItemRightClick), NULL, this);
  Connect(wxEVT_COMMAND_LIST_ITEM_SELECTED, wxListEventHandler(RosoutListControl::onItemSelected), NULL, this);
}

RosoutListControl::~RosoutListControl()
{
}

void RosoutListControl::setModel(RosoutPanel* model)
{
  model_ = model;
}

wxString RosoutListControl::getSeverityText(const roslib::LogConstPtr& message) const
{
  switch (message->level)
  {
  case roslib::Log::DEBUG:
    return wxT("Debug");
  case roslib::Log::INFO:
    return wxT("Info");
  case roslib::Log::WARN:
    return wxT("Warn");
  case roslib::Log::ERROR:
    return wxT("Error");
  case roslib::Log::FATAL:
    return wxT("Fatal");
  }

  return wxT("Unknown");
}

int RosoutListControl::OnGetItemImage(long item) const
{
  ROS_ASSERT(model_);

  roslib::LogConstPtr message = model_->getMessageByIndex(item);
  if (!message)
  {
    return -1;
  }

  switch (message->level)
  {
  case roslib::Log::DEBUG:
    return debug_image_id_;
  case roslib::Log::INFO:
    return info_image_id_;
  case roslib::Log::WARN:
    return warning_image_id_;
  case roslib::Log::ERROR:
    return error_image_id_;
  case roslib::Log::FATAL:
    return fatal_image_id_;
  }

  return -1;
}

wxListItemAttr * RosoutListControl::OnGetItemAttr(long item) const
{
#if 0
  ROS_ASSERT(model_);

  const roslib::Log& message = model_->getMessageByIndex(item);

  switch( message->level )
  {
    case roslib::Log::DEBUG:
    attr_.SetBackgroundColour( wxColour( 204, 255, 204 ) );
    break;
    case roslib::Log::INFO:
    attr_.SetBackgroundColour( *wxWHITE );
    break;
    case roslib::Log::WARN:
    attr_.SetBackgroundColour( wxColour( 255, 255, 153 ) );
    break;
    case roslib::Log::ERROR:
    attr_.SetBackgroundColour( wxColour( 255, 153, 0 ) );
    break;
    case roslib::Log::FATAL:
    attr_.SetBackgroundColour( *wxRED );
    break;
    default:
    ROS_BREAK();
  }
#endif

  return &attr_;
}

wxString RosoutListControl::OnGetItemText(long item, long column) const
{
  ROS_ASSERT(model_);

  roslib::LogConstPtr message = model_->getMessageByIndex(item);
  if (!message)
  {
    return wxString();
  }

  switch (column)
  {
  case columns::Severity:
  {
    return getSeverityText(message);
  }
    break;
  case columns::Time:
  {
    std::stringstream ss;
    ss << message->header.stamp;
    return wxString::FromAscii(ss.str().c_str());
  }
  case columns::Message:
  {
    std::string msg = message->msg;
    size_t pos = std::string::npos;
    while (true)
    {
      pos = msg.find('\n');
      if (pos == std::string::npos)
      {
        break;
      }

      msg.replace(pos, 1, "\\n");
    }
    while (true)
    {
      pos = msg.find('\r');
      if (pos == std::string::npos)
      {
        break;
      }

      msg.replace(pos, 1, "\\r");
    }
    return wxString::FromAscii(msg.c_str());
  }
  case columns::Topics:
  {
    std::stringstream ss;
    typedef std::vector<std::string> V_string;
    V_string::const_iterator it = message->topics.begin();
    V_string::const_iterator end = message->topics.end();
    for (; it != end; ++it)
    {
      if (it != message->topics.begin())
      {
        ss << ", ";
      }

      ss << *it;
    }

    return wxString::FromAscii(ss.str().c_str());
  }
  case columns::Location:
  {
    wxString str;
    str << wxString::FromAscii(message->file.c_str()) << wxT(":") << wxString::FromAscii(message->function.c_str()) << wxT(":") << message->line;
    return str;
  }
  case columns::Node:
  {
    return wxString::FromAscii(message->name.c_str());
  }
  }

  ROS_BREAK();
  return wxT("Unknown Column");
}

void RosoutListControl::onPopupKeyPressed(wxKeyEvent& event)
{
  int key = event.GetKeyCode();
  if (key == WXK_ESCAPE)
  {
    wxObject* obj = event.GetEventObject();
    wxWindow* win = wxDynamicCast(obj, wxWindow);
    if (win)
    {
      if (win->GetParent())
      {
        win->GetParent()->Close();
      }
      else
      {
        win->Close();
      }
    }
  }
  // For some reason the built-in ctrl-C handling doesn't work in linux
  else if (key == 3)
  {
    wxObject* obj = event.GetEventObject();
    wxRichTextCtrl* text = wxDynamicCast(obj, wxRichTextCtrl);
    if (text)
    {
      text->Copy();
    }

    event.Skip();
  }
  else
  {
    event.Skip();
  }
}

void RosoutListControl::onItemActivated(wxListEvent& event)
{
  ROS_ASSERT(model_);

  roslib::LogConstPtr message = model_->getMessageByIndex(event.GetIndex());
  if (!message)
  {
    return;
  }

  TextboxDialog* dialog = new TextboxDialog(this, wxID_ANY);
  dialog->Show();
  dialog->text_->SetFocus();
  dialog->Connect(wxEVT_CHAR, wxKeyEventHandler(RosoutListControl::onPopupKeyPressed), NULL, this);
  dialog->text_->Connect(wxEVT_CHAR, wxKeyEventHandler(RosoutListControl::onPopupKeyPressed), NULL, this);

  wxRichTextCtrl& t = *dialog->text_;

  // Node name
  {
    t.BeginBold();
    t.WriteText(wxT("Node: "));
    t.EndBold();
    t.WriteText(wxString::FromAscii(message->name.c_str()));
    t.Newline();
  }

  // Time
  {
    t.BeginBold();
    t.WriteText(wxT("Time: "));
    t.EndBold();
    std::stringstream ss;
    ss << message->header.stamp;
    t.WriteText(wxString::FromAscii(ss.str().c_str()));
    t.Newline();
  }

  // Severity
  {
    t.BeginBold();
    t.WriteText(wxT("Severity: "));
    t.EndBold();
    t.WriteText(getSeverityText(message));
    t.Newline();
  }

  // Location
  {
    if (!message->file.empty())
    {
      t.BeginBold();
      t.WriteText(wxT("Location: "));
      t.EndBold();
      std::stringstream ss;
      ss << message->file << ":" << message->function << ":" << message->line;
      t.WriteText(wxString::FromAscii(ss.str().c_str()));
      t.Newline();
    }
  }

  // Published Topics
  {
    t.BeginBold();
    t.WriteText(wxT("Published Topics: "));
    t.EndBold();
    t.WriteText(wxString::FromAscii(boost::join(message->topics, ", ").c_str()));
    t.Newline();
  }

  // Message
  {
    t.Newline();
    t.BeginTextColour(wxColour(127, 61, 2));
    t.BeginBold();
    t.WriteText(wxString::FromAscii(message->msg.c_str()));
    t.EndTextColour();
    t.EndBold();
  }
}

std::string escapeForRegex(const std::string& str)
{
  const static boost::regex esc("[\\^\\.\\$\\|\\(\\)\\[\\]\\*\\+\\?\\/\\\\]");
  const static std::string rep("\\\\\\1&");
  return boost::regex_replace(str, esc, rep, boost::match_default | boost::format_sed);
}

void RosoutListControl::onExcludeLocation(wxCommandEvent& event)
{
  if (selection_ == -1)
  {
    return;
  }

  roslib::LogConstPtr message = model_->getMessageByIndex(selection_);
  if (!message)
  {
    return;
  }

  RosoutTextFilterPtr filter = model_->createTextFilter();
  filter->setFilterType(RosoutTextFilter::Exclude);
  filter->setFieldMask(RosoutTextFilter::Location);
  std::stringstream ss;
  ss << message->file << ":" << message->function << ":" << message->line;
  filter->setText("^" + escapeForRegex(ss.str()) + "$");
  filter->setUseRegex(true);
}

void RosoutListControl::onExcludeNode(wxCommandEvent& event)
{
  if (selection_ == -1)
  {
    return;
  }

  roslib::LogConstPtr message = model_->getMessageByIndex(selection_);
  if (!message)
  {
    return;
  }

  RosoutTextFilterPtr filter = model_->createTextFilter();
  filter->setFilterType(RosoutTextFilter::Exclude);
  filter->setFieldMask(RosoutTextFilter::Node);
  filter->setText("^" + escapeForRegex(message->name) + "$");
  filter->setUseRegex(true);
}

void RosoutListControl::onExcludeMessage(wxCommandEvent& event)
{
  if (selection_ == -1)
  {
    return;
  }

  roslib::LogConstPtr message = model_->getMessageByIndex(selection_);
  if (!message)
  {
    return;
  }

  RosoutTextFilterPtr filter = model_->createTextFilter();
  filter->setFilterType(RosoutTextFilter::Exclude);
  filter->setFieldMask(RosoutTextFilter::Message);
  filter->setText("^" + escapeForRegex(message->msg) + "$");
  filter->setUseRegex(true);
}

void RosoutListControl::onIncludeLocation(wxCommandEvent& event)
{
  if (selection_ == -1)
  {
    return;
  }

  roslib::LogConstPtr message = model_->getMessageByIndex(selection_);
  if (!message)
  {
    return;
  }

  RosoutTextFilterPtr filter = model_->createTextFilter();
  filter->setFilterType(RosoutTextFilter::Include);
  filter->setFieldMask(RosoutTextFilter::Location);
  std::stringstream ss;
  ss << message->file << ":" << message->function << ":" << message->line;
  filter->setText("^" + escapeForRegex(ss.str()) + "$");
  filter->setUseRegex(true);
}

void RosoutListControl::onIncludeNode(wxCommandEvent& event)
{
  if (selection_ == -1)
  {
    return;
  }

  roslib::LogConstPtr message = model_->getMessageByIndex(selection_);
  if (!message)
  {
    return;
  }

  RosoutTextFilterPtr filter = model_->createTextFilter();
  filter->setFilterType(RosoutTextFilter::Include);
  filter->setFieldMask(RosoutTextFilter::Node);
  filter->setText("^" + escapeForRegex(message->name) + "$");
  filter->setUseRegex(true);
}

void RosoutListControl::onIncludeMessage(wxCommandEvent& event)
{
  if (selection_ == -1)
  {
    return;
  }

  roslib::LogConstPtr message = model_->getMessageByIndex(selection_);
  if (!message)
  {
    return;
  }

  RosoutTextFilterPtr filter = model_->createTextFilter();
  filter->setFilterType(RosoutTextFilter::Include);
  filter->setFieldMask(RosoutTextFilter::Message);
  filter->setText("^" + escapeForRegex(message->msg) + "$");
  filter->setUseRegex(true);
}

void RosoutListControl::onItemRightClick(wxListEvent& event)
{
  if (selection_ == -1)
  {
    return;
  }

  ROS_ASSERT(model_);

  roslib::LogConstPtr message = model_->getMessageByIndex(event.GetIndex());
  if (!message)
  {
    return;
  }

  wxMenu* menu = new wxMenu(wxT(""));
  wxMenu* exclude_menu = new wxMenu(wxT(""));
  wxMenu* include_menu = new wxMenu(wxT(""));
  wxMenuItem* item = 0;

  // Setup the include menu
  {
    if (!message->file.empty())
    {
      item = include_menu->Append(wxID_ANY, wxT("This Location"));
      Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(RosoutListControl::onIncludeLocation), NULL, this);
    }

    item = include_menu->Append(wxID_ANY, wxT("This Node"));
    Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(RosoutListControl::onIncludeNode), NULL, this);
    item = include_menu->Append(wxID_ANY, wxT("This Message"));
    Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(RosoutListControl::onIncludeMessage), NULL, this);

    menu->AppendSubMenu(include_menu, wxT("Include"));
  }

  // Setup the exclude menu
  {
    if (!message->file.empty())
    {
      item = exclude_menu->Append(wxID_ANY, wxT("This Location"));
      Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(RosoutListControl::onExcludeLocation), NULL, this);
    }

    item = exclude_menu->Append(wxID_ANY, wxT("This Node"));
    Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(RosoutListControl::onExcludeNode), NULL, this);
    item = exclude_menu->Append(wxID_ANY, wxT("This Message"));
    Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(RosoutListControl::onExcludeMessage), NULL, this);

    menu->AppendSubMenu(exclude_menu, wxT("Exclude"));
  }

  PopupMenu(menu);
}

void RosoutListControl::onItemSelected(wxListEvent& event)
{
  selection_ = event.GetIndex();

  disable_scroll_to_bottom_ = true;
}

void RosoutListControl::preItemChanges()
{
  /// @todo wxListCtrl::GetScrollRange doesn't work, so I have to work around it.  Switch to use GetScrollPos and GetScrollRange once Bug #10155 in the wxWidgets trac is fixed.
  scrollbar_at_bottom_ = false;
  int count_per_page = GetCountPerPage();
  int scroll_pos = GetScrollPos(wxVERTICAL);
#if __WXMAC__
  // wxListCtrl::GetScrollPos has different behavior on OSX, adjust for that
  //--count_per_page;
  int32_t item_height = 20;
  if (GetItemCount() > 0)
  {
    wxRect rect;
    if (GetItemRect(0, rect))
    {
      // For some reason this is always returning -1 right now, so we default to 20 (above)
      if (rect.GetHeight() > 0)
      {
        item_height = rect.GetHeight();
      }
    }
  }

  scroll_pos /= item_height;
#endif
  if (scroll_pos + count_per_page >= GetItemCount())
  {
    scrollbar_at_bottom_ = true;
  }

  Freeze();
}

void RosoutListControl::postItemChanges()
{
  if (!disable_scroll_to_bottom_ && scrollbar_at_bottom_ && GetItemCount() > 0)
  {
    EnsureVisible(GetItemCount() - 1);
  }

  disable_scroll_to_bottom_ = false;

  Thaw();

  // This for some reason prevents the control from flickering: http://wiki.wxwidgets.org/Flicker-Free_Drawing#No-flickering_for_wxListCtrl_with_wxLC_REPORT_.7C_wxLC_VIRTUAL_style
  wxIdleEvent idle;
  wxTheApp->SendIdleEvents(this, idle);
}

void RosoutListControl::setSelection(int32_t index)
{
  if (index == -1)
  {
    if (selection_ != -1)
    {
      SetItemState(selection_, 0, wxLIST_STATE_SELECTED|wxLIST_STATE_FOCUSED);
    }
  }
  else
  {
    SetItemState(index, wxLIST_STATE_SELECTED, wxLIST_STATE_SELECTED);
  }
}

} // namespace rxtools
