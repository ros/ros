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

#ifndef RXTOOLS_ROSOUT_LIST_CONTROL_H_
#define RXTOOLS_ROSOUT_LIST_CONTROL_H_

#include <wx/wx.h>
#include <wx/listctrl.h>

#include "roslib/Log.h"

#include <boost/function.hpp>

namespace rxtools
{

class RosoutPanel;

namespace columns
{
enum Column
{
  Message,
  Severity,
  Node,
  Time,
  Topics,
  Location,
};
}
typedef columns::Column Column;

/**
 * \brief Custom list control for displaying large numbers of constantly changing messages.
 *
 * wxListCtrl provides a special "virtual" mode that allows it to work with as much data as you can handle.
 * Virtual list controls must at least override the OnGetItemText function, which returns the text to display
 * inside a specific cell.  For more information, see http://docs.wxwidgets.org/stable/wx_wxlistctrl.html
 */
class RosoutListControl : public wxListCtrl
{
public:
  /**
   * \brief Standard wxListCtrl constructor
   */
  RosoutListControl(wxWindow* parent, wxWindowID id, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize, long style = wxLC_ICON,
                    const wxValidator& validator = wxDefaultValidator, const wxString& name = wxListCtrlNameStr);
  /**
   * \brief Destructor
   */
  virtual ~RosoutListControl();

  void setModel(RosoutPanel* model);

  /**
   * \brief Get a text representation of the severity level of a message
   * @param message The message
   * @return A text representation of the severity (ie, "Debug", "Info"...)
   */
  wxString getSeverityText(const roslib::LogConstPtr& message) const;
  /**
   * \brief Get the index of the currently selected item
   * @return The index of the currently selected item
   */
  int32_t getSelection()
  {
    return selection_;
  }

  void setSelection(int32_t index);

  void preItemChanges();
  void postItemChanges();

protected:

  roslib::LogConstPtr getSelectedMessage();

  // Callbacks
  void onItemActivated(wxListEvent& event);
  void onItemSelected(wxListEvent& event);
  void onItemRightClick(wxListEvent& event);

  void onIncludeLocation(wxCommandEvent& event);
  void onIncludeNode(wxCommandEvent& event);
  void onIncludeMessage(wxCommandEvent& event);
  void onExcludeLocation(wxCommandEvent& event);
  void onExcludeNode(wxCommandEvent& event);
  void onExcludeMessage(wxCommandEvent& event);

  void onPopupKeyPressed(wxKeyEvent& event);

  // overrides from wxListCtrl
  virtual wxListItemAttr * OnGetItemAttr(long item) const;
  virtual wxString OnGetItemText(long item, long column) const;
  virtual int OnGetItemImage(long item) const;

  mutable wxListItemAttr attr_;

  RosoutPanel* model_;

  int32_t error_image_id_;
  int32_t warning_image_id_;
  int32_t fatal_image_id_;
  int32_t info_image_id_;
  int32_t debug_image_id_;

  int32_t selection_;

  bool scrollbar_at_bottom_;
  bool disable_scroll_to_bottom_;
};

} // namespace rxtools

#endif /* RXTOOLS_ROSOUT_LIST_CONTROL_H_ */
