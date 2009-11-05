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

/*
 * wx panel for viewing rosout.
 *
 * Written by Josh Faust
 */
#ifndef RXTOOLS_ROSOUT_PANEL_H
#define RXTOOLS_ROSOUT_PANEL_H

#include "rosout_generated.h"
#include "rosout_filter.h"
#include "roslib/Log.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <string>
#include <vector>
#include <map>
#include <set>

#include "boost/thread/mutex.hpp"

class wxTimer;
class wxTimerEvent;
class wxAuiNotebook;
class wxRichTextCtrl;
class wxScrolledWindow;
class wxSizer;
class wxBitmapButton;
class wxCheckBox;
class wxWindow;
class wxPanel;

namespace rxtools
{

class RosoutSeverityFilter;
typedef boost::shared_ptr<RosoutSeverityFilter> RosoutSeverityFilterPtr;

typedef unsigned int uint32_t; // for swig

struct RosoutMessageSummary
{
  RosoutMessageSummary()
  : debug(0)
  , info(0)
  , warn(0)
  , error(0)
  , fatal(0)
  {}

  ~RosoutMessageSummary()
  {}

  uint32_t debug;
  uint32_t info;
  uint32_t warn;
  uint32_t error;
  uint32_t fatal;
};

/**
 * \class RosoutPanel
 * \brief An embeddable panel which listens on rosout and displays any messages that arrive.
 */
class RosoutPanel : public RosoutPanelBase
{
public:
  struct FilterInfo
  {
    RosoutFilterPtr filter;
    wxWindow* control;
    wxSizer* sizer;
    wxCheckBox* enabled_cb;
    wxBitmapButton* delete_button;
    wxBitmapButton* up_button;
    wxBitmapButton* down_button;
    wxPanel* panel;
  };

public:
  /**
   * \brief Constructor
   * @param parent The window which is the parent of this one
   */
  RosoutPanel(wxWindow* parent);
  ~RosoutPanel();

  /**
   * \brief Set this panel to be enabled or not.
   *
   * When enabled, it will be subscribed to the rosout topic and processing messages.  When disabled, it will not.
   * @param enabled Should we be enabled?
   */
  void setEnabled(bool enabled);
  /**
   * \brief Set the topic to listen on for roslib::Log messages
   * @param topic The topic name
   */
  void setTopic(const std::string& topic);

  /**
   * \brief Clear all messages
   */
  void clear();

  /**
   * \brief Set the number of messages to display before we start throwing away old ones
   * @param size The number of messages
   */
  void setBufferSize(uint32_t size);

  /**
   * \brief Returns a summary of the types of messages that have been received recently.
   * \param duration Controls how far in the past the function searches for messages
   * \return a RosoutMessageSummary structure detailing the types of messages received.
   */
  RosoutMessageSummary getMessageSummary(double duration) const;

  void addFilter(const RosoutFilterPtr& filter, wxWindow* control);
  void removeFilter(const RosoutFilterPtr& filter);

protected:
  /**
   * \brief (wx callback) Called when the "Setup" button is pressed
   */
  virtual void onSetup(wxCommandEvent& event);
  /**
   * \brief (wx callback) Called when the "Pause" button is pressed
   */
  virtual void onPause(wxCommandEvent& event);
  /**
   * \brief (wx callback) Called when the "Clear" button is pressed
   */
  virtual void onClear(wxCommandEvent& event);

  /**
   * \brief (wx callback) Called every 100ms so we can process new messages
   */
  void onProcessTimer(wxTimerEvent& evt);

  void onAddFilterPressed(wxCommandEvent& event);
  void onFilterEnableChecked(wxCommandEvent& event);
  void onFilterDelete(wxCommandEvent& event);
  void onFilterMoveUp(wxCommandEvent& event);
  void onFilterMoveDown(wxCommandEvent& event);

  ////////////////////////////////////////////////////////////////////////////////////////

  void onFilterChanged(const RosoutFilter*);

  /**
   * \brief subscribe to our topic
   */
  void subscribe();
  /**
   * \brief unsubscribe from our topic
   */
  void unsubscribe();

  /**
   * \brief (ros callback) Called when there is a new message waiting
   */
  void incomingMessage(const roslib::Log::ConstPtr& message);
  /**
   * \brief Processes any messages in our message queue
   */
  void processMessages();
  /**
   * \brief Process a log message
   * @param message The message to process
   */
  void processMessage(const roslib::Log::ConstPtr& message);
  /**
   * \brief Add a message to the table
   * @param message The message
   * @param id The unique id of the message
   */
  void addMessageToTable(const roslib::Log::ConstPtr& message, uint32_t id);

  /**
   * \brief Filter a message based on our current filter
   * @param id The id of the message to filter
   * @return True of anything in the message matches, false otherwise
   */
  bool filter(uint32_t id) const;
  /**
   * \brief Re-filter all messages
   */
  void refilter();

  /**
   * \brief Get a message by index in our ordered message list.  Used by the list control.
   * @param index Index of the message to return
   * @return The message
   */
  roslib::LogConstPtr getMessageByIndex(uint32_t index) const;

  /**
   * \brief Remove The oldest message
   */
  void popMessage();

  void resizeFiltersPane();
  void updateFilterBackgrounds();

  bool enabled_; ///< Are we enabled?
  std::string topic_; ///< The topic we're listening on (or will listen on once we're enabled)

  ros::NodeHandle nh_;

  typedef std::vector<roslib::Log::ConstPtr> V_Log;
  V_Log message_queue_; ///< Queue of messages we've received since the last time processMessages() was called

  wxTimer* process_timer_; ///< Timer used to periodically process messages

  uint32_t message_id_counter_; ///< Counter for generating unique ids for messages
  typedef std::map<uint32_t, roslib::Log::ConstPtr> M_IdToMessage;
  M_IdToMessage messages_; ///< Map of id->message

  typedef std::vector<uint32_t> V_u32;
  V_u32 ordered_messages_; ///< Already-filtered messages that are being displayed in the list

  uint32_t max_messages_; ///< Max number of messages to keep around.  When we hit this limit, we start throwing away the oldest messages

  bool needs_refilter_; ///< Set to true when we need to refilter our messages (ie. a filter has changed)
  float refilter_timer_; ///< Accumulator used to gate how often we refilter

  ros::CallbackQueue callback_queue_;
  ros::Subscriber sub_;

  typedef std::vector<FilterInfo> V_FilterInfo;
  V_FilterInfo filters_;
  // special severity filter, of which there is only one (and it's not in the "filters" collapsible pane)
  RosoutSeverityFilterPtr severity_filter_;

  wxBitmap delete_filter_bitmap_;

  bool pause_;
};

} // namespace rxtools

#endif // RXTOOLS_ROSOUT_PANEL_H
