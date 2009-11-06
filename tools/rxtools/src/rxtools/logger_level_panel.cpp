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

#include "logger_level_panel.h"

#include <ros/ros.h>
#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>

#include <wx/msgdlg.h>

#include <algorithm>

namespace rxtools
{

LoggerLevelPanel::LoggerLevelPanel(wxWindow* parent)
: LoggerLevelPanelBase(parent, wxID_ANY)
{
  fillNodeList();
}

LoggerLevelPanel::~LoggerLevelPanel()
{

}

void LoggerLevelPanel::fillNodeList()
{
  nodes_box_->Clear();

  ros::V_string nodes;
  ros::master::getNodes(nodes);

  std::sort(nodes.begin(), nodes.end());

  ros::V_string::iterator it = nodes.begin();
  ros::V_string::iterator end = nodes.end();
  for (; it != end; ++it)
  {
    const std::string& node = *it;

    if (ros::service::exists(node + "/get_loggers", false))
    {
      nodes_box_->Append(wxString::FromAscii(node.c_str()));
    }
  }
}

void LoggerLevelPanel::onNodesRefresh( wxCommandEvent& event )
{
  fillNodeList();
}

void LoggerLevelPanel::onNodeSelected( wxCommandEvent& event )
{
  loggers_box_->Clear();
  levels_box_->Clear();
  loggers_.clear();

  std::string node = (const char*)nodes_box_->GetStringSelection().fn_str();
  if (node.empty())
  {
    return;
  }

  roscpp::GetLoggers srv;
  if (ros::service::call(node + "/get_loggers", srv))
  {
    std::vector<roscpp::Logger>::iterator it = srv.response.loggers.begin();
    std::vector<roscpp::Logger>::iterator end = srv.response.loggers.end();
    for (; it != end; ++it)
    {
      roscpp::Logger& logger = *it;
      const std::string& name = logger.name;

      loggers_[name] = logger.level;

      loggers_box_->Append(wxString::FromAscii(name.c_str()));
    }
  }
  else
  {
    wxString msg;
    msg.Printf(wxT("Failed to call service [%s/get_loggers].  Did the node go away?"), wxString::FromAscii(node.c_str()).c_str());
    wxMessageBox(msg, wxT("Failed to lookup loggers"), wxOK|wxICON_ERROR);
  }
}

void LoggerLevelPanel::onLoggerSelected( wxCommandEvent& event )
{
  levels_box_->Clear();

  std::string logger = (const char*)loggers_box_->GetStringSelection().fn_str();
  if (logger.empty())
  {
    return;
  }

  M_string::iterator it = loggers_.find(logger);
  std::string level = it->second;

  std::transform(level.begin(), level.end(), level.begin(), (int(*)(int))std::toupper);

  int selection = 0;
  if (level == "DEBUG")
  {
    selection = 0;
  }
  else if (level == "INFO")
  {
    selection = 1;
  }
  else if (level == "WARN")
  {
    selection = 2;
  }
  else if (level == "ERROR")
  {
    selection = 3;
  }
  else if (level == "FATAL")
  {
    selection = 4;
  }
  else
  {
    ROS_ERROR("Unknown logger level [%s]", level.c_str());
    selection = -1;
  }

  levels_box_->Append(wxT("Debug"));
  levels_box_->Append(wxT("Info"));
  levels_box_->Append(wxT("Warn"));
  levels_box_->Append(wxT("Error"));
  levels_box_->Append(wxT("Fatal"));
  levels_box_->SetSelection(selection);
}

void LoggerLevelPanel::onLevelSelected( wxCommandEvent& event )
{
  std::string level = (const char*)levels_box_->GetStringSelection().fn_str();
  if (level.empty())
  {
    return;
  }

  std::string node = (const char*)nodes_box_->GetStringSelection().fn_str();
  std::string logger = (const char*)loggers_box_->GetStringSelection().fn_str();

  roscpp::SetLoggerLevel srv;
  srv.request.logger = logger;
  srv.request.level = level;
  if (!ros::service::call(node + "/set_logger_level", srv))
  {
    wxString msg;
    msg.Printf(wxT("Failed to call service [%s/set_logger_level].  Did the node go away?"), wxString::FromAscii(node.c_str()).c_str());
    wxMessageBox(msg, wxT("Failed to set logger level"), wxOK|wxICON_ERROR);
  }
  else
  {
    loggers_[logger] = level;
  }
}

} // namespace rxtools
