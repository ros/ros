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

#ifndef RXTOOLS_ROSOUT_TEXT_FILTER_CONTROL_H
#define RXTOOLS_ROSOUT_TEXT_FILTER_CONTROL_H

#include "rosout_generated.h"
#include <boost/shared_ptr.hpp>
#include <boost/signals/connection.hpp>

namespace rxtools
{

class RosoutTextFilter;
typedef boost::shared_ptr<RosoutTextFilter> RosoutTextFilterPtr;

class RosoutTextFilterControl : public RosoutTextFilterControlBase
{
public:
  RosoutTextFilterControl(wxWindow* parent, const RosoutTextFilterPtr& filter);

protected:
  void checkValid();
  void setIncludeExcludeColor();
  void read();

  virtual void onIncludeExclude( wxCommandEvent& event );
  virtual void onRegex( wxCommandEvent& event );
  virtual void onText( wxCommandEvent& event );
  virtual void onMessage( wxCommandEvent& event );
  virtual void onNode( wxCommandEvent& event );
  virtual void onLocation( wxCommandEvent& event );
  virtual void onTopics( wxCommandEvent& event );

private:
  RosoutTextFilterPtr filter_;
  boost::signals::connection filter_changed_connection_;
};

}

#endif // RXTOOLS_ROSOUT_TEXT_FILTER_CONTROL_H
