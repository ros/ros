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

#include <wx/wx.h>

#include "rxtools/rosout_panel.h"

#include "ros/node.h"

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif

using namespace rxtools;

///Puts the RosoutPanel into a window
class MyFrame: public wxFrame
{
public:
  MyFrame(wxWindow* parent) :
    wxFrame(parent, -1, _("rxconsole"), wxDefaultPosition, wxSize(800,
        600), wxDEFAULT_FRAME_STYLE)
  {
    RosoutPanel* rosout_panel = new RosoutPanel(this);

    rosout_panel->SetSize(this->GetSize());
    rosout_panel->setEnabled(true);
  }

  ~MyFrame()
  {
  }
};

// our normal wxApp-derived class, as usual
class MyApp: public wxApp
{
public:

  bool OnInit()
  {
#ifdef __WXMAC__
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
    SetFrontProcess(&PSN);
#endif

    int argc = 0;
    ros::init(argc, 0, "rxconsole", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    frame->Raise();
    return true;
  }

  int OnExit()
  {
    return 0;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);

