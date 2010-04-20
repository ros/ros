#!/usr/bin/env python
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jonathan Bohren 

import roslib; roslib.load_manifest('rxgraph')
import rospy

import sys
import xdot
import wx

DOTCODE= """digraph G {
  rankdir=LR;
  _included_talker [label="/included/talker"];
  _included_wg2_talker [label="/included/wg2/talker"];
  _included2_talker [label="/included2/talker"];
  _included2_listener [label="/included2/listener"];
  _included_wg2_listener [label="/included/wg2/listener"];
  _rosout [label="/rosout"];
  _included_listener [label="/included/listener"];
  _wg_listener [label="/wg/listener"];
  _wg_talker2 [label="/wg/talker2"];
  _wg_talker1 [label="/wg/talker1"];
  _included2_wg2_talker [label="/included2/wg2/talker"];
  _included2_wg2_listener [label="/included2/wg2/listener"];
    _wg_talker1->_rosout [label="/rosout"]
    _included2_wg2_listener->_rosout [label="/rosout"]
    _wg_talker2->_wg_listener [label="/wg/hello"]
    _wg_talker2->_rosout [label="/rosout"]
    _included2_wg2_talker->_included2_wg2_listener [label="/included2/wg2/chatter"]
    _included2_listener->_rosout [label="/rosout"]
    _included_wg2_listener->_rosout [label="/rosout"]
    _wg_talker1->_wg_listener [label="/wg/hello"]
    _included2_talker->_included2_listener [label="/included2/chatter"]
    _wg_listener->_rosout [label="/rosout"]
    _included_wg2_talker->_included_wg2_listener [label="/included/wg2/chatter"]
    _included_talker->_included_listener [label="/included/chatter"]
    _included_wg2_talker->_rosout [label="/rosout"]
    _included2_talker->_rosout [label="/rosout"]
    _included2_wg2_talker->_rosout [label="/rosout"]
    _included_talker->_rosout [label="/rosout"]
    _included_listener->_rosout [label="/rosout"]}
"""

class RxGraphViewerFrame(wx.Frame):
  def __init__(self):
    wx.Frame.__init__(self, None, -1, "rxgraph", size=(720,480))

    # TODO: convert to read-only property
    self.ns_filter = None
    self.quiet = False
    self.topic_boxes = False
    
    self._needs_refresh = False


    # setup UI
    vbox = wx.BoxSizer(wx.VERTICAL)
    
    # Create Splitter
    self.content_splitter = wx.SplitterWindow(self, -1,style = wx.SP_LIVE_UPDATE)
    self.content_splitter.SetMinimumPaneSize(24)
    self.content_splitter.SetSashGravity(0.85)

    # Create viewer pane
    viewer = wx.Panel(self.content_splitter,-1)

    # Setup the graph panel
    graph_view = wx.Panel(viewer, -1)
    gv_vbox = wx.BoxSizer(wx.VERTICAL)
    graph_view.SetSizer(gv_vbox)

    viewer_box = wx.BoxSizer()
    viewer_box.Add(graph_view,1,wx.EXPAND | wx.ALL, 4)
    viewer.SetSizer(viewer_box)
    
    # Construct toolbar
    toolbar = wx.ToolBar(graph_view, -1)

    toolbar.AddControl(wx.StaticText(toolbar,-1,"Path: "))

    self._ns_combo = wx.ComboBox(toolbar, -1, style=wx.CB_DROPDOWN)
    self._ns_combo .Bind(wx.EVT_COMBOBOX, self.set_path)
    self._ns_combo.Append('/')
    self._ns_combo.SetValue('/')
    self._namespaces = ['/']

    # display options
    quiet_check = wx.CheckBox(toolbar, -1, label="Quiet")
    quiet_check.Bind(wx.EVT_CHECKBOX, self.set_quiet_check)
    topic_check = wx.CheckBox(toolbar, -1, label="All topics")
    topic_check.Bind(wx.EVT_CHECKBOX, self.set_topic_boxes)
    
    toolbar.AddControl(self._ns_combo)

    toolbar.AddControl(wx.StaticText(toolbar,-1,"  "))
    toolbar.AddControl(quiet_check)
    toolbar.AddControl(wx.StaticText(toolbar,-1,"  "))
    toolbar.AddControl(topic_check)

    toolbar.AddControl(wx.StaticText(toolbar,-1,"  "))
    toolbar.AddLabelTool(wx.ID_HELP, 'Help',
        wx.ArtProvider.GetBitmap(wx.ART_HELP,wx.ART_OTHER,(16,16)) )
    toolbar.Realize()

    self.Bind(wx.EVT_TOOL, self.ShowControlsDialog, id=wx.ID_HELP)

    # Create dot graph widget
    self._widget = xdot.wxxdot.WxDotWindow(graph_view, -1)
    self._widget.set_dotcode(DOTCODE)
    self._widget.zoom_to_fit()

    gv_vbox.Add(toolbar, 0, wx.EXPAND)
    gv_vbox.Add(self._widget, 1, wx.EXPAND)


    # Create userdata widget
    borders = wx.LEFT | wx.RIGHT | wx.TOP
    border = 4
    self.ud_win = wx.ScrolledWindow(self.content_splitter, -1)
    self.ud_gs = wx.BoxSizer(wx.VERTICAL)
    self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Info:"),0, borders, border)
    self.ud_txt = wx.TextCtrl(self.ud_win,-1,style=wx.TE_MULTILINE | wx.TE_READONLY)
    self.ud_gs.Add(self.ud_txt,1,wx.EXPAND | borders, border)
    self.ud_win.SetSizer(self.ud_gs)

    # Set content splitter
    self.content_splitter.SplitVertically(viewer, self.ud_win, 512)

    vbox.Add(self.content_splitter, 1, wx.EXPAND | wx.ALL)
    self.SetSizer(vbox)
    self.Center()

    self.Bind(wx.EVT_IDLE,self.OnIdle)

    # user callback for select
    self._widget.register_select_callback(self.select_cb)

  def register_select_cb(self, callback):
    self._widget.register_select_callback(callback)

  def OnIdle(self, event):
    if self._needs_refresh:
      self.Refresh()
      self._needs_refresh = False

  def select_cb(self, *args):
    print args[0].item.url

  def set_quiet_check(self, event):
    self.quiet = event.Checked()

  def set_topic_boxes(self, event):
    self.topic_boxes = event.Checked()
    
  def set_path(self, event):
    self.ns_filter = self._ns_combo.GetValue()
    self._needs_zoom = True

  def update_namespaces(self, namespaces):
    # unfortunately this routine will not alphanumerically sort
    curr = self._namespaces
    new_filters = [n for n in namespaces if n not in curr]
    for f in new_filters:
      self._ns_combo.Append(f)
    curr.extend(new_filters)

  def ShowControlsDialog(self,event):
    dial = wx.MessageDialog(None,
        "Pan: Arrow Keys\nZoom: PageUp / PageDown\nZoom To Fit: F\n",
        'Keyboard Controls', wx.OK)
    dial.ShowModal()

  def set_dotcode(self, dotcode, zoom=True):
    if self._widget.set_dotcode(dotcode, None):
      self.SetTitle('rxgraph')
      if zoom or self._needs_zoom:
        self._widget.zoom_to_fit()
        self._needs_zoom = False
      self._needs_refresh = True
      wx.PostEvent(self.GetEventHandler(), wx.IdleEvent())

    
def main():
  app = wx.App()

  frame = RxGraphViewerFrame()
  frame.Show()

  # for testing only
  frame.set_dotcode(DOTCODE)
  
  app.MainLoop()

if __name__ == '__main__':
    main()
