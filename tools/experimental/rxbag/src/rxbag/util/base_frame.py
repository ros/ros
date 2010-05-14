# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import sys
import wx

## A Frame that can save/load its position and size. 
class BaseFrame(wx.Frame):
    def __init__(self, parent, config_name, config_key, id=wx.ID_ANY, title='Untitled', pos=wx.DefaultPosition, size=(800, 300), style=wx.DEFAULT_FRAME_STYLE):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)

        self.config     = wx.Config(config_name)
        self.config_key = config_key

        self._load_config()
        
        self.Bind(wx.EVT_CLOSE, self.on_close)

    def on_close(self, event):
        self._save_config()

        self.Destroy()

    ## Load position and size of the frame from config
    def _load_config(self):
        (x, y), (width, height) = self.GetPositionTuple(), self.GetSizeTuple()
        if self.config.HasEntry(self._config_x):      x      = self.config.ReadInt(self._config_x)
        if self.config.HasEntry(self._config_y):      y      = self.config.ReadInt(self._config_y)
        if self.config.HasEntry(self._config_width):  width  = self.config.ReadInt(self._config_width)
        if self.config.HasEntry(self._config_height): height = self.config.ReadInt(self._config_height)       

        self.SetPosition((x, y))
        self.SetSize((width, height))

    ## Save position and size of frame to config
    def _save_config(self):
        (x, y), (width, height) = self.GetPositionTuple(), self.GetSizeTuple()
        
        self.config.WriteInt(self._config_x,      x)
        self.config.WriteInt(self._config_y,      y)
        self.config.WriteInt(self._config_width,  width)
        self.config.WriteInt(self._config_height, height)       
        self.config.Flush()

    @property
    def _config_x(self): return self._config_property('X')

    @property
    def _config_y(self): return self._config_property('Y')

    @property
    def _config_width(self): return self._config_property('Width')

    @property
    def _config_height(self): return self._config_property('Height')

    def _config_property(self, property): return '/%s/%s' % (self.config_key, property)
