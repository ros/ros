#!/usr/bin/env python
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

PKG = 'rosh'
import roslib; roslib.load_manifest(PKG)
import rospy
import optparse
import sys
import wx
import Image

class PyImageViewFrame(wx.Frame):
    def __init__(self, topic, map=False):
        if map:
            wx.Frame.__init__(self, None, wx.ID_ANY, topic, pos=wx.DefaultPosition, size=(1000, 1000), style=wx.DEFAULT_FRAME_STYLE)
        else:
            wx.Frame.__init__(self, None, wx.ID_ANY, topic, pos=wx.DefaultPosition, size=(640, 480), style=wx.DEFAULT_FRAME_STYLE)
            
        self.image  = None
        self.map  = None
        self.bitmap = None
        self.dirty = False

        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SIZE,  lambda e: self.convert_bitmap())

        if not map:
            import sensor_msgs.msg
            rospy.Subscriber(topic, sensor_msgs.msg.Image, lambda img_msg: wx.CallAfter(self.set_image_msg, img_msg))
        else:
            import nav_msgs.msg
            rospy.Subscriber(topic, nav_msgs.msg.OccupancyGrid, lambda map_msg: wx.CallAfter(self.set_map_msg, map_msg))

    def set_image_msg(self, img_msg):
        self.image = self.imgmsg_to_wx(img_msg)
        self.dirty = True
        self.convert_bitmap()

    def set_map_msg(self, msg):
        self.image = self.occgridmsg_to_wx(msg)
        self.dirty = True
        self.convert_bitmap()
        
    def on_paint(self, event):
        if self.bitmap:
            wx.ClientDC(self).DrawBitmap(self.bitmap, 0, 0)

    def convert_bitmap(self):
        if self.image:
            width = self.image.GetWidth()
            height = self.image.GetHeight()
            target_width, target_height = self.GetClientSize()

            if self.dirty or ((width, height) != (target_width, target_height)):
                # Rescale operates in place
                cp = self.image.Copy()
                # 1) try to box to width first
                new_width = target_width
                new_height = int((float(target_width) / float(width)) * float(height))
                if new_height > target_height:
                    # 1) try to box to height instead
                    new_height = target_height
                    new_width = int((float(target_height) / float(height)) * float(width))

                cp.Rescale(new_width, new_height)
                cp.Resize((self.GetClientSize()[0], self.GetClientSize()[1]), wx.Point(0, 0), 0, 0, 0)
                self.bitmap = cp.ConvertToBitmap()
                self.dirty = False
            self.Refresh()

    def imgmsg_to_wx(self, img_msg):
        if img_msg.encoding == 'rgb8':
            return wx.ImageFromBuffer(img_msg.width, img_msg.height, img_msg.data)
        elif img_msg.encoding in ['mono8', 'bayer_rggb8', 'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8']:
            mode = 'L'
        elif img_msg.encoding == 'rgb8':
            mode = 'RGB'
        elif img_msg.encoding == 'bgr8':
            mode = 'BGR'
        elif img_msg.encoding == 'mono16':
            if img_msg.is_bigendian:
                mode = 'F;16B'
            else:
                mode = 'F:16'
        elif img_msg.encoding in ['rgba8', 'bgra8']:
            return None

        pil_img = Image.frombuffer('RGB', (img_msg.width, img_msg.height), img_msg.data, 'raw', mode, 0, 1)
        if pil_img.mode != 'RGB':
            pil_img = pil_img.convert('RGB')

        return wx.ImageFromData(pil_img.size[0], pil_img.size[1], pil_img.tostring())

    def occgridmsg_to_wx(self, map):
        # adapted from map_tiler
        size = (map.info.width, map.info.height)
        s = "".join([maptrans(x) for x in map.data])
        return wx.ImageFromData(map.info.width, map.info.height, s)

s128 = chr(128)*3
s0 = chr(0)*3
s255 = chr(255)*3
def maptrans(x):
  if x == -1:
      return s128
  elif x == 0:
      return s255
  elif x == 100:
      return s0
  else:
      return chr(x)*3
                        
if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('-t', '--topic', action='store', default='/image', help='topic to listen to image msg on')
    parser.add_option('-m', '--map', action='store_true', help='topic is is nav_msgs/OccupancyGrid')
    options, args = parser.parse_args(sys.argv[1:])

    rospy.init_node('py_image_view', anonymous=True)

    app = wx.PySimpleApp()
    frame = PyImageViewFrame(options.topic, map=options.map)
    frame.Show()
    app.MainLoop()
