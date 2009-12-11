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

PKG = 'rxplay'
import roslib; roslib.load_manifest(PKG)
import rospy
import time
import wx

class Layer:
    name = 'Untitled'
    
    def __init__(self, parent, title, x, y, width, height, max_repaint=None):
        self._x      = x
        self._y      = y
        self._width  = width
        self._height = height

        self.parent      = parent
        self.title       = title    
        self.max_repaint = max_repaint

        self.self_paint = False

        if not self.self_paint:
            self.bitmap = wx.EmptyBitmap(self._width, self._height)

        self._last_repaint  = None
        self._dirty         = True
        self._force_repaint = False

    # Interface to implement in derived classes

    def check_dirty(self):           pass
    def paint(self, dc):             pass
    def on_mouse_move(self, event):  pass
    def on_mousewheel(self, event):  pass
    def on_left_down(self, event):   pass
    def on_middle_down(self, event): pass
    def on_right_down(self, event):  pass
    def on_size(self, event):        pass
    def on_close(self, event):       pass

    def move(self, x, y):
        if self._x == x and self._y == y:
            return
        
        self._x, self._y = x, y
        
        self.force_repaint()

    def resize(self, width, height):
        if self._width == width and self._height == height:
            return 
        
        self._width, self._height = width, height
        
        if not self.self_paint:
            self.bitmap = wx.EmptyBitmap(self._width, self._height)

        self.force_repaint()

    def contains(self, x, y):
        return x >= self._x and y >= self._y and x <= self.right and y <= self.bottom

    def invalidate(self):
        self._dirty = True
        
    def force_repaint(self):
        self._dirty         = True
        self._force_repaint = True

    @property
    def x(self): return self._x

    @property
    def y(self): return self._y

    @property
    def width(self): return self._width

    @property
    def height(self): return self._height

    @property
    def right(self): return self._x + self._width
    
    @property
    def bottom(self): return self._y + self._height

    # Painting

    def draw(self, dc):
        if not self.self_paint:
            dc.DrawBitmap(self.bitmap, self._x, self._y)

    def should_repaint(self):
        # Repaint if forced
        if self._force_repaint:
            return True
        
        # Don't repaint if the layer contents aren't dirty
        if not self._dirty:
            return False

        # Don't repaint if it was repainted less than max_repaint seconds ago
        if self.max_repaint and self._last_repaint and time.time() - self._last_repaint < self.max_repaint:
            return False
            
        return True

    def paint_to_bitmap(self):
        dc = wx.MemoryDC()
        dc.SelectObject(self.bitmap)
        self.paint(dc)
        dc.SelectObject(wx.NullBitmap)

        self._last_repaint  = time.time()
        self._dirty         = False
        self._force_repaint = False

class TransparentLayer(Layer):
    TRANSPARENT_COLOR = wx.Colour(5, 5, 5) 

    def __init__(self, parent, title, x, y, width, height, max_repaint=None):
        Layer.__init__(self, parent, title, x, y, width, height, max_repaint)

        self._transparent_bitmap = None

        self.bitmap.SetMask(wx.Mask(self.bitmap, self.TRANSPARENT_COLOR))

    def paint_to_bitmap(self):
        Layer.paint_to_bitmap(self)

        self._transparent_bitmap = self._make_transparent(self.bitmap)

    def draw(self, dc):
        if self._transparent_bitmap:
            dc.DrawBitmap(self._transparent_bitmap, self.x, self.y, useMask=True)

    def paint(self, dc):
        dc.SetBackground(wx.Brush(self.TRANSPARENT_COLOR, wx.SOLID))
        dc.Clear()

    ## A bug in wxPython with transparent bitmaps: need to convert to/from Image to enable transparency.
    ## (see http://aspn.activestate.com/ASPN/Mail/Message/wxpython-users/3668628)
    def _make_transparent(self, bitmap):
        image = bitmap.ConvertToImage()
        if not image.HasAlpha():
            image.InitAlpha()

        w, h = image.GetWidth(), image.GetHeight()
        for y in xrange(h):
            for x in xrange(w):
                pix = wx.Colour(image.GetRed(x, y), image.GetGreen(x, y), image.GetBlue(x, y))
                if pix == self.TRANSPARENT_COLOR:
                    image.SetAlpha(x, y, 0)

        return image.ConvertToBitmap()

class LayerPanel(wx.Window):
    def __init__(self, *args, **kwargs):
        wx.Window.__init__(self, *args, **kwargs)
        
        self.background_brush = wx.WHITE_BRUSH

        self.bitmap      = None
        self.layers      = []
        self.clicked_pos = None
        self.painting    = False

        self.Bind(wx.EVT_PAINT,       self.on_paint)
        self.Bind(wx.EVT_SIZE,        self.on_size)
        self.Bind(wx.EVT_TIMER,       self.on_timer)
        self.Bind(wx.EVT_LEFT_DOWN,   self.on_left_down)
        self.Bind(wx.EVT_MIDDLE_DOWN, self.on_middle_down)
        self.Bind(wx.EVT_RIGHT_DOWN,  self.on_right_down)
        self.Bind(wx.EVT_MOTION,      self.on_mouse_move)
        self.Bind(wx.EVT_MOUSEWHEEL,  self.on_mousewheel)
        
        self.GetParent().Bind(wx.EVT_CLOSE, self.on_close)

        # Repaint frequently
        self.timer = wx.Timer(self)
        self.timer.Start(milliseconds=20, oneShot=False)

    @property
    def width(self): return self.bitmap.GetSize()[0]

    @property
    def height(self): return self.bitmap.GetSize()[1]

    # Painting events

    def on_paint(self, event):
        self.paint()
        
    def paint(self):
        paint_layers = [layer for layer in self.layers if not layer.self_paint]
        if len(paint_layers) == 0:
            return
        
        # Compose all layers into a buffer
        bitmap_dc = wx.MemoryDC()
        bitmap_dc.SelectObject(self.bitmap)
        bitmap_dc.SetBackground(self.background_brush)
        bitmap_dc.Clear()
        for layer in paint_layers:
            layer.draw(bitmap_dc)
        bitmap_dc.SelectObject(wx.NullBitmap)

        # Draw buffer to the window
        window_dc = wx.ClientDC(self)
        window_dc.DrawBitmap(self.bitmap, 0, 0)

    def on_timer(self, event):
        if not self.bitmap:
            return

        # Find layers that need to be repainted
        for layer in self.layers:
            layer.check_dirty()
        dirty_layers = [layer for layer in self.layers if layer.should_repaint()]

        # If none are dirty, nothing to do
        if len(dirty_layers) == 0:
            return

        # Repaint dirty layers
        for layer in dirty_layers:
            layer.paint_to_bitmap()

        # Compose layers and blit to window 
        self.paint()

    def on_size(self, event):
        size = self.GetClientSize()
        if not self.bitmap or self.bitmap.GetSize() != size:
            self.bitmap = wx.EmptyBitmap(*self.GetClientSize())
            
            for layer in self.layers:
                layer.on_size(event)

    # Mouse events

    def on_left_down(self, event):
        self.clicked_pos = event.GetPosition()
        for layer in self.layers:
            layer.on_left_down(event)

    def on_middle_down(self, event):
        self.clicked_pos = event.GetPosition()
        for layer in self.layers:
            layer.on_middle_down(event)

    def on_right_down(self, event):
        self.clicked_pos = event.GetPosition() 
        for layer in self.layers:
            layer.on_right_down(event)
                
    def on_mouse_move(self, event):
        for layer in self.layers:
            layer.on_mouse_move(event)

    def on_mousewheel(self, event):
        for layer in self.layers:
            layer.on_mousewheel(event)

    #

    def on_close(self, event):
        for layer in self.layers:
            layer.on_close(event)
        event.Skip()
