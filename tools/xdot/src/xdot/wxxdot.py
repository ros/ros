#!/usr/bin/env python
#
# wxpython widgets for using Jose Fonseca's cairo graphviz visualizer
# Copyright (c) 2010, Willow Garage, Inc.
# 
# Source modified from Jose Fonseca's XDot pgtk widgets. That code is
# Copyright 2008 Jose Fonseca
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from xdot import *

__all__ = ['WxDotWindow', 'WxDotFrame']

# We need to get the wx version with built-in cairo support
import wxversion
wxversion.select("2.8")
import wx
import wx.lib.wxcairo as wxcairo

# This is a crazy hack to get this to work on 64-bit systems
if 'wxMac' in wx.PlatformInfo:
  pass # Implement if necessary
elif 'wxMSW' in wx.PlatformInfo:
  pass # Implement if necessary
elif 'wxGTK' in wx.PlatformInfo:
  import ctypes
  gdkLib = wx.lib.wxcairo._findGDKLib()
  gdkLib.gdk_cairo_create.restype = ctypes.c_void_p

class WxDragAction(object):
  def __init__(self, dot_widget):
    self.dot_widget = dot_widget

  def on_button_press(self, event):
    x,y = event.GetPositionTuple()
    self.startmousex = self.prevmousex = x
    self.startmousey = self.prevmousey = y
    self.start()

  def on_motion_notify(self, event):
    x,y = event.GetPositionTuple()
    deltax = self.prevmousex - x
    deltay = self.prevmousey - y
    self.drag(deltax, deltay)
    self.prevmousex = x
    self.prevmousey = y

  def on_button_release(self, event):
    x,y = event.GetPositionTuple()
    self.stopmousex = x
    self.stopmousey = y
    self.stop()

  def draw(self, cr):
    pass

  def start(self):
    pass

  def drag(self, deltax, deltay):
    pass

  def stop(self):
    pass

  def abort(self):
    pass

class WxNullAction(WxDragAction):
  def on_motion_notify(self, event):
    pass

class WxPanAction(WxDragAction):
  def start(self):
    self.dot_widget.set_cursor(wx.CURSOR_SIZING)

  def drag(self, deltax, deltay):
    self.dot_widget.x += deltax / self.dot_widget.zoom_ratio
    self.dot_widget.y += deltay / self.dot_widget.zoom_ratio
    self.dot_widget.Refresh()

  def stop(self):
    self.dot_widget.set_cursor(wx.CURSOR_ARROW)

  abort = stop

class WxZoomAction(WxDragAction):
  def drag(self, deltax, deltay):
    self.dot_widget.zoom_ratio *= 1.005 ** (deltax + deltay)
    self.dot_widget.zoom_to_fit_on_resize = False
    self.dot_widget.Refresh()

  def stop(self):
    self.dot_widget.Refresh()

class WxZoomAreaAction(WxDragAction):
  def drag(self, deltax, deltay):
    self.dot_widget.Refresh()

  def draw(self, cr):
    cr.save()
    cr.set_source_rgba(.5, .5, 1.0, 0.25)
    cr.rectangle(self.startmousex, self.startmousey,
           self.prevmousex - self.startmousex,
           self.prevmousey - self.startmousey)
    cr.fill()
    cr.set_source_rgba(.5, .5, 1.0, 1.0)
    cr.set_line_width(1)
    cr.rectangle(self.startmousex - .5, self.startmousey - .5,
           self.prevmousex - self.startmousex + 1,
           self.prevmousey - self.startmousey + 1)
    cr.stroke()
    cr.restore()

  def stop(self):
    x1, y1 = self.dot_widget.window2graph(self.startmousex,
                        self.startmousey)
    x2, y2 = self.dot_widget.window2graph(self.stopmousex,
                        self.stopmousey)
    self.dot_widget.zoom_to_area(x1, y1, x2, y2)

  def abort(self):
    self.dot_widget.Refresh()

class WxDotWindow(wx.Panel):
  """wxpython Frame that draws dot graphs."""
  filter = 'dot'

  def __init__(self, parent, id):
    """constructor"""
    wx.Panel.__init__(self, parent, id)

    self.graph = Graph()
    self.openfilename = None

    self.x, self.y = 0.0, 0.0
    self.zoom_ratio = 1.0
    self.zoom_to_fit_on_resize = False
    self.animation = NoAnimation(self)
    self.drag_action = WxNullAction(self)
    self.presstime = None
    self.highlight = None
    
    # Bind events
    self.Bind(wx.EVT_PAINT, self.OnPaint)
    self.Bind(wx.EVT_SIZE, self.OnResize)

    self.Bind(wx.EVT_MOUSEWHEEL, self.OnScroll)

    self.Bind(wx.EVT_MOUSE_EVENTS, self.OnMouse)

    self.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)

    # Callback register
    self.select_cbs = []
    self.dc = None
    self.ctx = None
    self.items_by_url = {}

  ### User callbacks
  def register_select_callback(self, cb):
    self.select_cbs.append(cb)

  ### Event handlers
  def OnResize(self, event):
    self.Refresh()

  def OnPaint(self, event):
    """Redraw the graph."""
    dc = wx.PaintDC(self)

    #print dc
    ctx = wxcairo.ContextFromDC(dc)
    ctx = pangocairo.CairoContext(ctx)
    #print "DRAW"

    # Get widget size
    width, height = self.GetSize()
    #width,height = self.dc.GetSizeTuple()

    ctx.rectangle(0,0,width,height)
    ctx.clip()

    ctx.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    ctx.paint()

    ctx.save()
    ctx.translate(0.5*width, 0.5*height)

    ctx.scale(self.zoom_ratio, self.zoom_ratio)
    ctx.translate(-self.x, -self.y)
    self.graph.draw(ctx, highlight_items=self.highlight)
    ctx.restore()

    self.drag_action.draw(ctx)
    
  def OnScroll(self, event):
    """Zoom the view."""
    if event.GetWheelRotation() > 0:
      self.zoom_image(self.zoom_ratio * self.ZOOM_INCREMENT,
          pos=(event.GetX(), event.GetY()))
    else:
      self.zoom_image(self.zoom_ratio / self.ZOOM_INCREMENT,
          pos=(event.GetX(), event.GetY()))

  def OnKeyDown(self, event):
    """Process key down event."""
    key = event.GetKeyCode()
    if key == wx.WXK_LEFT:
      self.x -= self.POS_INCREMENT/self.zoom_ratio
      self.Refresh()
    if key == wx.WXK_RIGHT:
      self.x += self.POS_INCREMENT/self.zoom_ratio
      self.Refresh()
    if key == wx.WXK_UP:
      self.y -= self.POS_INCREMENT/self.zoom_ratio
      self.Refresh()
    if key == wx.WXK_DOWN:
      self.y += self.POS_INCREMENT/self.zoom_ratio
      self.Refresh()
    if key == wx.WXK_PAGEUP:
      self.zoom_image(self.zoom_ratio * self.ZOOM_INCREMENT)
      self.Refresh()
    if key == wx.WXK_PAGEDOWN:
      self.zoom_image(self.zoom_ratio / self.ZOOM_INCREMENT)
      self.Refresh()
    if key == wx.WXK_ESCAPE:
      self.drag_action.abort()
      self.drag_action = WxNullAction(self)
    if key == ord('F'):
      self.zoom_to_fit()
    if key == ord('R'):
      self.reload()
    if key == ord('Q'):
      self.reload()
      exit(0)

  ### Helper functions
  def get_current_pos(self):
    """Get the current graph position."""
    return self.x, self.y

  def set_current_pos(self, x, y):
    """Set the current graph position."""
    self.x = x
    self.y = y
    self.Refresh()

  def set_highlight(self, items):
    """Set a number of items to be hilighted."""
    if self.highlight != items:
      self.highlight = items
      self.Refresh()

  ### Cursor manipulation
  def set_cursor(self, cursor_type):
    self.cursor = wx.StockCursor(cursor_type)
    self.SetCursor(self.cursor)

  ### Zooming methods
  def zoom_image(self, zoom_ratio, center=False, pos=None):
    """Zoom the graph."""
    if center:
      self.x = self.graph.width/2
      self.y = self.graph.height/2
    elif pos is not None:
      width, height = self.GetSize()
      x, y = pos
      x -= 0.5*width
      y -= 0.5*height
      self.x += x / self.zoom_ratio - x / zoom_ratio
      self.y += y / self.zoom_ratio - y / zoom_ratio
    self.zoom_ratio = zoom_ratio
    self.zoom_to_fit_on_resize = False
    self.Refresh()

  def zoom_to_area(self, x1, y1, x2, y2):
    """Zoom to an area of the graph."""
    width, height = self.GetSize()
    area_width = abs(x1 - x2)
    area_height = abs(y1 - y2)
    self.zoom_ratio = min(
      float(width)/float(area_width),
      float(height)/float(area_height)
    )
    self.zoom_to_fit_on_resize = False
    self.x = (x1 + x2) / 2
    self.y = (y1 + y2) / 2
    self.Refresh()

  def zoom_to_fit(self):
    """Zoom to fit the size of the graph."""
    width,height = self.GetSize()
    x = self.ZOOM_TO_FIT_MARGIN
    y = self.ZOOM_TO_FIT_MARGIN
    width -= 2 * self.ZOOM_TO_FIT_MARGIN
    height -= 2 * self.ZOOM_TO_FIT_MARGIN

    if float(self.graph.width) > 0 and float(self.graph.height) > 0 and width > 0 and height > 0: 
      zoom_ratio = min(
        float(width)/float(self.graph.width),
        float(height)/float(self.graph.height)
      )
      self.zoom_image(zoom_ratio, center=True)
      self.zoom_to_fit_on_resize = True

  ZOOM_INCREMENT = 1.25
  ZOOM_TO_FIT_MARGIN = 12

  def on_zoom_in(self, action):
    self.zoom_image(self.zoom_ratio * self.ZOOM_INCREMENT)

  def on_zoom_out(self, action):
    self.zoom_image(self.zoom_ratio / self.ZOOM_INCREMENT)

  def on_zoom_fit(self, action):
    self.zoom_to_fit()

  def on_zoom_100(self, action):
    self.zoom_image(1.0)

  POS_INCREMENT = 100

  def get_drag_action(self, event):
    """Get a drag action for this click."""
    # Grab the button
    button = event.GetButton()
    # Grab modifier keys
    control_down  = event.ControlDown()
    alt_down = event.AltDown()
    shift_down = event.ShiftDown()

    drag = event.Dragging()
    motion = event.Moving()

    # Get the correct drag action for this click
    if button in (wx.MOUSE_BTN_LEFT, wx.MOUSE_BTN_MIDDLE): # left or middle button
      if control_down:
        if shift_down:
          return WxZoomAreaAction(self)
        else:
          return WxZoomAction(self)
      else:
        return WxPanAction(self)

    return WxNullAction(self)

  def OnMouse(self, event):
    x,y = event.GetPositionTuple()

    item = None

    # Get the item
    if not event.Dragging():
      item = self.get_url(x, y)
      if item is None:
        item = self.get_jump(x, y)

      if item is not None:
        self.set_cursor(wx.CURSOR_HAND)
        self.set_highlight(item.highlight)

        for cb in self.select_cbs:
          cb(item,event)
      else:
        self.set_cursor(wx.CURSOR_ARROW)
        self.set_highlight(None)

    if item is None:
      if event.ButtonDown():
        self.animation.stop()
        self.drag_action.abort()

        # Get the drag action
        self.drag_action = self.get_drag_action(event)
        self.drag_action.on_button_press(event)

        self.pressx = x
        self.pressy = y

      if event.Dragging() or event.Moving():
        self.drag_action.on_motion_notify(event)

      if event.ButtonUp():
        self.drag_action.on_button_release(event)
        self.drag_action = WxNullAction(self)

    event.Skip()
    

  def on_area_size_allocate(self, area, allocation):
    if self.zoom_to_fit_on_resize:
      self.zoom_to_fit()

  def animate_to(self, x, y):
    self.animation = ZoomToAnimation(self, x, y)
    self.animation.start()

  def window2graph(self, x, y):
    "Get the x,y coordinates in the graph from the x,y coordinates in the window."""
    width, height = self.GetSize()
    x -= 0.5*width
    y -= 0.5*height
    x /= self.zoom_ratio
    y /= self.zoom_ratio
    x += self.x
    y += self.y
    return x, y

  def get_url(self, x, y):
    x, y = self.window2graph(x, y)
    return self.graph.get_url(x, y)

  def get_jump(self, x, y):
    x, y = self.window2graph(x, y)
    return self.graph.get_jump(x, y)

  def set_filter(self, filter):
    self.filter = filter

  def set_dotcode(self, dotcode, filename='<stdin>'):
    if isinstance(dotcode, unicode):
      dotcode = dotcode.encode('utf8')
    p = subprocess.Popen(
      [self.filter, '-Txdot'],
      stdin=subprocess.PIPE,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      shell=False,
      universal_newlines=True
    )
    xdotcode, error = p.communicate(dotcode)
    if p.returncode != 0:
      print "ERROR PARSING DOT CODE", error
      dialog = gtk.MessageDialog(type=gtk.MESSAGE_ERROR,
                     message_format=error,
                     buttons=gtk.BUTTONS_OK)
      dialog.set_title('Dot Viewer')
      dialog.run()
      dialog.destroy()
      return False
    try:
      self.set_xdotcode(xdotcode)

      # Store references to all the items 
      self.items_by_url = {}
      for item in self.graph.nodes + self.graph.edges:
        if item.url is not None:
          self.items_by_url[item.url] = item

      # Store references to subgraph states
      self.subgraph_shapes = self.graph.subgraph_shapes

    except ParseError, ex:
      print "ERROR PARSING XDOT CODE"
      dialog = gtk.MessageDialog(type=gtk.MESSAGE_ERROR,
                     message_format=str(ex),
                     buttons=gtk.BUTTONS_OK)
      dialog.set_title('Dot Viewer')
      dialog.run()
      dialog.destroy()
      return False
    else:
      self.openfilename = filename
      return True

  def set_xdotcode(self, xdotcode):
    """Set xdot code."""
    #print xdotcode
    parser = XDotParser(xdotcode)
    self.graph = parser.parse()
    self.highlight = None
    #self.zoom_image(self.zoom_ratio, center=True)

  def reload(self):
    if self.openfilename is not None:
      try:
        fp = file(self.openfilename, 'rt')
        self.set_dotcode(fp.read(), self.openfilename)
        fp.close()
      except IOError:
        pass


class WxDotFrame(wx.Frame):
  def __init__(self):
    wx.Frame.__init__(self, None, -1, "Dot Viewer", size=(512,512))
    
    vbox = wx.BoxSizer(wx.VERTICAL)

    # Construct toolbar
    toolbar = wx.ToolBar(self, -1)
    toolbar.AddLabelTool(wx.ID_OPEN, 'Open File',
        wx.ArtProvider.GetBitmap(wx.ART_FOLDER_OPEN,wx.ART_OTHER,(16,16)))
    toolbar.AddLabelTool(wx.ID_HELP, 'Help',
        wx.ArtProvider.GetBitmap(wx.ART_HELP,wx.ART_OTHER,(16,16)) )
    toolbar.Realize()

    self.Bind(wx.EVT_TOOL, self.DoOpenFile, id=wx.ID_OPEN)
    self.Bind(wx.EVT_TOOL, self.ShowControlsDialog, id=wx.ID_HELP)

    # Create dot widge
    self.widget = WxDotWindow(self, -1)

    # Add elements to sizer
    vbox.Add(toolbar, 0, wx.EXPAND)
    vbox.Add(self.widget, 100, wx.EXPAND | wx.ALL)

    self.SetSizer(vbox)
    self.Center()

  def ShowControlsDialog(self,event):
    dial = wx.MessageDialog(None,
        "\
Pan: Arrow Keys\n\
Zoom: PageUp / PageDown\n\
Zoom To Fit: F\n\
Refresh: R",
        'Keyboard Controls', wx.OK)
    dial.ShowModal()

  def DoOpenFile(self,event):
    wcd = 'All files (*)|*|GraphViz Dot Files(*.dot)|*.dot|'
    dir = os.getcwd()
    open_dlg = wx.FileDialog(self, message='Choose a file', defaultDir=dir, defaultFile='', 
    wildcard=wcd, style=wx.OPEN|wx.CHANGE_DIR)
    if open_dlg.ShowModal() == wx.ID_OK:
      path = open_dlg.GetPath()

      try:
        self.open_file(path)

      except IOError, error:
        dlg = wx.MessageDialog(self, 'Error opening file\n' + str(error))
        dlg.ShowModal()

      except UnicodeDecodeError, error:
        dlg = wx.MessageDialog(self, 'Error opening file\n' + str(error))
        dlg.ShowModal()

    open_dlg.Destroy()

  def OnExit(self, event):
    pass

  def set_dotcode(self, dotcode, filename='<stdin>'):
    if self.widget.set_dotcode(dotcode, filename):
      self.SetTitle(os.path.basename(filename) + ' - Dot Viewer')
      self.widget.zoom_to_fit()

  def set_xdotcode(self, xdotcode, filename='<stdin>'):
    if self.widget.set_xdotcode(xdotcode):
      self.SetTitle(os.path.basename(filename) + ' - Dot Viewer')
      self.widget.zoom_to_fit()

  def open_file(self, filename):
    try:
      fp = file(filename, 'rt')
      self.set_dotcode(fp.read(), filename)
      fp.close()
    except IOError, ex:
      """
      dlg = gtk.MessageDialog(type=gtk.MESSAGE_ERROR,
                  message_format=str(ex),
                  buttons=gtk.BUTTONS_OK)
      dlg.set_title('Dot Viewer')
      dlg.run()
      dlg.destroy()
      """

  def set_filter(self, filter):
    self.widget.set_filter(filter)

