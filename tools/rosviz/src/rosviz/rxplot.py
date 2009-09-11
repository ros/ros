#!/usr/bin/env python
#
# Based on public domain code from Eli Bendersky (eliben@gmail.com)
# http://eli.thegreenplace.net/2008/08/01/matplotlib-with-wxpython-guis/
# 
# Additions are covered by the license below:
#
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

import roslib;roslib.load_manifest('rosviz')
import roslib.packages

import os
import string
import sys
import wx
import threading

import rosviz.rosutil
import rospy

import rosviz.vizutil
rosviz.vizutil.check_matplotlib_deps()

## MPL imports

import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigCanvas, \
    NavigationToolbar2WxAgg as NavigationToolbar

import numpy as np
import pylab

## Plotting colors. We artificially constrain the number of allowed plots by the number of
## COLORS we specify. rxplot has not been stress-tested yet
COLORS = (1, 0, 0), (0, 0, 1), (0, 1, 0), (1, 0, 1), (0, 1, 1), (0.5, 0.24, 0), (0, 0.5, 0.24), (1, 0.5, 0), 

## Subscriber to ROS topic that buffers incoming data
class ROSData(object):
    
    def __init__(self, topic, start_time):
        self.name = topic
        self.start_time = start_time
        self.error = None
        
        self.lock = threading.Lock()
        self.buff_x = []
        self.buff_y = []
        
        topic_type, real_topic, fields = rosviz.rosutil.get_topic_type(topic)
        self.field_evals = rosviz.rosutil.generate_field_evals(fields)
        data_class = roslib.scriptutil.get_message_class(topic_type)
        self.sub = rospy.Subscriber(real_topic, data_class, self._ros_cb)

    def close(self):
      self.sub.unregister()

    ## ROS subscriber callback
    ## @param self
    ## @param msg        
    def _ros_cb(self, msg):
        try:
            self.lock.acquire()
            try:
                self.buff_y.append(self._get_data(msg))
                # #944: use message header time if present
                if msg.__class__._has_header:
                    self.buff_x.append(msg.header.stamp.to_seconds() - self.start_time)
                else:
                    self.buff_x.append(rospy.get_time() - self.start_time)                    
                #self.axes[index].plot(datax, buff_y)
            except AttributeError, e:
                self.error = Exception("Invalid topic spec [%s]: %s"%(self.name, str(e)))
        finally:
            self.lock.release()
        
    ## Get the next data in the series
    ## @param self
    ## @return [xdata], [ydata]
    def next(self):
        if self.error:
            raise self.error
        try:
            self.lock.acquire()
            buff_x = self.buff_x
            buff_y = self.buff_y
            self.buff_x = []
            self.buff_y = []
        finally:
            self.lock.release()
        return buff_x, buff_y
        
    def _get_data(self, msg):
        val = msg
        try:
            if not self.field_evals:
                return float(val)
            for f in self.field_evals:
                val = f(val)
            return float(val)
        except TypeError:
            print "[%s] value was not numeric: %s"%(self.name, val)
            sys.exit(1)

_paused = False
def toggle_ros_pause():
    global _paused
    _paused = not _paused
    
_stopped = False
def set_ros_stop():
    global _stopped, _paused
    _stopped = True
    _paused = True
        
# TODO: rewrite so that pause button toggles activation of other buttons
## Toolbar controls for rxplot
class RxPlotToolbar(NavigationToolbar):
    ON_PAUSE = wx.NewId()
    ON_STOP = wx.NewId()
    def __init__(self, canvas):
        NavigationToolbar.__init__(self, canvas)
        
        self.play_bm = wx.Bitmap(roslib.packages.get_pkg_dir('rosviz') + '/icons/play-16.png')
        self.pause_bm = wx.Bitmap(roslib.packages.get_pkg_dir('rosviz') + '/icons/pause-16.png')
        self.AddSimpleTool(self.ON_PAUSE, self.pause_bm, 'Pause', 'Activate pause')
        wx.EVT_TOOL(self, self.ON_PAUSE, self._on_pause)

        self.AddSimpleTool(self.ON_STOP, wx.Bitmap(roslib.packages.get_pkg_dir('rosviz') + '/icons/stop-16.png'), 'Stop', 'Activate stop')

        wx.EVT_TOOL(self, self.ON_STOP, self._on_stop)
        self._set_tb_enable()

    ## Most toolbar buttons remain disabled until rxplot enters paused or stopped state.
    def _set_tb_enable(self):
        # parent toolbar doesn't expose home, save, or subplot
        # buttons. However, these don't behave as badly under pause so
        # it's not as important to disable those.
        tb_enabled = _paused or _stopped
        for b in [self._NTB2_BACK, self._NTB2_FORWARD, self._NTB2_PAN, self._NTB2_ZOOM]:
            self.EnableTool(b, tb_enabled)

    def _on_pause(self, evt):
        toggle_ros_pause()
        self._set_tb_enable()
        if _paused:
            self.SetToolNormalBitmap(self.ON_PAUSE, self.play_bm)
        else:
            self.SetToolNormalBitmap(self.ON_PAUSE, self.pause_bm)
        # cancel state of pan/zoom
        self.ToggleTool(self._NTB2_PAN, False)
        self.ToggleTool(self._NTB2_ZOOM, False)
        self._active = 'ZOOM'
        self.zoom(tuple())

    def _on_stop(self, evt):
        set_ros_stop()
        self._set_tb_enable()
        # kill pause/stop buttons as well
        self.EnableTool(self.ON_PAUSE, False)
        self.EnableTool(self.ON_STOP, False)        
            
## WX Frame containing plots
class RxPlotFrame(wx.Frame):
    def __init__(self, topics, options):
        self.title = options.title
        wx.Frame.__init__(self, None, -1, self.title)

        # There are two different indicies for our list data types based on the fact that topics is
        # a list[list(str)]. 
        #
        # axes_index: first index into topics
        #
        # plot_index: index based on flattened topic list, i.e. absolute topic number
        #
        # e.g. for self.topics = [ ['/p0/f0'], ['/p1/f1', '/p2/f2']]:
        #  * axes_index 0 indexes ['/p0/f0'], 
        #  * axes_index 1 indexes ['/p1/f1', '/p2/f2'],
        #  * plot_index 0 indexes '/p0/f0'
        #  * plot_index 1 indexes '/p1/f1'
        #  * plot_index 2 indexes '/p2/f2'
        
        # self.axes and self.topics uses axes_index
        self.axes = []
        self.topics = topics
        self.period = options.period
        self.marker = options.marker
        
        # initialize data storage and sources
        # datagen, datax, and datay use plot_index.
        self.datagen = []
        self.datax = []
        self.datay = []        
        
        self.start_time = rospy.get_time()
        for topic_list in topics:
            for t in topic_list:
                dg = ROSData(t, self.start_time)
                self.datagen.append(dg)
                datax, datay = dg.next()
                self.datax.append(datax)
                self.datay.append(datay)
        
        self.create_menu()
        self.statusbar = self.CreateStatusBar()
        self.create_main_canvas()
        
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)        
        #self.redraw_timer.Start(100)
        self.redraw_timer.Start(500)        

    def create_menu(self):
        self.menubar = wx.MenuBar()
        
        menu_file = wx.Menu()
        m_expt = menu_file.Append(-1, "&Save plot\tCtrl-S", "Save plot to file")
        self.Bind(wx.EVT_MENU, self.on_save_plot, m_expt)
        menu_file.AppendSeparator()
        m_exit = menu_file.Append(-1, "E&xit\tCtrl-X", "Exit")
        self.Bind(wx.EVT_MENU, self.on_exit, m_exit)
                
        self.menubar.Append(menu_file, "&File")
        self.SetMenuBar(self.menubar)

    def onpick(self, event):
        thisline = event.artist
        xdata, ydata = thisline.get_data()
        ind = event.ind
        self.statusbar.SetStatusText("%s, %s" % (xdata[ind][len(xdata[ind])/2], ydata[ind][len(ydata[ind])/2]))

    def create_main_canvas(self):
        self.init_plot()
        self.canvas = FigCanvas(self, -1, self.fig)
        self.canvas.mpl_connect('pick_event', self.onpick)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.SetSizer(self.sizer)

        self.add_toolbar()        

        self.SetSize(wx.Size(500, 700))
        self.Layout()
    
    def init_plot(self):
        self.dpi = 100
        rc = matplotlib.figure.SubplotParams(left=0.125, bottom=0.12, right=0.99, top=0.99, wspace=0.001, hspace=0.1)
        self.fig = Figure((3.0, 3.0), dpi=self.dpi, subplotpars=rc)

        self.plot_data = []
        flat_topics = []

        axes_index = 0
        plot_index = 0
        for topic_list in self.topics:
            axes = self.fig.add_subplot(string.atoi('%d1%d'%(len(self.topics), axes_index+1))) 
            axes.set_axis_bgcolor('white')            
            #axes.set_title(', '.join(topic_list), size=8)
            pylab.setp(axes.get_xticklabels(), fontsize=6)
            pylab.setp(axes.get_yticklabels(), fontsize=8)
            # equiv to self.axes[axes_index] = axes
            self.axes.append(axes)
            axes_index += 1

            for topic in topic_list:
                flat_topics.append(topic)
                plot_data = \
                          axes.plot(self.datax[plot_index],
                                    self.datay[plot_index],
                                    marker=self.marker,
                                    linewidth=1,
                                    picker=5,
                                    color=COLORS[plot_index],
                                    )[0]
                # equiv to self.plot_data[plot_index] = plot_data
                self.plot_data.append(plot_data)
                plot_index += 1

        for ax in self.axes:
            ax.grid(True, color='gray')
            pylab.setp(ax.get_xticklabels(), visible=True)

        fp = matplotlib.font_manager.FontProperties(size=8)
        if len(flat_topics) > 1:
            self.fig.legend(self.plot_data, flat_topics, 'lower right', prop=fp)

    def draw_plot(self, relimit=False):
        if not self.plot_data:
            return
        
        if relimit and self.datax[0]:
            axes_index = 0
            plot_index = 0
            # axes are indexed by topic_list, plots are indexed by topic number
            for topic_list in self.topics:
                axes = self.axes[axes_index] 
                axes_index += 1
                ymin = ymax = None
                for t in topic_list:
                    datax = self.datax[plot_index]
                    datay = self.datay[plot_index]
                    plot_index += 1

                    xmax = datax[-1]
                    xmin = xmax - self.period
                    if ymin is None:
                        ymin = min(datay)
                        ymax = max(datay)
                    else:
                        ymin = min(min(datay), ymin)
                        ymax = max(max(datay), ymax)

                    # pad the min/max
                    delta = ymax - ymin
                    ymin -= .05*delta
                    ymax += .05*delta
                    
                    axes.set_xbound(lower=xmin, upper=xmax)
                    axes.set_ybound(lower=ymin, upper=ymax)

        for plot_index in xrange(0, len(self.plot_data)):
            datax = self.datax[plot_index]
            datay = self.datay[plot_index]

            plot_data = self.plot_data[plot_index]
            plot_data.set_data(np.array(datax), np.array(datay))

        self.canvas.draw()
    
    def on_save_plot(self, event):
        file_choices = "PNG (*.png)|*.png"
        
        dlg = wx.FileDialog(
            self, 
            message="Save plot as...",
            defaultDir=os.getcwd(),
            defaultFile="plot.png",
            wildcard=file_choices,
            style=wx.SAVE)
        
        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            self.canvas.print_figure(path, dpi=self.dpi)
            self.flash_status_message("Saved to %s" % path)
    
    def on_redraw_timer(self, event):
        # if paused do not add data, but still redraw the plot
        # (to respond to scale modifications, grid change, etc.)
        #
        if _stopped:
          for dg in self.datagen:
            dg.close()
          self.datagen = []

        if not _paused:
            for plot_index in xrange(0, len(self.datagen)):
                try:
                    datax, datay = self.datagen[plot_index].next()
                except Exception, e:
                    print >> sys.stderr, str(e)
                    wx.GetApp().Exit()
                self.datax[plot_index].extend(datax)
                self.datay[plot_index].extend(datay)
        
            self.draw_plot(relimit=True)
    
    def on_exit(self, event):
        self.Destroy()
    
    def flash_status_message(self, msg, flash_len_ms=1500):
        self.statusbar.SetStatusText(msg)
        self.timeroff = wx.Timer(self)
        self.Bind(
            wx.EVT_TIMER, 
            self.on_flash_status_off, 
            self.timeroff)
        self.timeroff.Start(flash_len_ms, oneShot=True)
    
    def on_flash_status_off(self, event):
        self.statusbar.SetStatusText('')

    def add_toolbar(self):
        self.toolbar = RxPlotToolbar(self.canvas)
        self.toolbar.Realize()
        self.SetToolBar(self.toolbar)
        if 0:
            if wx.Platform == '__WXMAC__':
                # Mac platform (OSX 10.3, MacPython) does not seem to cope with
                # having a toolbar in a sizer. This work-around gets the buttons
                # back, but at the expense of having the toolbar at the top
                self.SetToolBar(self.toolbar)
            else:
                # On Windows platform, default window size is incorrect, so set
                # toolbar width to figure width.
                tw, th = self.toolbar.GetSizeTuple()
                fw, fh = self.canvas.GetSizeTuple()
                # By adding toolbar in sizer, we are able to put it at the bottom
                # of the frame - so appearance is closer to GTK version.
                # As noted above, doesn't work for Mac.
                self.toolbar.SetSize(wx.Size(fw, th))
                self.sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
        # update the axes menu on the toolbar
        self.toolbar.update()

# TODO: poll for rospy.is_shutdown()
def rxplot_main():
    from optparse import OptionParser
    parser = OptionParser(usage="usage: rxplot [options] /topic/field1 [/topic/field2]")
    parser.add_option("-p", "--period", type="int",
                      dest="period", default=5,
                      help="set period displayed in window")
    parser.add_option("-m", "--marker", type="string",
                      dest="marker", default='None',
                      help="set the line marker, e.g. o, ^, .")
    parser.add_option("-t", "--title", type="string",
                      dest="title", default='RXPlot',
                      help="set the title")
    options, topics = parser.parse_args()
    
    topic_list = []
    for t in topics:
        # c_topics is the list of topics to plot together
        c_topics = []
        # compute combined topic list, t == '/foo/bar1,/baz/bar2'
        for sub_t in [x for x in t.split(',') if x]:
            # check for shorthand '/foo/field1:field2:field3'
            if ':' in sub_t:
                base = sub_t[:sub_t.find(':')]
                # the first prefix includes a field name, so save then strip it off
                c_topics.append(base)
                if not '/' in base:
                    parser.error("%s must contain a topic and field name"%sub_t)
                base = base[:base.rfind('/')]

                # compute the rest of the field names
                fields = sub_t.split(':')[1:]
                c_topics.extend(["%s/%s"%(base, f) for f in fields if f])
            else:
                c_topics.append(sub_t)

        # #1053: resolve command-line topic names
        c_topics = [roslib.scriptutil.script_resolve_name('rxplot', n) for n in c_topics]

        topic_list.append(c_topics)

    #flatten for printing
    print_topic_list = []
    for l in topic_list:
        if type(l) == list:
            print_topic_list.extend(l)
        else:
            print_topic_list.append(l)

    # constrain the number of things we plot by len(COLORS). this is an artificial
    # performance hack.
    if len(print_topic_list) > len(COLORS):
        parser.error("maximum number of topics that can be plotted is %d"%len(COLORS))
    print "plotting topics", ', '.join(print_topic_list)

    rospy.init_node('rxplot', anonymous=True)

    app = wx.PySimpleApp()    
    app.frame = RxPlotFrame(topic_list, options)
    app.frame.Show()
    app.MainLoop()
    
    rospy.signal_shutdown('GUI shutdown')    

if __name__ == '__main__':
    rxplot_main()
    



