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
from matplotlib.backends.backend_wx import _load_bitmap

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
        rospy.Subscriber(real_topic, data_class, self._ros_cb)

    ## ROS subscriber callback
    ## @param self
    ## @param msg        
    def _ros_cb(self, msg):
        try:
            self.lock.acquire()
            try:
                self.buff_y.append(self._get_data(msg))
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
    
        
# TODO: rewrite so that pause button toggles activation of other buttons
## Toolbar controls for rxplot
class RxPlotToolbar(NavigationToolbar):
    ON_PAUSE = wx.NewId()
    def __init__(self, canvas):
        NavigationToolbar.__init__(self, canvas)
        
        # TODO: load actual pause button
        self.AddSimpleTool(self.ON_PAUSE, _load_bitmap('stock_up.xpm'),
                           'Pause', 'Activate pause')
        wx.EVT_TOOL(self, self.ON_PAUSE, self._on_pause)

    def _on_pause(self, evt):
        toggle_ros_pause()

## WX Frame containing plots
class RxPlotFrame(wx.Frame):
    title = 'RXPlot'
    
    def __init__(self, topics, combined=False):
        wx.Frame.__init__(self, None, -1, self.title)

        self.topics = topics
        self.combined = combined        
        
        # initialize data storage and sources
        self.datagen = []
        self.datax = []
        self.datay = []        

        self.start_time = rospy.get_time()
        for t in topics:
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

    def create_main_canvas(self):
        self.init_plot()
        self.canvas = FigCanvas(self, -1, self.fig)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.SetSizer(self.sizer)

        self.add_toolbar()        

        self.SetSize(wx.Size(500, 600))

        self.Layout()
    
    def init_plot(self):
        self.dpi = 100
        rc = matplotlib.figure.SubplotParams(left=0.125, bottom=0.12, right=0.99, top=0.99, wspace=0.001, hspace=0.1)
        self.fig = Figure((3.0, 3.0), dpi=self.dpi, subplotpars=rc)

        self.plot_data = []
        if self.combined:
            axes = self.fig.add_subplot(111)
            axes.set_axis_bgcolor('white')            
            axes.set_title(', '.join(self.topics), size=8)
            pylab.setp(axes.get_xticklabels(), fontsize=6)
            pylab.setp(axes.get_yticklabels(), fontsize=8)

            self.axes = [ axes ]
        else:
            self.axes = []
            
        #TODO: legend
        for i in xrange(0, len(self.topics)):
            if not self.combined:
                axes = self.fig.add_subplot(string.atoi('%d1%d'%(len(self.topics), i+1)))
                axes.set_axis_bgcolor('white')
                #axes.set_title(self.topics[i], size=8)
                self.axes.append(axes)

                pylab.setp(axes.get_xticklabels(), fontsize=6)
                pylab.setp(axes.get_yticklabels(), fontsize=8)

            plot_data = \
                      axes.plot(self.datax[i],
                                self.datay[i], 
                                linewidth=1,
                                color=COLORS[i],
                                )[0]
            self.plot_data.append(plot_data)

        for ax in self.axes:
            ax.grid(True, color='gray')
            pylab.setp(ax.get_xticklabels(), visible=True)

        fp = matplotlib.font_manager.FontProperties(size=8)
        if len(self.topics) > 1:
            self.fig.legend(self.plot_data, self.topics, 'lower right', prop=fp)

    def draw_plot(self, relimit=False):
        if not self.plot_data:
            return
        
        if relimit and self.datax[0]:
            if self.combined:
                xmax = self.datax[0][-1]
                ymin = round(min(self.datay[0]), 0) - 1
                ymax = round(max(self.datay[0]), 0) + 1                
                for i in xrange(1, len(self.plot_data)):
                    xmax = max(xmax, self.datax[i][-1])
                    ymin = min(ymin, round(min(self.datay[i]), 0) - 1)
                    ymax = max(ymax, round(max(self.datay[i]), 0) + 1)

                xmin = xmax - 5 #5 seconds
                self.axes[0].set_xbound(lower=xmin, upper=xmax)
                self.axes[0].set_ybound(lower=ymin, upper=ymax)
            else:
                for i in xrange(0, len(self.plot_data)):
                    axes = self.axes[i]                
                    datax = self.datax[i]
                    datay = self.datay[i]

                    xmax = datax[-1]
                    xmin = xmax - 5 #5 seconds
                    ymin = round(min(datay), 0) - 1
                    ymax = round(max(datay), 0) + 1

                    axes.set_xbound(lower=xmin, upper=xmax)
                    axes.set_ybound(lower=ymin, upper=ymax)

        for i in xrange(0, len(self.plot_data)):
            datax = self.datax[i]
            datay = self.datay[i]

            plot_data = self.plot_data[i]
            plot_data.set_xdata(np.array(datax))
            plot_data.set_ydata(np.array(datay))

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
        if not _paused:
            for i in xrange(0, len(self.datagen)):
                try:
                    datax, datay = self.datagen[i].next()
                except Exception, e:
                    print >> sys.stderr, str(e)
                    wx.GetApp().Exit()
                self.datax[i].extend(datax)
                self.datay[i].extend(datay)
        
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
    parser.add_option("-c", "--combined",
                      dest="combined", default=None,action="store_true",
                      help="plot topics on combined plot")
    options, topics = parser.parse_args()
    
    # constrain the number of things we plot by len(COLORS). this is an artificial
    # performance hack.
    if len(topics) > len(COLORS):
        parser.error("maximum number of topics that can be plotted is %d"%len(COLORS))
    print "plotting topics", topics

    rospy.init_node('rxplot', anonymous=True)

    app = wx.PySimpleApp()    
    app.frame = RxPlotFrame(topics, combined=options.combined)
    app.frame.Show()
    app.MainLoop()
    
    rospy.signal_shutdown('GUI shutdown')    

if __name__ == '__main__':
    rxplot_main()
    



