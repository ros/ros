#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

"""
An example of how to use wx or wxagg in an application with the new
toolbar - comment out the setA_toolbar line for no toolbar
"""

import roslib; roslib.load_manifest('rosmultiplot')

import sys, traceback, logging, time, itertools

import chartdata
import rosdata

from numpy import arange, sin, pi
import matplotlib
matplotlib.use('WXAgg')
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.backends.backend_wx import NavigationToolbar2Wx

from matplotlib.numerix import sin, cos, exp, pi, arange
from pylab import figure, show, setp

import wx

class CanvasFrame(wx.Frame):

    def __init__(self):
        wx.Frame.__init__(self,None,-1,
                          'CanvasFrame',size=(550,350))
        self.SetBackgroundColour(wx.NamedColor("WHITE"))
        self.figure = fig = figure(frameon=True, facecolor="white")
        self.setup_plot(fig)
        self.canvas = FigureCanvas(self, -1, fig)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1, wx.LEFT | wx.TOP | wx.GROW)
        self.SetSizer(self.sizer)
        self.Fit()

        #self.add_toolbar()  # comment this out for no toolbar

    def update(self):
        self.canvas.draw()

    def OnPaint(self, event):
        self.canvas.draw()

    def setup_plot(self, fig):        
        yprops = dict(rotation=0,
                      horizontalalignment='right',
                      verticalalignment='center',
                      x=-0.01)

        axprops = { 'picker': True, 'alpha': 0.7 }
        colors = ['red', 'blue', 'green', 'orange', 'purple']
        ax = fig.add_subplot(111) #212 will be topics graph
        labels = []
        xmax   = 0
        #TODO: get rid of this 
        if 1:
            xmax   = 800
            ymax   = 20
            labels = ['/axiscam', '/foo/bar']
            xdata  = [ [ (0, 500), (500, 200),  (700, 100)], [ (0, 410), (410, 20), (430, 10), ] ]
            ycurr  = 19
            labels = [x for x in reversed(labels)]
            for x in xdata:
                barcol = colors[:len(xdata)+1]
                print barcol
                ax.broken_barh(x , (ycurr, 1), facecolors=barcol, edgecolors=barcol, **axprops)
                ycurr -= 1
            
        ax.set_ylim(0,ymax)
        ax.set_xlim(0,xmax)
        ax.set_xlabel('bandwidth')

        self._updatey(ax, labels, ymax)

    ## update y-axis labelling and tickmarks basedon number of labels
    def _updatey(self, ax, labels, ymax):
        #TODO: resizing of ymax if num nodes grows
        
        # generate tick indicies -- center on half marks
        ticks = [float(x) + 0.5 for x in range(0, ymax)]
        ax.set_yticks(ticks)
        # pad labels
        labels = [x for x in itertools.repeat('', len(ticks) - len(labels))] + labels
        ax.set_yticklabels(labels)
        
        setp(ax.get_xticklines(), visible=False)    
        setp(ax.get_yticklines(), visible=False)    
        
class App(wx.App):

    def __init__(self, arg):
        wx.App.__init__(self, arg)
        
    def OnInit(self):
        self.frame = CanvasFrame()
        #wx.EVT_CLOSE(self.frame, self.on_close)

        self.timer = wx.Timer(self, id=1)
        self.timer.Start(100) #10Hz
        self.Bind(wx.EVT_TIMER, rosdata.update, id=1)

        self.frame.Show(True)
        return True

    def update(self):
        if self.frame:
            self.frame.update()

class RVException(Exception): pass

def rosvizmain():
    try:
        app = App(0)
        rosdata.init()
        import thread
        thread.start_new_thread(rosdata.rosloop, ())
        app.MainLoop()
    except RVException, e:
        print >> sys.stdrr, str(e)

if __name__ == '__main__':
    rosvizmain()
