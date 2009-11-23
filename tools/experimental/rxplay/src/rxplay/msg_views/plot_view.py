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

import collections
import string
import threading
import time

import rxtools.vizutil
rxtools.vizutil.check_matplotlib_deps()

import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigCanvas, NavigationToolbar2WxAgg as NavigationToolbar

import numpy
import pylab
import wx

from rxplay.bag_file import BagFile
from rxplay import msg_view

class PlotView(msg_view.TopicMsgView):
    name = 'Plot'

    def __init__(self, timeline, parent, title, x, y, width, height, max_repaint=0.2):
        msg_view.TopicMsgView.__init__(self, timeline, parent, title, x, y, width, height, max_repaint)

        # Input data
        self.series_list = []
        self.datax       = []
        self.datay       = []
        self.playhead    = 0

        # View parameters
        self.period = -1
        self.marker = 'o'
        self.legend = ''

        # Drawing objects
        self._axes          = None
        self._series_data   = None
        self._playhead_line = None

        # Create window, figure, canvas
        self._create_figure()

        self._init_plot([])
        
        self._data_thread = None

    def set_series_data(self, series, datax, datay):
        data = sorted([(datax[i], datay[i]) for i in range(len(datax))])

        self.datax[series] = [x for (x, y) in data]
        self.datay[series] = [y for (x, y) in data]

        self.invalidate()

    def set_playhead(self, playhead):
        self.playhead = playhead
        
        self.invalidate()

    def message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg):
        msg_view.TopicMsgView.message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg)

        if not self._data_thread:
            self._data_thread = PlotDataLoader(self, bag_file, bag_index, topic)
            self._data_thread.start()

        self.set_playhead(stamp.to_sec() - bag_index._data.get_start_stamp())
        
    def message_cleared(self):
        msg_view.TopicMsgView.message_cleared(self)

    def _create_figure(self):
        self._window = wx.Window(self.parent, -1, (self._x, self._y), (self._width, self._height))
        self._window.Show(False)

        self.dpi = 100
        figsize = (float(self.width) / self.dpi, float(self.height) / self.dpi)  # 1px border
        rc      = matplotlib.figure.SubplotParams(left=0.05, bottom=0.12, right=0.98, top=0.96, wspace=0.001, hspace=0.1)
        self.figure = Figure(figsize, dpi=self.dpi, subplotpars=rc, facecolor='w')

        self.canvas = FigCanvas(self._window, -1, self.figure)
        self.canvas.mpl_connect('pick_event', self.on_pick)

    def _init_plot(self, series_list):
        self.figure.clear()
        
        self.series_list = series_list

        self.series_data = []
        self.axes        = []
        flat_series = []

        for subplot_series in self.series_list:
            # Create axes for this plot
            axes = self.figure.add_subplot(len(self.series_list), 1, len(self.axes) + 1) 
            axes.set_axis_bgcolor('white')
            axes.grid(True, color='gray')
            pylab.setp(axes.get_xticklabels(), fontsize=9)
            pylab.setp(axes.get_yticklabels(), fontsize=9)
            self.axes.append(axes)

            # Create the series data
            for series in subplot_series:
                series_index = len(self.series_data)
                
                self.datax.append([])
                self.datay.append([])
                
                series_data = axes.plot(self.datax[series_index], self.datay[series_index], marker=self.marker, markersize=3, linewidth=1, picker=self.plot_picker)[0]

                self.series_data.append(series_data)

                flat_series.append(series)

        # Hide the x tick labels for every subplot except for the last
        for ax in self.axes:
            pylab.setp(ax.get_xticklabels(), visible=(ax == self.axes[-1]))

        # Set legend
        fp = matplotlib.font_manager.FontProperties(size=9)
        legends = self.legend.split(',') if self.legend else []
        if legends:
            if len(legends) != len(flat_series):
                raise "Number of legend strings does not match the number of series"
            self.figure.legend(self.series_data, legends,     'lower right', prop=fp)
        elif len(flat_series) > 1:
            self.figure.legend(self.series_data, flat_series, 'lower right', prop=fp)

    def paint(self, dc):
        dc.SetBrush(wx.WHITE_BRUSH)
        dc.SetPen(wx.BLACK_PEN)
        dc.DrawRectangle(0, 0, self.width, self.height)

        # Draw plot (not visible)
        self._draw_plot(True)
        
        dc.DrawBitmap(self.canvas.bitmap, 0, 0)

    def _draw_plot(self, relimit=False):
        if self.series_data and relimit and self.datax[0]:
            axes_index = 0
            plot_index = 0
            # axes are indexed by topic_list, plots are indexed by topic number
            for subplot_series in self.series_list:
                axes = self.axes[axes_index] 
                axes_index += 1
                
                xmin = xmax = None
                ymin = ymax = None
                for series in subplot_series:
                    datax = self.datax[plot_index]
                    datay = self.datay[plot_index]
                    plot_index += 1

                    if self.period < 0:
                        if xmin is None:
                            xmin = min(datax)
                            xmax = max(datax)
                        else:
                            xmin = min(min(datax), xmin)
                            xmax = max(max(datax), xmax)
                    else:
                        xmax = self.playhead + (self.period / 2)
                        xmin = xmax - self.period

                    if ymin is None:
                        ymin = min(datay)
                        ymax = max(datay)
                    else:
                        ymin = min(min(datay), ymin)
                        ymax = max(max(datay), ymax)

                    # pad the min/max
                    delta = ymax - ymin
                    ymin -= 0.1 * delta
                    ymax += 0.1 * delta
                    
                    axes.set_xbound(lower=xmin, upper=xmax)
                    axes.set_ybound(lower=ymin, upper=ymax)

        if self.series_data:
            for plot_index in xrange(0, len(self.series_data)):
                datax = self.datax[plot_index]
                datay = self.datay[plot_index]
    
                series_data = self.series_data[plot_index]
                series_data.set_data(numpy.array(datax), numpy.array(datay))

        self.canvas.draw()

    # Events

    def on_size(self, event):
        w, h = self.parent.GetClientSize()

        self._window.SetSize((w, h))
        self.figure.set_size_inches((float(w) / self.dpi, float(h) / self.dpi))
        self.resize(w, h)

    def on_right_down(self, event):
        self.clicked_pos = event.GetPosition()

        if self.contains(*self.clicked_pos):
            self.parent.PopupMenu(PlotPopupMenu(self.parent, self), self.clicked_pos)

    def on_close(self, event):
        if self._data_thread:
            self._data_thread.bag_index = None

    @staticmethod
    def plot_picker(artist, mouseevent):
        return True, {}

    def on_pick(self, event):
        print event.artist.get_xdata()

class PlotPopupMenu(wx.Menu):
    periods = [(   1, '1s'),
               (   5, '5s'),
               (  10, '10s'),
               (  30, '30s'),
               (  60, '1min'),
               ( 120, '2min'),
               ( 300, '5min'),
               ( 600, '10min'),
               (1800, '30min'),
               (3600, '1hr'),
               (  -1, 'All')]

    def __init__(self, parent, plot):
        wx.Menu.__init__(self)

        self.parent = parent

        for period, label in self.periods:
            period_item = self.PeriodMenuItem(self, wx.NewId(), label, period, plot)
            self.AppendItem(period_item)
            period_item.Check(plot.period == period_item.period)

    class PeriodMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, period, plot):
            wx.MenuItem.__init__(self, parent, id, label, kind=wx.ITEM_CHECK)
            
            self.period = period
            self.plot   = plot

            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())
    
        def on_menu(self, event):
            self.plot.period = self.period
            self.plot.force_repaint()

class PlotDataLoader(threading.Thread):
    def __init__(self, plot, bag_file, bag_index, topic):
        threading.Thread.__init__(self)
        
        self.setDaemon(True)

        self.plot      = plot
        self.bag_file  = bag_file
        self.bag_index = bag_index
        self.topic     = topic

        def load_odom_data(msg):
            return ([['translation.x',
                      'translation.y']],
                    [[(tf.header.stamp, tf.transform.translation.x) for tf in msg.transforms if tf.header.frame_id == '/odom_combined' and tf.child_frame_id == '/base_footprint'], 
                     [(tf.header.stamp, tf.transform.translation.y) for tf in msg.transforms if tf.header.frame_id == '/odom_combined' and tf.child_frame_id == '/base_footprint']])
    
        def load_joint_state_data(msg):
            return ([[n + ' pos' for n in msg.name], [n + ' vel' for n in msg.name], [n + ' effort' for n in msg.name]],
                    [[(msg.header.stamp, p)] for p in msg.position] + [[(msg.header.stamp, v)] for v in msg.velocity] + [[(msg.header.stamp, e)] for e in msg.effort])

        self.loaders = {
            'tf/tfMessage':           load_odom_data,
            'sensor_msgs/JointState': load_joint_state_data,
        }

    def run(self):
        bag_file = BagFile(self.bag_file.path)

        start_stamp = self.bag_index._data.get_start_stamp()
        end_stamp   = self.bag_index._data.get_end_stamp()

        last_stamp = None

        datax, datay = None, None
        
        for stamp in self.subdivide(start_stamp, end_stamp):
            if not self.bag_index:
                break
            
            index = self.bag_index._data.find_stamp_index(self.topic, stamp)
            if index is None:
                continue

            pos = self.bag_index._data.msg_positions[self.topic][index][1]

            (datatype, msg, msg_stamp) = bag_file.load_message(pos, self.bag_index)
            if not msg:
                continue

            if datatype in self.loaders:
                series_list, data = self.loaders[datatype](msg)
                if len(data) > 0:
                    if not datax:
                        self.plot._init_plot(series_list)
                        
                        datax, datay = [], []
                        for subplot_series in series_list:
                            for series in subplot_series:
                                datax.append([])
                                datay.append([])

                    for i, series_data in enumerate(data):
                        x = [s.to_sec() - start_stamp for (s, y) in series_data]
                        y = [y for (s, y) in series_data]

                        datax[i].extend(x)
                        datay[i].extend(y)

                        self.plot.set_series_data(i, datax[i], datay[i])

            if last_stamp and abs(stamp - last_stamp) < 0.1:
                break
            last_stamp = stamp

            time.sleep(0.002)

    @staticmethod
    def subdivide(left, right):
        yield left
        yield right

        intervals = collections.deque([(left, right)])
        while True:
            (left, right) = intervals.popleft()
            
            mid = (left + right) / 2
            
            intervals.append((left, mid))
            intervals.append((mid,  right))
            
            yield mid
