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

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)
import rospy

import bisect
import time

import wx

from rxbag.util.layer import Layer

class TimelineRenderer(object):
    """
    A custom renderer for interval of time of a topic on the timeline.
    
    @param msg_combine_px: don't draw discrete messages if they're less than this many pixels separated [default: 1.5]
    @type  msg_combine_px: float
    """
    def __init__(self, timeline, msg_combine_px=1.5):
        self.timeline       = timeline
        self.msg_combine_px = msg_combine_px 

    def get_segment_height(self, topic):
        """
        Get the height of the topic segment on the timeline.
        
        @param topic: topic name to draw
        @type  topic: str
        @return: height in pixels of the topic segment. If none, the timeline default is used.
        @rtype:  int or None
        """
        return None

    def draw_timeline_segment(self, dc, topic, stamp_start, stamp_end, x, y, width, height):
        """
        Draw the timeline segment.
        
        @param dc: Cairo device context to render into
        @param topic: topic name
        @param stamp_start: start of the interval on the timeline
        @param stamp_end: start of the interval on the timeline
        @param x: x coordinate of the timeline interval
        @param y: y coordinate of the timeline interval
        @param width: width in pixels of the timeline interval
        @param height: height in pixels of the timeline interval
        @return: whether the interval was renderered
        @rtype:  bool
        """
        return False

    def close(self):
        """
        Close the renderer, releasing any resources.
        """
        pass

