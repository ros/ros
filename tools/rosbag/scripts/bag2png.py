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

import sys
import array
import Image

import rospy
import rosbag

def int16_str(d):
    return array.array('B', [ min(x, 255) for x in d ]).tostring()
    #return array.array('f', [ float(x) for x in d ]).tostring()

def msg2im(msg):
    """Take an sensor_msgs/Image and return a PIL image"""
    if len(msg.uint8_data.data) == 0 and len(msg.int16_data.data) == 0:
        return None
    
    if msg.depth == 'uint8':
        ma, image_data = msg.uint8_data, ma.data
    else:
        ma, image_data = msg.int16_data, int16_str(ma.data)
        
    dim = dict([(d.label, d.size) for d in ma.layout.dim])
    mode = { ('uint8',1) : "L", ('uint8',3) : "RGB", ('int16',1) : "L" }[msg.depth, dim['channel']]
    (w, h) = (dim['width'], dim['height'])

    return Image.fromstring(mode, (w, h), image_data)

counter = 0
for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
    if topic.endswith('stereo/raw_stereo'):
        for (mi, c) in [ (msg.left_image, 'L'), (msg.right_image, 'R'), (msg.disparity_image, 'D')]:
            im = msg2im(mi)
            if im:
                ext = { 'L':'png', 'RGB':'png', 'F':'tiff' }[im.mode]
                im.save('%06d%s.%s' % (counter, c, ext))
        counter += 1
