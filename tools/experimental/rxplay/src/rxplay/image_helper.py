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

import Image
import wx

import cv
from cv_bridge import CvBridge, CvBridgeError

## Helper class for converting ROS sensor_msgs/Image <-> wxImage
class ImageHelper:
    cv_bridge = CvBridge()

    @classmethod
    def load_image_from_msg(cls, msg):
        if cls.valid_encoding(msg):
            try:
                if msg.encoding == 'bayer_bggr8':
                    msg.encoding = 'mono8'
                    
                if msg.encoding == 'mono8':
                    cv_img  = cls.cv_bridge.imgmsg_to_cv(msg)
                    pil_img = Image.fromstring('L', cv.GetSize(cv_img), cv_img.tostring()).convert('RGB')
                else:
                    cv_img  = cls.cv_bridge.imgmsg_to_cv(msg, desired_encoding='rgb8')
                    pil_img = Image.fromstring('RGB', cv.GetSize(cv_img), cv_img.tostring())
    
                return wx.ImageFromData(pil_img.size[0], pil_img.size[1], pil_img.tostring())
            
            except CvBridgeError, e:
                rospy.logwarn('CvBridgeError: %s' % str(e))
            except AttributeError, e:
                rospy.logwarn('AttributeError: %s' % str(e))

        return None

    @classmethod
    def valid_encoding(cls, msg):
        return True# msg.encoding != 'bayer_bggr8'
