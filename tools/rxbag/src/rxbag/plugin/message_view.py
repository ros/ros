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

class MessageView(object):
    """
    A message details renderer. When registered with rxbag, a MessageView is called
    whenever the timeline playhead moves.
    """
    name = 'Untitled'
    
    def __init__(self, timeline):
        self.timeline = timeline
    
    def message_viewed(self, bag, topic, msg, t):
        """
        View the message.
        
        @param bag: the bag file the message is contained in
        @type  bag: rosbag.Bag
        @param topic: the message topic
        @type  topic: str
        @param msg: the message
        @type  msg: roslib.Message
        @param t: the message timestamp
        @type  t: rospy.Time
        """
        pass

    def message_cleared(self):
        """
        Clear the currently viewed message (if any).
        """
        pass
    
    def timeline_changed(self):
        """
        Called when the messages in a timeline change, e.g. if a new message is recorded, or 
        a bag file is added
        """
        pass
    
    def close(self):
        """
        Close the message view, releasing any resources.
        """
        pass
