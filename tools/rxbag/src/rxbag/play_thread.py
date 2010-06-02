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

from __future__ import with_statement

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)
import rospy

import sys
import threading

import time

class PlayThread(threading.Thread):
    def __init__(self, timeline):
        threading.Thread.__init__(self)

        self.timeline     = timeline
        self.stop_flag    = False
        self.wrap         = True       # should the playhead wrap when it reaches the end?
        self.stick_to_end = False      # should the playhead stick to the end?

        self.setDaemon(True)
        self.start()

    def run(self):
        self._play_all = None

        try:
            while not self.stop_flag:
                # Reset on switch of playing mode
                if self._play_all != self.timeline._play_all or self.timeline.playhead != self.last_playhead:
                    self._play_all = self.timeline._play_all
    
                    self.last_frame       = None
                    self.last_playhead    = None
                    self.desired_playhead = None
    
                if self._play_all:
                    self.step_next_message()
                    time.sleep(0.05)
                else:
                    self.step_fixed()
                    time.sleep(0.05)

        except Exception, ex:
            print >> sys.stderr, 'Error advancing playhead: %s' % str(ex)
            import traceback
            traceback.print_exc(file=sys.stderr)

    def step_fixed(self):
        if self.timeline.play_speed == 0.0 or not self.timeline.playhead:
            self.last_frame    = None
            self.last_playhead = None
            return

        now = rospy.Time.from_sec(time.time())
        if self.last_frame:
            if self.stick_to_end:
                new_playhead = self.timeline.end_stamp
            else:
                new_playhead = self.timeline.playhead + rospy.Duration.from_sec((now - self.last_frame).to_sec() * self.timeline.play_speed)
    
                start_stamp, end_stamp = self.timeline.play_region

                if new_playhead > end_stamp:
                    if self.wrap:
                        if self.timeline.play_speed > 0.0:
                            new_playhead = start_stamp
                        else:
                            new_playhead = end_stamp
                    else:
                        new_playhead = end_stamp

                        if self.timeline.play_speed > 0.0:
                            self.stick_to_end = True

                elif new_playhead < start_stamp:
                    if self.wrap:
                        if self.timeline.play_speed < 0.0:
                            new_playhead = end_stamp
                        else:
                            new_playhead = start_stamp
                    else:
                        new_playhead = start_stamp

            # Update the playhead
            self.timeline.playhead = new_playhead

        self.last_frame    = now
        self.last_playhead = self.timeline.playhead

    def step_next_message(self):
        if self.timeline.play_speed <= 0.0 or not self.timeline.playhead:
            self.last_frame    = None
            self.last_playhead = None
            return

        if self.last_frame:
            if not self.desired_playhead:
                self.desired_playhead = self.timeline.playhead
            else:
                delta = rospy.Time.from_sec(time.time()) - self.last_frame
                if delta > rospy.Duration.from_sec(0.1):
                    delta = rospy.Duration.from_sec(0.1)
                self.desired_playhead += delta

            # Get the occurrence of the next message
            next_message_time = self.timeline.get_next_message_time()

            if next_message_time < self.desired_playhead:
                self.timeline.playhead = next_message_time
            else:
                self.timeline.playhead = self.desired_playhead

        self.last_frame    = rospy.Time.from_sec(time.time())
        self.last_playhead = self.timeline.playhead

    def stop(self):
        self.stop_flag = True
        self.join()
