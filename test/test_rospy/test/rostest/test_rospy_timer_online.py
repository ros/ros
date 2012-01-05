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

import os
import sys
import unittest
import time

import rosunit

from threading import Thread

class TestRospyTimerOnline(unittest.TestCase):
    
    def __init__(self, *args):
        unittest.TestCase.__init__(self, *args)
        import rospy
        rospy.init_node('test_rospy_timer_online')
        self.timer_callbacks = 0
        self.timer_event = None

    def test_sleep(self):
        import rospy
        import time
        t = time.time()
        rospy.sleep(0.1)
        dur = time.time() - t
        # #2842 raising bounds from .01 to .03 for amazon VM 

        # make sure sleep is approximately right
        self.assert_(abs(dur - 0.1) < 0.03, dur)

        t = time.time()
        rospy.sleep(rospy.Duration.from_sec(0.1))
        dur = time.time() - t
        # make sure sleep is approximately right
        self.assert_(abs(dur - 0.1) < 0.03, dur)

        # sleep for neg duration
        t = time.time()
        rospy.sleep(rospy.Duration.from_sec(-10.))
        dur = time.time() - t
        # make sure returned immediately
        self.assert_(abs(dur) < 0.1, dur)

    def test_Rate(self):
        import rospy
        import time
        t = time.time()
        count = 0
        r = rospy.Rate(10)
        for x in xrange(10):
            r.sleep()
        dur = time.time() - t
        # make sure sleep is approximately right
        self.assert_(abs(dur - 1.0) < 0.5, dur)
        
    def _Timer_callback(self, event):
        self.timer_callbacks += 1
        self.timer_event = event

    def callback(event):
        print 'last_expected:        ', event.last_expected
        print 'last_real:            ', event.last_real
        print 'current_expected:     ', event.current_expected
        print 'current_real:         ', event.current_real
        print 'current_error:        ', (event.current_real - event.current_expected).to_sec()
        print 'profile.last_duration:', event.last_duration
        if event.last_real:
            print 'last_error:           ', (event.last_real - event.last_expected).to_sec(), 'secs'

    def test_Timer(self):
        import rospy
        timer = rospy.Timer(rospy.Duration(1), self._Timer_callback)
        time.sleep(10)
        timer.shutdown()
        
        # make sure we got an approximately correct number of callbacks
        self.assert_(abs(self.timer_callbacks - 10) < 2)
        # make sure error is approximately correct.  the Timer
        # implementation tracks error in accumulated real time.
        ev = self.timer_event
        self.assert_(ev is not None)
        self.assert_(abs((ev.current_real - ev.current_expected).to_sec()) < 2.)
        
if __name__ == '__main__':
    rosunit.unitrun('test_rospy', sys.argv[0], TestRospyTimerOnline, coverage_packages=['rospy.timer'])
