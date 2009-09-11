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

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import struct
import unittest
from cStringIO import StringIO
import time
import random

import rostest

import rospy.rostime

class TestRospyTime(unittest.TestCase):
    
    def test_switch_to_wallclock(self):
        rospy.rostime.switch_to_wallclock()
        self.assertAlmostEqual(time.time(), rospy.get_time(), 1)
                                 
    def test_update_rostime(self):
        from rospy.rostime import _update_rostime
        from rospy import Time 
        from roslib.msg import Time as TimeMsg

        self.assertAlmostEqual(time.time(), rospy.get_time(), 1)

        t = Time.from_seconds(1.0)
        _update_rostime(TimeMsg(None, t))
        self.assertEquals(t, rospy.get_rostime())
        self.assertEquals(t.to_time(), rospy.get_time())        

        t = Time.from_seconds(4.0)        
        _update_rostime(TimeMsg(None, t))
        self.assertEquals(t, rospy.get_rostime())
        self.assertEquals(t.to_time(), rospy.get_time())        

    def test_get_rostime(self):
        rospy.rostime.switch_to_wallclock()
        self.assertAlmostEqual(time.time(), rospy.get_time(), 1)
        self.assertAlmostEqual(time.time(), rospy.get_rostime().to_time(), 1)
        #rest of get_rostime implicitly tested by update_rostime tests

    def test_sleep(self):
        # test wallclock sleep
        rospy.rostime.switch_to_wallclock()
        rospy.sleep(0.1)
        rospy.sleep(rospy.Duration(0.1))        
        
        from rospy.rostime import _update_rostime
        from rospy import Time 
        from roslib.msg import Time as TimeMsg

        t = Time.from_seconds(1.0)
        _update_rostime(TimeMsg(None, t))
        self.assertEquals(t, rospy.get_rostime())
        self.assertEquals(t.to_time(), rospy.get_time())        

        import thread

        #start sleeper
        self.failIf(test_sleep_done)
        thread.start_new_thread(sleeper, ())
        time.sleep(1.0) #make sure thread is spun up
        self.failIf(test_sleep_done)

        t = Time.from_seconds(1000000.0)
        _update_rostime(TimeMsg(None, t))
        time.sleep(0.5) #give sleeper time to wakeup
        self.assert_(test_sleep_done, "sleeper did not wake up")

        #start duration sleeper
        self.failIf(test_duration_sleep_done)                
        thread.start_new_thread(duration_sleeper, ())
        time.sleep(1.0) #make sure thread is spun up
        self.failIf(test_duration_sleep_done)

        t = Time.from_seconds(2000000.0)
        _update_rostime(TimeMsg(None, t))
        time.sleep(0.5) #give sleeper time to wakeup
        self.assert_(test_sleep_done, "sleeper did not wake up")

        #start backwards sleeper
        self.failIf(test_backwards_sleep_done)
        thread.start_new_thread(backwards_sleeper, ())
        time.sleep(1.0) #make sure thread is spun up
        self.failIf(test_backwards_sleep_done)

        t = Time.from_seconds(1.0)
        _update_rostime(TimeMsg(None, t))
        time.sleep(0.5) #give sleeper time to wakeup
        self.assert_(test_backwards_sleep_done, "backwards sleeper was not given an exception")
    
test_duration_sleep_done = False
def duration_sleeper():
    global test_duration_sleep_done
    rospy.sleep(rospy.Duration(10000.0))
    test_duration_sleep_done = True

test_sleep_done = False
def sleeper():
    global test_sleep_done
    rospy.sleep(10000.0)
    test_sleep_done = True
test_backwards_sleep_done = False    
def backwards_sleeper():
    global test_backwards_sleep_done
    try:
        rospy.sleep(10000.0)
    except rospy.ROSException:
        test_backwards_sleep_done = True

if __name__ == '__main__':
    rostest.unitrun('test_rospy', sys.argv[0], TestRospyTime, coverage_packages=['rospy.rostime'])
