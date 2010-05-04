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

import roslib
roslib.load_manifest('test_rosrecord')

import unittest
import rospy
import rostest
import sys
from cStringIO import StringIO
import time
from random_messages import RandomMsgGen
import subprocess
import os

DELAY = 0.5

class RandomPlay(unittest.TestCase):

  def msg_cb_topic(self, topic):
    def msg_cb(msg):
      nowtime = rospy.Time.now()
      if self.start is None:
        self.start = nowtime

      if topic not in self.topics_seen:
        self.delay += DELAY
        self.topics_seen.add(topic)

      # We know this message is coming in after a delay
      self.input.append((topic, msg, (nowtime-self.start-rospy.Duration(self.delay)).to_sec()))

    return msg_cb


  def test_random_play(self):
    rospy.init_node('random_sub', anonymous=True)

    self.start = None
    self.input = []

    self.assertTrue(len(sys.argv[1]) > 3)

    seed    = int(sys.argv[1])
    topics  = int(sys.argv[2])
    length  = float(sys.argv[3])
    scale   = float(sys.argv[4])

    rmg = RandomMsgGen(int(seed), topics, length)

    self.delay = -float(DELAY)
    self.topics_seen = set()

    subscribers = {}
    for (topic, msg_class) in rmg.topics():
      subscribers[topic] = rospy.Subscriber(topic, msg_class, self.msg_cb_topic(topic))

    binpath = os.path.join(roslib.rospack.rospackexec(['find', 'rosrecord']), 'bin', 'rosplay')
    bagpath = os.path.join(roslib.rospack.rospackexec(['find', 'test_rosrecord']), 'test', 'recording_%d.bag'%seed)
    cmd = [binpath, '-s', str(DELAY), '-r', str(scale), bagpath]
    f1 = subprocess.Popen(cmd)

    r = rospy.Rate(10)
    while (len(self.input) < rmg.message_count()):
      r.sleep()

    self.assertEqual(len(self.input), rmg.message_count())

    for (expect_topic, expect_msg, expect_time) in rmg.messages():

      expect_time /= scale

      buff = StringIO()
      expect_msg.serialize(buff)
      expect_msg.deserialize(buff.getvalue())

      msg_match = False

      for ind in xrange(0,100):
        (input_topic, input_msg, input_time) = self.input[ind]

        if (roslib.message.strify_message(expect_msg) == roslib.message.strify_message(input_msg)):
          msg_match = True
          del self.input[ind]
          # Messages can arrive late, but never very early
          self.assertTrue(input_time - expect_time > -.1)
          # .5 seconds late is a pretty big deal...
          self.assertTrue(abs(input_time - expect_time) < .1 + DELAY)
          break

      if not msg_match:
        print "No match at time: %f"%expect_time

      self.assertTrue(msg_match)

    (o1,e1) = f1.communicate()    
    self.assertEqual(f1.returncode, 0)

if __name__ == '__main__':
  rostest.rosrun('test_rosrecord', 'random_record_play', RandomPlay, sys.argv)
