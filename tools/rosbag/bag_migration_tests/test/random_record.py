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

import unittest
import rospy
import rostest
import sys
from cStringIO import StringIO
import time
from random_messages import RandomMsgGen
import subprocess
import signal
import os
import atexit

class RandomRecord(unittest.TestCase):

  def test_random_record(self):
    rospy.init_node('random_pub')
  
    if (len(sys.argv) < 2):
      raise Exception("Expected seed as first argument")

    seed = int(sys.argv[1])

    seed    = int(sys.argv[1])
    topics  = int(sys.argv[2])
    length  = float(sys.argv[3])

    rmg = RandomMsgGen(seed, topics, length)

    publishers = {}

    for (topic, msg_class) in rmg.topics():
      publishers[topic] = rospy.Publisher(topic, msg_class)

    bagpath = os.path.join('/tmp', 'test_rosbag_random_record_%d'%seed)
    cmd = ['rosbag', 'record', '-a', '-O', bagpath]
    f1 = subprocess.Popen(cmd)

    def finalkill():
      try:
        os.kill(f1.pid, signal.SIGKILL)
      except:
        pass

    atexit.register(finalkill)

    # Sleep an extra 5 seconds for good measure
    rospy.sleep(rospy.Duration.from_sec(5.0))

    start = rospy.Time.now()
    for (topic, msg, time) in rmg.messages():
      d = start + rospy.Duration.from_sec(time) - rospy.Time.now()
      rospy.sleep(d)
      publishers[topic].publish(msg)

    # Sleep an extra 5 seconds for good measure
    rospy.sleep(rospy.Duration.from_sec(5.0))

    # Initial terminate using SIGINT so bag clean up nicely
    os.kill(f1.pid, signal.SIGINT)

    # Sleep an extra 5 seconds for good measure
    rospy.sleep(rospy.Duration.from_sec(5.0))

    # Keep trying to kill until it's dead instead of blocking on communicate
    while (f1.poll() is None):
      try:
        os.kill(f1.pid, signal.SIGKILL)
      except:
        pass
      rospy.sleep(rospy.Duration.from_sec(1.0))
      
    self.assertEqual(f1.returncode, 0)


if __name__ == '__main__':
  rostest.rosrun('test_rosbag', 'random_record_play', RandomRecord, sys.argv)
