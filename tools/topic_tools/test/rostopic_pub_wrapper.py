#!/usr/bin/env python

# A wrapper to allow rostopic to be called as a node from roslaunch /
# rostest.

import roslib; roslib.load_manifest('topic_tools')
import rospy
import sys
import os
import subprocess

cmd = ['rostopic']
args = rospy.myargv()
cmd.extend(args[1:])
try:
    subprocess.check_call(cmd)
except KeyboardInterrupt:
    pass

