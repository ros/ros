#!/usr/bin/env python

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord
import fileinput

def fastrebag(inbag, outbag):
  rebag = rosrecord.Rebagger(outbag)
  for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
    rebag.add(topic, msg, t, raw=True)
  rebag.close()

if __name__ == '__main__':
  import sys
  if len(sys.argv) == 3:
    fastrebag(sys.argv[1], sys.argv[2])
  else:
    print "usage: fastrebag.py <inbag> <outbag>"
