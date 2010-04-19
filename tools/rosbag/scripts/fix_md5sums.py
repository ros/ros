#!/usr/bin/env python

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import fileinput
import os

def fix_md5sums(inbags):
  for b in inbags:
    print "Trying to migrating file: %s"%b
    outbag = b+'.tmp'
    rebag = rosrecord.Rebagger(outbag)
    try:
      for i,(topic, msg, t) in enumerate(rosrecord.logplayer(b, raw=True)):
        rebag.add(topic, msg, t, raw=True)
      rebag.close()
    except rosrecord.ROSRecordException, e:
      print " Migration failed: %s"%(e.message)
      os.remove(outbag)
      continue
    oldnamebase = b+'.old'
    oldname = oldnamebase
    i = 1
    while os.path.isfile(oldname):
      i=i+1
      oldname = oldnamebase + str(i)
    os.rename(b, oldname)
    os.rename(outbag, b)
    print " Migration successful.  Original stored as: %s"%oldname

if __name__ == '__main__':
  import sys
  if len(sys.argv) >= 2:
    fix_md5sums(sys.argv[1:])
  else:
    print "usage: fix_md5sums.py bag1 [bag2 bag3 ...]"

