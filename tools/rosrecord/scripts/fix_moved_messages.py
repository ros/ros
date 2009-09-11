#!/usr/bin/env python

PKG = 'rosrecord'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord
import fileinput


def fixbags(md5file, inbag, outbag):
  d = dict()
  finput = fileinput.input(md5file)
  for line in finput:
    sp = line.split()
    d[sp[1]] = [sp[0], sp[2], sp[3]]

  rebag = rosrecord.Rebagger(outbag)

  for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
    if rospy.is_shutdown():
      break
    type = msg[0]
    bytes = msg[1]
    md5 = msg[2]

    if md5 in d:
      if type != d[md5][0]:
        print 'WARNING: found matching md5, but non-matching name'
        continue
      msg = (d[md5][1], msg[1], d[md5][2])

    rebag.add(topic, msg, t, raw=True)
  rebag.close()

if __name__ == '__main__':
  import sys
  if len(sys.argv) == 4:
    fixbags(sys.argv[1], sys.argv[2], sys.argv[3])
  else:
    print "usage: fix_moved_messages.py <name_md5_file> <inbag> <outbag>"

