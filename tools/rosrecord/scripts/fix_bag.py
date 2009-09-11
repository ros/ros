#!/usr/bin/env python
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

PKG = 'rosrecord'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord
import fileinput

from optparse import OptionParser

def fixbag(rulesfile, inbag, outbag):
  rules = dict()

  execfile(rulesfile)

  rebag = rosrecord.Rebagger(outbag)

  for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
    bagtype = msg[4]
    if (rules.has_key(bagtype._md5sum)):
      bagmsg = bagtype()
      bagmsg.deserialize(msg[1])
      newmsg = rules[bagtype._md5sum](bagmsg)
      rebag.add(topic, newmsg, t)
    else:
      rebag.add(topic, (msg[0], msg[1], msg[2], msg[4]), t, raw=True)

  rebag.close()

if __name__ == '__main__':
  import sys

  parser = OptionParser(usage="usage: %prog <rulesfile> <inbag>")

  (options, args) = parser.parse_args()

  if len(args) == 3:
    fixbag(args[0], args[1], args[2])
  else:
    print "usage: fix_bag <rulesfile> <inbag> <outbag>"
