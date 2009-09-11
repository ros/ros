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
#
# Revision $Id$

PKG = 'rosbagmigration'
import roslib; roslib.load_manifest(PKG)

from rosbagmigration.rules import MessageMigrator
import rospy
import rosrecord
import fileinput

def fixbag(inbag, outbag, rulemodules):
  target_classes = {}

  # Double parsing the bag is inefficient, but we can speed this up later
  mm = MessageMigrator(rulemodules)
  for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
    target_classes[msg[2]] = mm.find_path(msg[4], None)

  # Deserializing all messages is inefficient, but we can speed this up later
  if mm.all_rules_valid():
    rebag = rosrecord.Rebagger(outbag)
    for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag)):
      new_msg = target_classes[msg._md5sum]()
      mm.migrate(msg, new_msg)
      rebag.add(topic, new_msg, t)
    rebag.close()    
  else:
    print "Bag is not fixable.  Use checkbag.py and generate appropriate rules."

if __name__ == '__main__':
  import sys
  if len(sys.argv) >= 3:
    fixbag(sys.argv[1], sys.argv[2], sys.argv[3:])
  else:
    print "usage: fixbag.py <inbag> <outbag> [rulefile1, rulefile2, ...]"

