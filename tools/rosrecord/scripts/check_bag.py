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

def checkbag(inbag, genrules):
  broken = dict()

  for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
    bagtype = msg[4]
    rostype = roslib.scriptutil.get_message_class(msg[0])
    if (bagtype._md5sum != rostype._md5sum):
      broken[msg[0]] = (bagtype,rostype)
    
  if not genrules:
    for t,(o,n) in broken.iteritems():
      print "Incompatible definitions for %s:\n---OLD---\n%s\n---NEW---\n%s\n-----\n"%(t,o._full_text,n._full_text)
  else:
    for t,(o,n) in broken.iteritems():
      print "def rule_%s_%s(msg):"%(o._md5sum,n._md5sum)
      print "  if (msg._md5sum != '%s'):"%o._md5sum
      print "    return None"
      print "  new_type='%s'"%n._type
      print "  new_def=\"\"\"%s\"\"\""%n._full_text
      print "  new_msg_type = roslib.genpy.generate_dynamic(new_type, new_def)[new_type]"
      print "  new_msg = new_msg_type()"
      print ""
      print "  # Make assignments to new_msg here"
      print ""
      print "  return new_msg"
      print "rules['%s'] = rule_%s_%s"%(o._md5sum,o._md5sum,n._md5sum)
      print ""

  if (len(broken)==0):
    print "No incompatible definitions found."

if __name__ == '__main__':
  import sys

  parser = OptionParser(usage="usage: %prog [-g] <inbag>")
  parser.add_option("-g","--genrules",
                    action="store_true", dest="genrules")

  (options, args) = parser.parse_args()

  if len(args) == 1:
    checkbag(args[0], options.genrules)
  else:
    print "usage: check_bag [-g] <inbag>"
