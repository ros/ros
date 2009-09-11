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

import sys
import rosbagmigration
import re

import roslib.message

if __name__ == '__main__':
  if len(sys.argv) >= 3:
    f = open(sys.argv[1])

    if f is None:
      print >> sys.stderr, "Could not open message full definition: %s"
      sys.exit()

    type_line = f.readline()
    pat = re.compile(r"\[(.*)]:")
    type_match = pat.match(type_line)
    if type_match is None:
      print >> sys.stderr, "Full definition file malformed.  First line should be: '[my_package/my_msg]:'"
      sys.exit()

    old_type = type_match.groups()[0]
    old_full_text = f.read()

    old_class = roslib.genpy.generate_dynamic(old_type,old_full_text)[old_type]

    if old_class is None:
      print >> sys.stderr, "Could not generate class from full definition file."
      sys.exit()

    new_class = roslib.message.get_message_class(sys.argv[2])

    if new_class is None:
      print >> sys.stderr, "Could not find class corresponding to %s"%sys.argv[2]
      sys.exit()

    # If the type and md5sum match, there is nothing necessary to do
    if ((old_class._type, old_class._md5sum) == (new_class._type, new_class._md5sum)):
      print >> sys.stderr, "Types already match, nothing to do."
      sys.exit()
    
    mm = rosbagmigration.MessageMigrator(sys.argv[3:])

    new_rule = mm.make_update_rule(old_class, new_class)
    R = new_rule(mm, "GENERATED")
    R.find_sub_paths()

    # Print rule definition and all additionally necessery definitions
    R.print_def()
    [r.print_def() for r in mm.get_invalid_rules()]

    
  else:
    print "usage: makerule.py <msg_full_text> <new_msg_name> [rulefile1, rulefile2, ...]"

