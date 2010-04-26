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

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)

import sys
import rosbag
from rosbag import bag_migration
import roslib.message

from optparse import OptionParser

if __name__ == '__main__':
    parser = OptionParser(usage='usage: savemsg.py [-b <bagfile] type')
    parser.add_option('-b', '--bagfiles', action='store', dest='bagfile', default=None, help='Save message from a bagfile rather than system definition')

    (options, args) = parser.parse_args()

    if len(args) < 1:
        parser.error('Message type not specified')

    if options.bagfile is None:
        sys_class = roslib.message.get_message_class(args[0])
        if sys_class is None:
            print >> sys.stderr, 'Could not find message %s.' % args[0]
        else:
            print '[%s]:' % args[0]
            print sys_class._full_text
    else:
        for i, (topic, msg, t) in enumerate(rosbag.Bag(options.bagfile).readMessages(raw=True)):
            if msg[0] == args[0]:
                print '[%s]:' % args[0]
                print msg[4]._full_text
                exit(0)
        print >> sys.stderr, 'Could not find message %s in bag %s.' % (args[0], options.bagfile)
