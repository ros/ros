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

from __future__ import print_function

import rospy
import rosbag
import fileinput

def fixbags(md5file, inbag, outbag):
    d = {}
    for line in fileinput.input(md5file):
        sp = line.split()
        d[sp[1]] = [sp[0], sp[2], sp[3]]

    rebag = rosbag.Bag(outbag, 'w')

    for topic, msg, t in rosbag.Bag(inbag).read_messages(raw=True):
        type, bytes, md5 = msg[0], msg[1], msg[2]

        if md5 in d:
            if type != d[md5][0]:
                print('WARNING: found matching md5, but non-matching name')
                continue
            msg = (d[md5][1], msg[1], d[md5][2])

        rebag.write(topic, msg, t, raw=True)

    rebag.close()

if __name__ == '__main__':
    import sys
    if len(sys.argv) == 4:
        fixbags(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print('usage: fix_moved_messages.py <name_md5_file> <inbag> <outbag>')
        exit(2)
