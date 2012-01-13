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

import os
import rospy
import rosbag

def fix_md5sums(inbags):
    for b in inbags:
        print('Trying to migrating file: %s' % b)
        outbag = b + '.tmp'
        rebag = rosbag.Bag(outbag, 'w')
        try:
            for i,(topic, msg, t) in enumerate(rosbag.Bag(b).read_messages(raw=True)):
                rebag.write(topic, msg, t, raw=True)
            rebag.close()
        except rosbag.ROSBagException as e:
            print(' Migration failed: %s' % str(e))
            os.remove(outbag)
            continue
        
        oldnamebase = b + '.old'
        oldname = oldnamebase
        i = 1
        while os.path.isfile(oldname):
            i += 1
            oldname = oldnamebase + str(i)
        os.rename(b, oldname)
        os.rename(outbag, b)
        print(' Migration successful.  Original stored as: %s' % oldname)

if __name__ == '__main__':
    import sys
    if len(sys.argv) >= 2:
        fix_md5sums(sys.argv[1:])
    else:
        print("usage: fix_md5sums.py bag1 [bag2 bag3 ...]")
