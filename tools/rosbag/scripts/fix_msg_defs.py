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

import sys
import rosbag.migration

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('usage: fix_msg_defs.py <inbag> <outbag>')
        exit(2)

    mm = rosbag.migration.MessageMigrator()

    checked = set()
    migrations = []

    inbag = rosbag.Bag(sys.argv[1], 'r')
    outbag = rosbag.Bag(sys.argv[2], 'w')
    lookup_cache = {}

    #msg is: datatype, data, pytype._md5sum, bag_pos, pytype
    for topic, msg, t in inbag.read_messages(raw=True):
        if msg[4]._md5sum != msg[2]:
            k = (msg[0], msg[2])
            if k in lookup_cache:
                real_msg_type = lookup_cache[k]
            else:
                real_msg_type = mm.lookup_type(k)
                if real_msg_type != None:
                    print("FOUND: %s [%s] was defined in migration system\n"%(msg[0], msg[2]), file=sys.stderr)
                else:
                    systype = roslib.message.get_message_class(msg[0])
                    if systype != None and systype._md5sum == msg[2]:
                        real_msg_type = systype
                        print("FOUND: %s [%s] was defined on your package path\n"%(msg[0], msg[2]), file=sys.stderr)
                if real_msg_type == None:
                    real_msg_type = msg[4]
                    print("WARNING: Type [%s] with md5sum [%s] has an unknown definition.\n"%(msg[0], msg[2]), file=sys.stderr)
                lookup_cache[k] = real_msg_type
            outbag.write(topic, (msg[0], msg[1], msg[2], msg[3], real_msg_type), t, raw=True)
        else:
            outbag.write(topic, msg, t, raw=True)
        
    inbag.close()
    outbag.close()

    exit(0)
