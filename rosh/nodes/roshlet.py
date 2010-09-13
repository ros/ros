#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

from __future__ import with_statement
import roslib; roslib.load_manifest('rosh')

import os
import sys
import optparse

NAME='roshlet.py'

def roshlet_standalone(name, package, type_, plugins):
    import rosh.impl.roshlets
    rosh.impl.roshlets.standalone(name, package, type_, plugins)
    
def roshlet_main(args=None):
    if args is None:
        args=sys.argv
    args = args[1:]
    parser = optparse.OptionParser(usage="usage: %prog pkg/Type [plugins...]", prog=NAME)
    options, args = parser.parse_args(args)

    names = [n for n in args if n.startswith('__name:=')]
    if names:
        if len(names) != 1:
            parser.error("You may only specify one roshlet name")
        name = names[0]
    else:
        name = 'roshlet'

    # filter out remapping args
    import rospy
    args = rospy.myargv(args)
    if len(args) < 1:
        parser.error("Please specify roshlet pkg/Type")
        
    if '/' not in args[0]:
        parser.error("Please specify roshlet pkg/Type, e.g. rosh/echolet.py")
    try:
        package, type_ = args[0].split('/')
    except:
        parser.error("Invalid roshlet pkg/Type: %s"%(args[0]))

    try:
        roshlet_standalone(name, package, type_, args[1:])
    except KeyboardInterrupt:
        # squelch ctrl-c exits
        pass

if __name__ == '__main__':
    roshlet_main()
