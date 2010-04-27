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

import roslib; roslib.load_manifest('rosbag')

import optparse
import os
import signal
import subprocess
import sys

import rosbag

def filter_cmd(argv):
    def expr_eval(expr):
        def eval_fn(topic, m, t):
            return eval(expr)
        return eval_fn

    parser = optparse.OptionParser(usage="""rosbag filter INBAG OUTBAG EXPRESSION

EXPRESSION can be any Python-legal expression.

The following variables are available:
 * topic: name of topic
 * m: message
 * t: time of message (t.secs, t.nsecs)""")
    parser.add_option('--print', dest='verbose_pattern', default=None, metavar='PRINT-EXPRESSION', help='Python expression to print for verbose debugging. Uses same variables as filter-expression')

    options, args = parser.parse_args(argv)
    if len(args) == 0:
        parser.error('You must specify an in bag, an out bag, and an expression.')
    if len(args) == 1:
        parser.error('You must specify an out bag and an expression.')
    if len(args) == 2:
        parser.error("You must specify an expression.")
    if len(args) > 3:
        parser.error("Too many arguments.")

    inbag_filename, outbag_filename, expr = args

    if not os.path.isfile(inbag_filename):
        print >> sys.stderr, "Cannot locate input bag file [%s]" % inbag_filename
        sys.exit(2)

    filter_fn = expr_eval(expr)

    outbag = rosbag.Bag(outbag_filename, 'w')
    inbag  = rosbag.Bag(inbag_filename)

    try:
        if options.verbose_pattern:
            verbose_pattern = expr_eval(options.verbose_pattern)
    
            for topic, msg, t in inbag.readMessages():
                if filter_fn(topic, msg, t):
                    print "MATCH", verbose_pattern(topic, msg, t)
                    outbag.write(topic, msg, t)
                else:
                    print "NO MATCH", verbose_pattern(topic, msg, t)          
        else:
            for topic, msg, t in inbag.readMessages():
                if filter_fn(topic, msg, t):
                    outbag.write(topic, msg, t)
    finally:
        inbag.close()
        outbag.close()
