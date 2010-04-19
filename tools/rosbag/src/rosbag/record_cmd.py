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

import optparse
import signal
import subprocess
import sys
from optparse import OptionParser

def record_cmd(argv):
    parser = OptionParser(usage="rosbag record TOPIC1 [TOPIC2 TOPIC3 ...]",
                          description="Record a bag file with the contents of specified topics.",
                          formatter=optparse.IndentedHelpFormatter())

    parser.add_option("-a", "--all",           dest="all",      default=False, action="store_true",        help="record all topics")
    parser.add_option("-q", "--quiet",         dest="quiet",    default=False, action="store_true",        help="suppress console output")
    parser.add_option("-o", "--output-prefix", dest="prefix",   default=None,  action="store",             help="prepend PREFIX to beginning of bag name (name will always end with date stamp)")
    parser.add_option("-O", "--output-name",   dest="name",     default=None,  action="store",             help="record to bag with namename NAME.bag")
    parser.add_option("--split",               dest="split",    default=0,     type='int', action="store", help="split bag into files of size SIZE", metavar="SIZE")
    parser.add_option("-b", "--buffsize",      dest="buffsize", default=256,   type='int', action="store", help="use in internal buffer of SIZE MB (Default: %default, 0 = infinite)", metavar="SIZE")
    parser.add_option("-l", "--limit",         dest="num",      default=0,     type='int', action="store", help="only record NUM messages on each topic")
    #parser.add_option("-z", "--zlib",          dest="zlib",     default=False, action="store_true",        help="use ZLIB compression")
    parser.add_option("-j", "--bz2",           dest="bz2",      default=False, action="store_true",        help="use BZ2 compression")

    (options, args) = parser.parse_args(argv)

    if len(args) == 0 and not options.all:
        parser.error("You must specify a topic name or else use the '-a' option.")

    if options.prefix is not None and options.name is not None:
        parser.error("Can't set both prefix and name.")

    cmd = ['record']

    cmd.extend(['-m', str(options.buffsize)])
    cmd.extend(['-c', str(options.num)])
    cmd.extend(['-S', str(options.split)])

    if options.prefix: cmd.extend(["-f", options.prefix])
    if options.name:   cmd.extend(["-F", options.name])
    if options.all:    cmd.extend(["-a"])
    #if options.zlib:   cmd.extend(["-z"])
    if options.bz2:    cmd.extend(["-j"])

    cmd.extend(args)

    proc = subprocess.Popen(cmd)

    # Ignore sigint since we're basically just pretending to be the subprocess now.
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    res = proc.wait()

    sys.exit(res)
