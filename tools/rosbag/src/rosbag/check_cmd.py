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
import os
import signal
import subprocess
import sys

def check_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag check BAG [-g RULEFILE] [EXTRARULES1 EXTRARULES2 ...]')
    parser.add_option('-g', '--genrules',  action='store',      dest='rulefile', default=None, help='generate a rulefile named RULEFILE')
    parser.add_option('-a', '--append',    action='store_true', dest='append',                 help='append to the end of an existing rulefile after loading it')
    parser.add_option('-n', '--noplugins', action='store_true', dest='noplugins',              help='do not load rulefiles via plugins')
    (options, args) = parser.parse_args(argv)

    if len(args) == 0:
        parser.error('Must specify a bag file to check.')
    if options.append and options.rulefile is None:
        parser.error('Cannot specify -a without also specifying -g.')
    if options.rulefile is not None:
        rulefile_exists = os.path.isfile(options.rulefile)
        if rulefile_exists and not options.append:
            parser.error('The file %s already exists.  Include -a if you intend to append.' % options.rulefile)
        if not rulefile_exists and options.append:
            parser.error('The file %s does not exist, and so -a is invalid.' % options.rulefile)

    cmd = ['rosrun', 'rosbag', 'checkbag.py']
    if options.rulefile:  cmd.extend(['-g', options.rulefile])
    if options.append:    cmd.extend(['-a'])
    if options.noplugins: cmd.extend(['-n'])
    cmd.extend(args)

    proc = subprocess.Popen(cmd)
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # ignore sigint since we're basically just pretending to be the subprocess now
    res = proc.wait()
    sys.exit(res)
