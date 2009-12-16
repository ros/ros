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

import sys
import signal
import subprocess
import optparse
from optparse import OptionParser

def play_cmd(argv):
    parser = OptionParser(usage="rosbag play BAGFILE1 [BAGFILE2 BAGFILE3 ...]",
                          description="Play back the contents of one or more bag files in a time-synchronized fashion.")
    parser.add_option("-q", "--quiet", dest="quiet", default=False, action="store_true",
                      help="suppress console output")

    parser.add_option("-i", "--immediate",  dest="immediate",  default=False, action="store_true",
                      help="play back all messages without waiting")                      
    parser.add_option("--pause",      dest="pause",      default=False, action="store_true",
                      help="start in paused mode")

    parser.add_option("--queue",      dest="queue",   default=0,   type='int', action="store",
                      help="use an outgoign queue of size SIZE (defaults to %default)", metavar="SIZE")
    parser.add_option("--frequency",  dest="freq",       default=100, type='float', action="store",
                      help="publish the log time at a frequency of HZ (default: %default)", metavar="HZ")
    parser.add_option("-d", "--delay",      dest="delay",      default=0.2, type='float', action="store",
                      help="Sleep SEC seconds after every advertise call (to allow subscribers to connect).", metavar="SEC")
    parser.add_option("-r", "--rate",       dest="rate",       default=1.0, type='float', action="store",
                      help="multiply the publish rate by FACTOR", metavar="FACTOR")
    parser.add_option("-s", "--start",      dest="sleep",      default=0.0, type='float', action="store",
                      help="start TIME seconds into the bag files", metavar="TIME")


    (options, args) = parser.parse_args(argv)

    if (len(args) == 0 and not options.all):
        parser.error("You must specify at least 1 bagfile to play back.")

    cmd = ["rosplay"]

    if options.quiet:     cmd.extend(["-n"])
    if options.pause:     cmd.extend(["-p"])
    if options.immediate: cmd.extend(["-a"])

    cmd.extend(["-q", str(options.queue)])
    cmd.extend(["-b", str(options.freq)])
    cmd.extend(["-r", str(options.rate)])
    cmd.extend(["-s", str(options.delay)])
    cmd.extend(["-t", str(options.sleep)])


    cmd.extend(args)

    proc = subprocess.Popen(cmd)

    # Ignore sigint since we're basically just pretending to be the subprocess now.
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    res = proc.wait()

    sys.exit(res)
