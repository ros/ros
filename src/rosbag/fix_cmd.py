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
import os
import signal
import subprocess
import optparse
from optparse import OptionParser

import rosbagmigration

def fix_cmd(argv):
    parser = OptionParser(usage="rosbag fix INBAG OUTBAG [EXTRARULES1 EXTRARULES2 ...]")

    parser.add_option("-n","--noplugins",action="store_false",dest="noplugins",
                      help = "Do not load rulefiles via plugins.")

    (options, args) = parser.parse_args(argv)
  
    if len(args) >= 2:
        if args[1].split('.')[-1] == "bmr":
            parser.error("Second argument should be a bag, not a rule file.")

        if args[1].split('.')[-1] != "bag":
            parser.error("Output file must be a bag (not a .gz or .bz2)")

        mm = rosbagmigration.MessageMigrator(args[2:], plugins=options.noplugins)

        if rosbagmigration.fixbag(mm, args[0], args[1]):
            print "Bag migrated successfully."
        else:
            print "Bag could not be migrated"
    else:
        parser.error("Must pass in 2 bagfiles")
