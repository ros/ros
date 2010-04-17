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

import roslib; roslib.load_manifest('rosbag')

import sys
import os
import signal
import subprocess
import optparse
from optparse import OptionParser

import rosbagmigration

def print_trans(old,new,indent):
    from_txt = "%s [%s]"%(old._type, old._md5sum)
    if new is not None:
        to_txt= "%s [%s]"%(new._type, new._md5sum)
    else:
        to_txt = "Unknown"
    print "    "*indent + " * From: %s"%(from_txt,)
    print "    "*indent + "   To:   %s"%(to_txt,)

def fix_cmd(argv):
    parser = OptionParser(usage="rosbag fix INBAG OUTBAG [EXTRARULES1 EXTRARULES2 ...]")

    parser.add_option("-n","--noplugins",action="store_true",dest="noplugins",
                      help = "do not load rulefiles via plugins")

    (options, args) = parser.parse_args(argv)

    if len(args) >= 2:
        if args[1].split('.')[-1] == "bmr":
            parser.error("Second argument should be a bag file, not a rule file.")

        if args[1].split('.')[-1] != "bag":
            parser.error("Output file must be a .bag file (not a .gz or .bz2)")

        outname = args[1] + '.tmp'

        if os.path.exists(args[1]):
            if not os.access(args[1], os.W_OK):
                print >> sys.stderr, "Don't have permissions to access %s"%args[1]
                sys.exit(1)
        else:
            try:
                file = open(args[1],'w')
                file.close()
            except IOError, e:
                print >> sys.stderr, "Cannot open %s for writing"%args[1]
                sys.exit(1)

        if os.path.exists(outname):
            if not os.access(outname, os.W_OK):
                print >> sys.stderr, "Don't have permissions to access %s"%outname
                sys.exit(1)
        else:
            try:
                file = open(outname,'w')
                file.close()
            except IOError, e:
                print >> sys.stderr, "Cannot open %s for writing"%outname
                sys.exit(1)

        if (options.noplugins is None):
            options.noplugins = False

        mm = rosbagmigration.MessageMigrator(args[2:], plugins=not options.noplugins)

        migrations = rosbagmigration.fixbag2(mm, args[0], outname)

        if migrations == []:
            print "%s %s"%(outname, args[1])
            os.rename(outname, args[1])
            print "Bag migrated successfully."

        else:
            print "Bag could not be migrated.  The following migrations could not be performed:"
            for m in migrations:
                print_trans(m[0][0].old_class, m[0][-1].new_class, 0)
                if len(m[1]) > 0:
                    print "    %d rules missing:"%(len(m[1]))
                    for r in m[1]:
                        print_trans(r.old_class, r.new_class,1)
            print "Try running 'rosbag check' to create the necessary rule files."
            os.remove(outname)

    else:
        parser.error("Must pass in 2 bag files")
