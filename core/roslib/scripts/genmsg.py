#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id: genmsg.py 2197 2008-09-19 23:26:54Z sfkwc $
# $Author: sfkwc $
"""
Script for running message generators. Introspects installation
setup to figure out which ROS client implementations to run
"""

import os, sys, glob, traceback
from subprocess import Popen, PIPE

import roslib.msgs

def usage(progname, stdout=sys.stdout):
    print >> stdout,  "%(progname)s file(s)"%vars()

def check_header(scriptPath, stdout, stderr):
    scripts = os.path.dirname(os.path.realpath(scriptPath))
    roslib_dir = os.path.dirname(scripts)
    header = os.path.join(roslib_dir, 'msg', 'Header.msg')
    if not os.path.isfile(header):
        print >> stderr, "ERROR: cannot locate Header.msg. Excepted to find it at '%s'"%header
        return False
    try:
        name, spec = roslib.msgs.load_from_file(header, '')
        names = spec.names
        expected = ['frame_id', 'stamp', 'seq']
        missing = [x for x in expected if x not in names]
        if missing:
            print >> stderr, "ERROR: Header.msg is missing the following required fields: %s"%missing
            return False
    except:
        traceback.print_exc()
        print >> stderr, "ERROR: cannot validate Header.msg in roslib."%header
        return False
    return True

def genmsgMain(argv, stdout, stderr, env):
    #optlist, args = getopt.getopt(argv[1:], "sn:p:u:", ["help", "test", "debug"])
    if not argv[1:]:
        usage(argv[0], stdout)
    files = []
    for arg in argv[1:]:
        files.extend(glob.glob(arg))
    if not files:
        return #prevent side-effects
    #error check Header.msg before calling sub-generators
    if not check_header(argv[0], stdout, stderr):
        sys.exit(1)
    
    pkgs_table = (Popen(['rospack', 'list'], stdout=PIPE).communicate()[0] or '').strip().split('\n')

    pkgs = [x.split(' ') for x in pkgs_table]
    if not pkgs:
        print >> stderr, "ERROR: cannot locate any ROS installations. Are ROS_ROOT and ROS_PACKAGE_PATH set?"
        sys.exit(1)
    gens = [x for x in pkgs if x[0].startswith('ros')]
    gens = map(lambda x: os.path.join(x[1], 'scripts', 'genmsg_%s'%x[0].split('-')[0][3:]), gens)
    gens = filter(os.path.exists, gens)
    #run all the located generators and report the results
    if gens:
        print >> stdout, 'Found message generators:\n * '+'\n * '.join(gens)
    exits = [os.spawnv(os.P_WAIT, g, [g]+argv[1:]) for g in gens]
    if gens:
        print >> stdout, "\nSummary:"
        for g, code in zip(gens, exits):
            
            if code == os.EX_OK:
                str = "SUCCESS" 
            else:
                str = "FAIL"
            print >> stdout, " * %s: %s"%(os.path.basename(g), str)
    else:
        print >> stdout, 'No message generators found'
    if filter(lambda x: x != os.EX_OK, exits):
        sys.exit(1)

if __name__ == "__main__":
    genmsgMain(sys.argv, sys.stdout, sys.stderr, os.environ)
