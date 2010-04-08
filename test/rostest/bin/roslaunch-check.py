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
#
# Revision $Id$

from __future__ import with_statement
import roslib; roslib.load_manifest('rostest')

import os
import sys

import roslib.packages

import roslaunch.rlutil

def usage():
    print >> sys.stderr, """Usage:
\troslaunch-check [file|directory]
"""
    print sys.argv
    sys.exit(os.EX_USAGE)

def check_roslaunch_file(roslaunch_file):
    print "checking", roslaunch_file
    error_msg = roslaunch.rlutil.check_roslaunch(roslaunch_file)
    if error_msg:
        return "[%s]:\n\t%s"%(roslaunch_file,error_msg)

def check_roslaunch_dir(roslaunch_dir):
    error_msgs = []
    for f in os.listdir(roslaunch_dir):
        if f.endswith('.launch'):
            roslaunch_file = os.path.join(roslaunch_dir, f)
            if os.path.isfile(roslaunch_file):
                error_msgs.append(check_roslaunch_file(roslaunch_file))
    return '\n'.join([e for e in error_msgs if e])

## run check and output test result file
if __name__ == '__main__':
    if len(sys.argv) < 2:
        usage()
    roslaunch_path = sys.argv[1]

    pkg_dir, pkg = roslib.packages.get_dir_pkg(roslaunch_path) 

    if os.path.isfile(roslaunch_path):
        error_msg = check_roslaunch_file(roslaunch_path)
        outname = os.path.basename(roslaunch_path).replace('.', '_')
    else:
        print "checking *.launch in directory", roslaunch_path        
        error_msg = check_roslaunch_dir(roslaunch_path)
        outname = os.path.relpath(os.path.abspath(roslaunch_path), pkg_dir).replace(os.sep, '_')
        if outname == '.':
            outname = '_pkg'

    import rostest.rostestutil
    test_file = rostest.rostestutil.xmlResultsFile(pkg, "roslaunch_check_"+outname, is_rostest=False)    

    print "...writing test results to", test_file

    test_name = roslaunch_path
    if error_msg:
        print>> sys.stderr, "FAILURE:\n%s"%error_msg
        if not os.path.exists(os.path.dirname(test_file)):
            os.makedirs(os.path.dirname(test_file))
        with open(test_file, 'w') as f:
            message = "roslaunch check [%s] failed:\n %s"%(roslaunch_path, error_msg)
            f.write(rostest.rostestutil.test_failure_junit_xml(test_name, message))
            f.close()
        print "wrote test file to [%s]"%test_file
        sys.exit(1)
    else:
        print "passed"
        with open(test_file, 'w') as f:
            f.write(rostest.rostestutil.test_success_junit_xml(test_name))            
