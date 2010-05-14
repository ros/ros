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
# Revision $Id: catunit 3804 2009-02-11 02:16:00Z rob_wheeler $

from __future__ import with_statement
import roslib; roslib.load_manifest('rostest')

import os
import sys

import roslib.packages

def usage():
    print >> sys.stderr, """Usage:
\troslaunch-check file.launch
"""
    print sys.argv
    sys.exit(os.EX_USAGE)

def check_roslaunch(f):
    pass

## run check and output test result file
if __name__ == '__main__':
    if len(sys.argv) < 2:
        usage()
    roslaunch_file = sys.argv[1]
    _, package = roslib.packages.get_dir_pkg(roslaunch_file)
    
    import roslaunch.rlutil
    import rostest.rostestutil

    pkg_dir, pkg = roslib.packages.get_dir_pkg(roslaunch_file) 
    outname = os.path.basename(roslaunch_file).replace('.', '_')
    test_file = rostest.rostestutil.xmlResultsFile(pkg, outname, is_rostest=False)
        
    error_msg = roslaunch.rlutil.check_roslaunch(roslaunch_file)
    test_name = roslaunch_file
    print "...writing test results to", test_file
    if error_msg:
        print>> sys.stderr, "FAILURE:\n%s"%error_msg
        if not os.path.exists(os.path.dirname(test_file)):
            os.makedirs(os.path.dirname(test_file))
        with open(test_file, 'w') as f:
            message = "roslaunch file %s failed to parse:\n %s"%(roslaunch_file, error_msg)
            f.write(rostest.rostestutil.test_failure_junit_xml(test_name, message))
            f.close()
        print "wrote test file to [%s]"%test_file
    else:
        print "passed"
        with open(test_file, 'w') as f:
            f.write(rostest.rostestutil.test_success_junit_xml(test_name))            

