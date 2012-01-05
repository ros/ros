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

from __future__ import print_function

import os
import sys

import rospkg

import roslaunch.rlutil

def check_roslaunch_file(roslaunch_file):
    print("checking", roslaunch_file)
    error_msg = roslaunch.rlutil.check_roslaunch(roslaunch_file)
    # error message has to be XML attr safe
    if error_msg:
        return "[%s]:\n\t%s"%(roslaunch_file,error_msg)

def check_roslaunch_dir(roslaunch_dir):
    error_msgs = []
    for f in os.listdir(roslaunch_dir):
        if f.endswith('.launch'):
            roslaunch_file = os.path.join(roslaunch_dir, f)
            if os.path.isfile(roslaunch_file):
                error_msgs.append(check_roslaunch_file(roslaunch_file))
    # error message has to be XML attr safe
    return '\n'.join([e for e in error_msgs if e])

## run check and output test result file
if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser(usage="usage: \troslaunch-check [options] <file|directory> [env=value...]")
    # #3674
    parser.add_option("-o", "--output-file",
                      dest="output_file", default=None,
                      help="file to store test results in", metavar="FILE")

    options, args = parser.parse_args()
    if not args:
        parser.error('please specify a file or directory for roslaunch files to test')
    roslaunch_path = args[0]

    # #2590: implementing this quick and dirty as this script should only be used by higher level tools
    env_vars = args[1:]
    for e in env_vars:
        var, val = e.split('=')
        os.environ[var] = val

    pkg = rospkg.get_package_name(roslaunch_path)
    r = rospkg.RosPack()
    pkg_dir = r.get_path(pkg)

    if os.path.isfile(roslaunch_path):
        error_msg = check_roslaunch_file(roslaunch_path)
        outname = os.path.basename(roslaunch_path).replace('.', '_')
    else:
        print("checking *.launch in directory", roslaunch_path)
        error_msg = check_roslaunch_dir(roslaunch_path)
        abspath = os.path.abspath
        relpath = abspath(roslaunch_path)[len(abspath(pkg_dir))+1:]
        outname = relpath.replace(os.sep, '_')
        if outname == '.':
            outname = '_pkg'

    import rostest.rostestutil
    if options.output_file:
        test_file = options.output_file
        # If we were given a test file name to write, then construct the
        # test name from it, to ensure uniqueness
        test_name = os.path.basename(test_file)
    else:
        test_file = rostest.rostestutil.xmlResultsFile(pkg, "roslaunch_check_"+outname, is_rostest=False)    
        test_name = roslaunch_path

    print("...writing test results to", test_file)

    dir_path = os.path.abspath(os.path.dirname(test_file))
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

    if error_msg:
        print("FAILURE:\n%s"%error_msg, file=sys.stderr)
        with open(test_file, 'w') as f:
            message = "roslaunch check [%s] failed"%(roslaunch_path)
            f.write(rostest.rostestutil.test_failure_junit_xml(test_name, message, stdout=error_msg))
        print("wrote test file to [%s]"%test_file)
        sys.exit(1)
    else:
        print("passed")
        with open(test_file, 'w') as f:
            f.write(rostest.rostestutil.test_success_junit_xml(test_name))            
