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

from __future__ import print_function

import errno
import os
import sys
import logging

import rospkg

from .xmlrunner import XMLTestRunner

XML_OUTPUT_FLAG = '--gtest_output=xml:' #use gtest-compatible flag

def printlog(msg, *args):
    if args:
        msg = msg%args
    print("[ROSUNIT]"+msg)
    
def printlog_bold(msg, *args):
    if args:
        msg = msg%args
    print('\033[1m[ROSUNIT]' + msg + '\033[0m')
    
def printerrlog(msg, *args):
    if args:
        msg = msg%args
    print("[ROSUNIT]"+msg, file=sys.stderr)

# this is a copy of the roslogging utility. it's been moved here as it is a common
# routine for programs using accessing ROS directories
def makedirs_with_parent_perms(p):
    """
    Create the directory using the permissions of the nearest
    (existing) parent directory. This is useful for logging, where a
    root process sometimes has to log in the user's space.
    @param p: directory to create
    @type  p: str
    """    
    p = os.path.abspath(p)
    parent = os.path.dirname(p)
    # recurse upwards, checking to make sure we haven't reached the
    # top
    if not os.path.exists(p) and p and parent != p:
        makedirs_with_parent_perms(parent)
        s = os.stat(parent)
        try:
            os.mkdir(p)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise

        # if perms of new dir don't match, set anew
        s2 = os.stat(p)
        if s.st_uid != s2.st_uid or s.st_gid != s2.st_gid:
            os.chown(p, s.st_uid, s.st_gid)
        if s.st_mode != s2.st_mode:
            os.chmod(p, s.st_mode)    

def xml_results_file(test_pkg, test_name, is_rostest=False, env=None):
    """
    @param test_pkg: name of test's package 
    @type  test_pkg: str
    @param test_name str: name of test
    @type  test_name: str
    @param is_rostest: True if the results file is for a rostest-generated unit instance
    @type  is_rostest: bool
    @return: name of xml results file for specified test
    @rtype:  str
    """
    test_dir = os.path.join(rospkg.get_test_results_dir(env=env), test_pkg)
    if not os.path.exists(test_dir):
        try:
            makedirs_with_parent_perms(test_dir)
        except OSError as error:
            raise IOError("cannot create test results directory [%s]: %s"%(test_dir, str(error)))

    # #576: strip out chars that would bork the filename
    # this is fairly primitive, but for now just trying to catch some common cases
    for c in ' "\'&$!`/\\':
        if c in test_name:
            test_name = test_name.replace(c, '_')
    if is_rostest:
        return os.path.join(test_dir, 'rostest-%s.xml'%test_name)
    else:
        return os.path.join(test_dir, 'rosunit-%s.xml'%test_name)
    
def rostest_name_from_path(pkg_dir, test_file):
    """
    Derive name of rostest based on file name/path. rostest follows a
    certain convention defined above.
    
    @return: name of test
    @rtype: str
    """
    test_file_abs = os.path.abspath(test_file)
    if test_file_abs.startswith(pkg_dir):
        # compute package-relative path
        test_file = test_file_abs[len(pkg_dir):]
        if test_file[0] == os.sep:
            test_file = test_file[1:]
    outname = test_file.replace(os.sep, '_')
    if '.' in outname:
        outname = outname[:outname.rfind('.')]
    return outname

def create_xml_runner(test_pkg, test_name, results_file=None, is_rostest=False):
    """
    Create the unittest test runner with XML output
    @param test_pkg: package name
    @type  test_pkg: str
    @param test_name: test name
    @type  test_name: str
    @param is_rostest: if True, use naming scheme for rostest itself instead of individual unit test naming
    @type  is_rostest: bool
    """
    test_name = os.path.basename(test_name)
    # determine output xml file name
    if not results_file:
        results_file = xml_results_file(test_pkg, test_name, is_rostest)
    test_dir = os.path.abspath(os.path.dirname(results_file))
    if not os.path.exists(test_dir):
        try:
            makedirs_with_parent_perms(test_dir) #NOTE: this will pass up an error exception if it fails
        except OSError as error:
            raise IOError("cannot create test results directory [%s]: %s"%(test_dir, str(error)))

    elif os.path.isfile(test_dir):
        raise Exception("ERROR: cannot run test suite, file is preventing creation of test dir: %s"%test_dir)
    
    print("[ROSUNIT] Outputting test results to " + results_file)
    outstream = open(results_file, 'w')
    outstream.write('<?xml version="1.0" encoding="utf-8"?>\n')
    return XMLTestRunner(stream=outstream)
    
