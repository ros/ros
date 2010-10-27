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
# Revision $Id$

"""
Library and tools support unit testing in ROS.
"""

# NOTE: while this makes some forward references to conventions used
# in rostest, it does not use rostest itself.
import os
import roslib

def xml_results_file(test_pkg, test_name, is_rostest=False):
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
    test_dir = os.path.join(roslib.rosenv.get_test_results_dir(), test_pkg)
    if not os.path.exists(test_dir):
        try:
            roslib.rosenv.makedirs_with_parent_perms(test_dir)
        except OSError:
            raise IOError("cannot create test results directory [%s]. Please check permissions."%(test_dir))
        
    # #576: strip out chars that would bork the filename
    # this is fairly primitive, but for now just trying to catch some common cases
    for c in ' "\'&$!`/\\':
        if c in test_name:
            test_name = test_name.replace(c, '_')
    if is_rostest:
        return os.path.join(test_dir, 'TEST-rostest__%s.xml'%test_name)
    else:
        return os.path.join(test_dir, 'TEST-%s.xml'%test_name)        
    
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

