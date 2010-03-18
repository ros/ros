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
import roslib; roslib.load_manifest('test_roslib')

import os
import sys
import unittest

import roslib.rosenv
import rostest

class SubArgsTest(unittest.TestCase):
  
    def test_resolve_args(self):
        from roslib.substitution_args import resolve_args, SubstitutionException
        from roslib.packages import get_pkg_dir
        rospy_dir = get_pkg_dir('rospy', required=True)

        context = {'foo': 'bar'}
        
        tests = [
            ('$(find rospy)', rospy_dir),
            ('hello$(find rospy)', 'hello'+rospy_dir),
            ('$(find rospy )', rospy_dir),
            ('$$(find rospy )', '$'+rospy_dir),
            ('$( find rospy )', rospy_dir),
            ('$(find  rospy )', rospy_dir),
            ('$(find rospy)$(find rospy)', rospy_dir+rospy_dir),
            ('$(find rospy)/foo/bar.xml', rospy_dir+os.sep+'foo'+os.sep+'bar.xml'),
            (r'$(find rospy)\foo\bar.xml $(find rospy)\bar.xml', rospy_dir+os.sep+'foo'+os.sep+'bar.xml '+rospy_dir+os.sep+'bar.xml'),
            ('$(find rospy)\\foo\\bar.xml more/stuff\\here', rospy_dir+os.sep+'foo'+os.sep+'bar.xml more/stuff\\here'),
            ('$(env ROS_ROOT)', os.environ['ROS_ROOT']),
            ('$(env ROS_ROOT)', os.environ['ROS_ROOT']),
            ('$(env ROS_ROOT )', os.environ['ROS_ROOT']),
            ('$(optenv ROS_ROOT)', os.environ['ROS_ROOT']),
            ('$(optenv ROS_ROOT)$(optenv ROS_ROOT)', os.environ['ROS_ROOT']+os.environ['ROS_ROOT']),
            ('$(optenv ROS_ROOT alternate text)', os.environ['ROS_ROOT']),
            ('$(optenv NOT_ROS_ROOT)', ''),
            ('$(optenv NOT_ROS_ROOT)more stuff', 'more stuff'),
            ('$(optenv NOT_ROS_ROOT alternate)', 'alternate'),
            ('$(optenv NOT_ROS_ROOT alternate text)', 'alternate text'),

            # #1776
            ('$(anon foo)', 'bar'),
            ('$(anon foo)/baz', 'bar/baz'),
            ('$(anon foo)/baz/$(anon foo)', 'bar/baz/bar'),
            ]
        for arg, val in tests:
            self.assertEquals(val, resolve_args(arg, context=context))

        # more #1776
        r = resolve_args('$(anon foo)/bar')
        self.assert_('/bar' in r)
        self.failIf('$(anon foo)' in r)        
        
            
        # test against strings that should not match
        noop_tests = [
            '$(find rospy', '$find rospy', '', ' ', 'noop', 'find rospy', 'env ROS_ROOT', '$$', ')', '(', '()',
            None, 
            ]
        for t in noop_tests:
            self.assertEquals(t, resolve_args(t))
        failures = [
            '$((find rospy))',  '$(find $rospy)',
            '$(find)', '$(find rospy roslib)', '$(export rospy)',
            '$(env)', '$(env ROS_ROOT alternate)',
            '$(env NOT_SET)',
            '$(optenv)',
            '$(anon)',
            '$(anon foo bar)',            
            ]
        for f in failures:
            try:
                resolve_args(f)
                self.fail("resolve_args(%s) should have failed"%f)
            except SubstitutionException: pass

if __name__ == '__main__':
  rostest.unitrun('test_roslib', 'test_substitution_args', SubArgsTest, coverage_packages=['roslib.substitution_args'])

