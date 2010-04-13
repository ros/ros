#!/usr/bin/env python
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

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import struct
import unittest
import time

class TestRospyValidators(unittest.TestCase):

    def test_ParameterInvalid(self):
        # not really testing anything here other than typos
        from rospy.validators import ParameterInvalid
        self.assert_(isinstance(ParameterInvalid('param'), Exception))
        
    def test_validators(self):
        from rospy.validators import ParameterInvalid
        from rospy.validators import non_empty
        contextes = ['', '/', '/foo']
        for context in contextes:
            valid = ['foo', 1, [1]]
            for v in valid:
                non_empty('param-name')(v, context)
            invalid = ['', 0, []]
            for i in invalid:
                try:
                    non_empty('param-name-foo')(i, context)
                except ParameterInvalid, e:
                    self.assert_('param-name-foo' in str(e))

            from rospy.validators import non_empty_str
            valid = ['foo', 'f', u'f']
            for v in valid:
                non_empty_str('param-name')(v, context)
            invalid = ['', 1, ['foo']]
            for i in invalid:
                try:
                    non_empty_str('param-name-bar')(i, context)
                except ParameterInvalid, e:
                    self.assert_('param-name-bar' in str(e))
                    
            from rospy.validators import not_none

            valid = ['foo', 'f', 1, False, 0, '']
            for v in valid:
                not_none('param-name')(v, context)
            invalid = [None]
            for i in invalid:
                try:
                    not_none('param-name-charlie')(i, context)
                except ParameterInvalid, e:
                    self.assert_('param-name-charlie' in str(e))
                
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_rospy', sys.argv[0], TestRospyValidators, coverage_packages=['rospy.validators'])
