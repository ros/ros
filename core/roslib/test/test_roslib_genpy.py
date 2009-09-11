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
# Revision $Id: test_pubsub_order.py 2255 2008-09-29 22:37:08Z sfkwc $

NAME = 'test_genpy'
import roslib; roslib.load_manifest('roslib')

import sys 
import unittest

# this is one of the few cases where its okay to cheat: we
# can't have a circular package dependency on rostest, so
# pretend to be the rostest package
import roslib; roslib.load_manifest('rostest')
import rostest

class TestGenpy(unittest.TestCase):

    def setUp(self):
        pass
        
    ## Test genpy.reduce_pattern
    def test_reduce_pattern(self):
        tests = [
            ('', ''),
            ('hhhh', '4h'),
            ('hhhhi', '4hi'),
            ('hhhhiiiibbb', '4h4i3b'),            
            ('1h2h3h', '1h2h3h'),            
            ('hIi', 'hIi'),
            ('66h', '66h'),
            ('%ss', '%ss'), #don't reduce strings with format chars in them
            ('<I', '<I'),
            ('<11s', '<11s'),            
            ]
        from roslib.genpy import reduce_pattern
        for input, result in tests:
            self.assertEquals(result, reduce_pattern(input))
        
if __name__ == '__main__':
    rostest.unitrun('roslib', NAME, TestGenpy, sys.argv, coverage_packages=['roslib.genpy'])
