# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
# Revision $Id: test_rosservice_command_line_offline.py 6214 2009-09-18 23:38:22Z kwc $

from __future__ import with_statement

NAME = 'test_rosh_impl_topic'
import roslib; roslib.load_manifest('rosh')

import os
import sys 
import unittest
        
import rostest

        
from rosh.impl.namespace import Namespace, NamespaceConfig, Context


class MockConfig(NamespaceConfig):
    def __init__(self):
        NamespaceConfig.__init__(self, Context(), None)
    
class TopicMock(Namespace):
    def __init__(self):
        Namespace.__init__(self, '/', MockConfig())
    def _add_subscriber_callback(self, cb):
        pass
    def _remove_subscriber_callback(self, cb):
        pass

class TestRoshImplTopic(unittest.TestCase):

    def setUp(self):
        pass

    def test__TopicSliceCallback(self):
        from rosh.impl.topic import _TopicSliceCallback

        mock = TopicMock()

        # test empty slices
        x = _TopicSliceCallback(mock, 0, 0, None, 1)
        self.assertEquals([], list(x))
        x(2)
        self.assertEquals([], list(x))
        # test with > 0 empty slices
        x = _TopicSliceCallback(mock, 5, 5, None, 1)
        self.assertEquals([], list(x))
        x(2)
        self.assertEquals([], list(x))

        # negative step should fail
        try:
            x = _TopicSliceCallback(mock, 0, 10, 0, 1)
            self.fail("should have raised")
        except ValueError:
            pass

        # test that if we are already satisfied, call does not affect anything
        x = _TopicSliceCallback(mock, 0, 1, None, 2)
        x(3)
        self.assertEquals([2], list(x))        

        # test call logic with no step and last msg
        x = _TopicSliceCallback(mock, 0, 10, None, 1)
        l = [1]
        for i in xrange(1, 10):
            x(i)
            l.append(i)
        # - just check whatever Python says is correct
        self.assertEquals(l[0:10], list(x))
        
        # test call logic with no step and no last msg
        x = _TopicSliceCallback(mock, 0, 10, None, None)
        l = [0] # put in a fake start msg to represent the last msg that we aren't including
        for i in xrange(1, 10):
            x(i)
            l.append(i)
        # - shift start idx by 1 as last msg is not present
        self.assertEquals(l[1:10], list(x))
        
        # test call logic with step and last msg
        for step in xrange(1, 11):
            x = _TopicSliceCallback(mock, 0, 10, step, 1)
            l = [1]
            for i in xrange(1, 10):
                x(i)
                l.append(i)
            # - just check whatever Python says is correct
            self.assertEquals(l[0:10:step], list(x), "failed on step %s"%step)

        # test call logic with step and no last msg
        for step in xrange(1, 11):
            x = _TopicSliceCallback(mock, 0, 10, step, None)
            l = [0] # represents last msg that isn't present
            for i in xrange(1, 10):
                x(i)
                l.append(i)
            self.assertEquals(l[1:10:step], list(x))

        # test call logic with step and no last msg
        for step in xrange(1, 11):
            x = _TopicSliceCallback(mock, 5, 10, step, None)
            l = [0]
            for i in xrange(1, 10):
                x(i)
                l.append(i)
            self.assertEquals(l[5:10:step], list(x))
            
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('rosh', sys.argv[0], TestRoshImplTopic, coverage_packages=['rosh.impl.topic'])
