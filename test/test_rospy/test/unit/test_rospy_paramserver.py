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

import os
import sys
import struct
import unittest
import time
import random
import datetime

from rosgraph.names import make_global_ns, ns_join

## Unit tests for rospy.paramserver module
class TestRospyParamServer(unittest.TestCase):
    
    def test_param_server_cache(self):
        from rospy.impl.paramserver import get_param_server_cache
        ps = get_param_server_cache()
        self.assert_(ps is not None)
        try:
            ps.get('foo')
            self.fail("get should fail on non-existent key")
        except KeyError, e:
            pass
        for i in xrange(0, 10):
            k = 'key-%s%s'%(i, random.randint(0, 1000))
            v = 'value-%s'%random.randint(0, 1000)
            try:
                ps.update(k, v)
                self.fail("update should fail on non-existent key "+k)
            except KeyError, e:
                pass
                
            ps.set(k, v)
            self.assertEquals(v, ps.get(k))
            v = 'value-%s'%random.randint(0, 1000)
            ps.update(k, v)            
            self.assertEquals(v, ps.get(k))
        
            ps.delete(k)
            try:
                ps.get(k)
                self.fail('get should fail on deleted key')
            except KeyError: pass
