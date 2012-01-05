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
from cStringIO import StringIO
import time
import random
import math

from genpy import SerializationError
        
class TestGensrvPy(unittest.TestCase):

    ## Utility for testing roundtrip serialization
    ## @param orig Message to test roundtrip serialization of
    ## @param blank Uninitialized instance of message to deserialize into
    ## @param float bool: if True, use almostEquals instead of equals
    ## comparison. This variant assumes only data field is named
    ## 'data'
    def _test_ser_deser(self, orig, blank, float=False):
        b = StringIO()
        orig.serialize(b)
        blank.deserialize(b.getvalue())
        if not float:
            self.assertEquals(orig, blank, str(orig)+" != "+str(blank))
        else:
            self.assertAlmostEquals(orig.data, blank.data, 5)

    ## #2133/2139
    def test_test_rospy_TransitiveImport(self):
        from test_rospy.srv import TransitiveSrvRequest
        m = TransitiveSrvRequest()
        # invoking serialize should be enough to expose issue. The bug
        # was that genmsg_py was failing to include the imports of
        # embedded messages. Because messages are flattened, this
        # causes ImportErrors.
        self._test_ser_deser(m, TransitiveSrvRequest())        
