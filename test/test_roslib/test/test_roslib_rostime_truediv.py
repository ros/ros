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

from __future__ import division

import roslib; roslib.load_manifest('test_roslib')

import os
import sys
import unittest

import rosunit

class RostimeTruedivTest(unittest.TestCase):

  def test_Duration(self):
      import roslib.rostime
      Duration = roslib.rostime.Duration

      # See #3667 as well as PEP 238
      d = Duration(13, 500000000)
      to_sec = d.to_sec()
      self.assertEquals(Duration(to_sec / 2.), d/2)

      # Test div
      self.assertEquals(Duration(4), Duration(8) / 2)
      self.assertEquals(Duration(4), Duration(8) / 2.)      
      self.assertEquals(Duration(4), Duration(8) // 2)      
      self.assertEquals(Duration(4), Duration(8) // 2.)      
      self.assertEquals(Duration(4), Duration(9) // 2)      
      self.assertEquals(Duration(4), Duration(9) // 2.)      
      self.assertEquals(Duration(4, 2), Duration(8, 4) / 2)
      v = Duration(4, 2) - (Duration(8, 4) / 2.)
      self.assert_(abs(v.to_nsec()) < 100)            
      
      self.assertEquals(Duration(4, 2), Duration(8, 4) // 2)
      self.assertEquals(Duration(4, 2), Duration(9, 5) // 2)
      v = Duration(4, 2) - (Duration(9, 5) // 2.)
      self.assert_(abs(v.to_nsec()) < 100)                  
      
if __name__ == '__main__':
    rosunit.unitrun('test_roslib', 'test_rostime_truediv', RostimeTruedivTest, coverage_packages=['roslib.rostime'])

