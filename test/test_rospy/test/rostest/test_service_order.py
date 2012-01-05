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

## Integration test for empty services to test serializers
## and transport

PKG = 'test_rospy'

import sys, time
import unittest

import rospy, rostest
from test_rospy.srv import *

SERVICE_BEFORE = 'service_order_before'
SERVICE_AFTER  = 'service_order_after'

FAKE_SECRET = 123456

WAIT_TIMEOUT = 10.0 #s

def handle_empty_req(req):
    print "Returning fake_secret"
    return EmptyReqSrvResponse(FAKE_SECRET)

def service_before():
    s = rospy.Service(SERVICE_BEFORE, EmptyReqSrv, handle_empty_req)
    rospy.init_node('service_before')
    rospy.spin()

# #530: verify that init_node can occur after service declarations
def service_after():
    rospy.init_node('service_after')
    s = rospy.Service(SERVICE_AFTER, EmptyReqSrv, handle_empty_req)
    rospy.spin()

class TestServiceOrder(unittest.TestCase):
        
    def _test(self, name, srv, req):
        rospy.wait_for_service(name, WAIT_TIMEOUT)        
        s = rospy.ServiceProxy(name, srv)
        resp = s.call(req)
        self.assert_(resp is not None)
        return resp
    def test_before(self):
        resp = self._test(SERVICE_BEFORE, EmptyReqSrv,
                          EmptyReqSrvRequest())
        self.assertEquals(FAKE_SECRET, resp.fake_secret, 
                          "fake_secret fields is not set as expected")        
    def test_after(self):
        resp = self._test(SERVICE_AFTER, EmptyReqSrv,
                          EmptyReqSrvRequest())
        self.assertEquals(FAKE_SECRET, resp.fake_secret, 
                          "fake_secret fields is not set as expected")        
        
if __name__ == '__main__':
    if '--before' in sys.argv:
        service_before()
    elif '--after' in sys.argv:
        service_after()
    else:
        rostest.run(PKG, 'rospy_service_decl_order', TestServiceOrder, sys.argv)
