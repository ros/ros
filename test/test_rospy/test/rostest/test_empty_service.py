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
NAME = 'empty_service'

import sys, time
import unittest

import rospy, rostest
from test_rospy.srv import *

EMPTY_SERVICE      = 'empty_service'
EMPTY_RETURN_SERVICE = 'empty_return_service'
EMPTY_REQ_SERVICE  = 'empty_req_service'
EMPTY_RESP_SERVICE = 'empty_resp_service'

FAKE_SECRET = 123456

WAIT_TIMEOUT = 10.0 #s

def handle_empty(req):
    print "Returning empty"
    return EmptySrvResponse()

def handle_return_empty(req):
    "print returning empty list"
    return []

## handle empty request 
def handle_empty_req(req):
    print "Returning fake_secret"
    return EmptyReqSrvResponse(FAKE_SECRET)
## handle empty response 
def handle_empty_resp(req):
    if req.fake_secret == FAKE_SECRET:
        print "Request validated, returning empty"        
        return EmptyRespSrvResponse()
    else:
        print "Request did not validate, returning None"

def empty_service():
    rospy.init_node(NAME)
    s1 = rospy.Service(EMPTY_SERVICE, EmptySrv, handle_empty)
    s2 = rospy.Service(EMPTY_REQ_SERVICE, EmptyReqSrv, handle_empty_req)
    s3 = rospy.Service(EMPTY_RESP_SERVICE, EmptyRespSrv, handle_empty_resp)
    s4 = rospy.Service(EMPTY_RETURN_SERVICE, EmptySrv, handle_return_empty)
    rospy.spin()

class TestEmptyServiceClient(unittest.TestCase):
        
    def _test(self, name, srv, req):
        rospy.wait_for_service(name, WAIT_TIMEOUT)        
        s = rospy.ServiceProxy(name, srv)
        resp = s.call(req)
        self.assert_(resp is not None)
        return resp

    # test that __call__ and s.call() work with no-args on an empty request
    def test_call_empty(self):
        rospy.wait_for_service(EMPTY_REQ_SERVICE, WAIT_TIMEOUT)        
        s = rospy.ServiceProxy(EMPTY_REQ_SERVICE, EmptyReqSrv)
        resp = s()
        self.assertEquals(FAKE_SECRET, resp.fake_secret, 
                          "fake_secret fields is not set as expected")        
        resp = s.call()
        self.assertEquals(FAKE_SECRET, resp.fake_secret, 
                          "fake_secret fields is not set as expected")        
        
    def test_empty(self):
        self._test(EMPTY_SERVICE, EmptySrv, EmptySrvRequest())
    # test that an empty return service handler can return an empty list
    def test_return_empty(self):
        self._test(EMPTY_RETURN_SERVICE, EmptySrv, EmptySrvRequest())
    def test_empty_req(self):
        resp = self._test(EMPTY_REQ_SERVICE, EmptyReqSrv,
                          EmptyReqSrvRequest())
        self.assertEquals(FAKE_SECRET, resp.fake_secret, 
                          "fake_secret fields is not set as expected")        
    def test_empty_resp(self):
        self._test(EMPTY_RESP_SERVICE, EmptyRespSrv,
                   EmptyRespSrvRequest(FAKE_SECRET))
        
if __name__ == '__main__':
    if '--service' in sys.argv:
        empty_service()
    else:
        rostest.run(PKG, 'rospy_empty_service', TestEmptyServiceClient, sys.argv)
