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

import rospy
import rospy.service

class MockServiceClass(object):
    _request_class = rospy.AnyMsg
    _response_class = rospy.AnyMsg

class TestRospyService(unittest.TestCase):

    def test_ServiceException(self):
        self.assert_(isinstance(rospy.service.ServiceException(), Exception))
        
    def test_ServiceManager(self):
        class MockService(rospy.service._Service):
            def __init__(self, name, service_class, uri):
                rospy.service._Service.__init__(self, name, service_class)
                self.uri = uri

        from rospy.service import ServiceManager
        sm = ServiceManager()
        self.assertEquals({}, sm.map)
        try:
            sm.lock.acquire()
        finally:
            sm.lock.release()
        self.assertEquals([], sm.get_services())
        sm.unregister_all()
        self.assertEquals([], sm.get_services())

        # test actual registration
        mock = MockService('/serv', MockServiceClass, "rosrpc://uri:1")
        mock2 = MockService('/serv', MockServiceClass, "rosrpc://uri:2")        

        sm.register('/serv', mock)
        self.assertEquals(mock, sm.get_service('/serv'))
        self.assertEquals([('/serv', mock.uri)], sm.get_services())
        try:
            sm.register('/serv', mock2)
            self.fail("duplicate reg should fail")
        except rospy.service.ServiceException: pass
        
        sm.unregister_all()
        self.assertEquals([], sm.get_services())
        
        #  - register two services
        sm.register('/serv', mock)
        sm.unregister('/serv', mock2)
        self.assertEquals(mock, sm.get_service('/serv'))

        sm.register('/serv2', mock2)
        self.assertEquals(mock, sm.get_service('/serv'))
        self.assertEquals(mock2, sm.get_service('/serv2'))
        self.assert_(('/serv', mock.uri) in sm.get_services())
        self.assert_(('/serv2', mock2.uri) in sm.get_services())        
        
        sm.unregister('/serv', mock)
        self.assertEquals([('/serv2', mock2.uri)], sm.get_services())
        sm.unregister('/serv2', mock2)
        self.assertEquals([], sm.get_services())
