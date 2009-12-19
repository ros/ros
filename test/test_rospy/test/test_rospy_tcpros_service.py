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
# Revision $Id: test_rospy_api.py 4898 2009-06-17 01:31:18Z sfkwc $

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import socket
import struct
import unittest
import time

class FakeSocket(object):
    def __init__(self):
        self.data = ''
        self.sockopt = None
    def fileno(self):
        # fool select logic by giving it stdout fileno
        return 1
    def setblocking(self, *args):
        pass
    def setsockopt(self, *args):
        self.sockopt = args
    def send(self, d):
        self.data = self.data+d
        return len(d)
    def sendall(self, d):
        self.data = self.data+d
    def close(self):
        pass

# test service implementation
class TestRospyTcprosService(unittest.TestCase):

    def test_convert_return_to_response(self):
        import rospy
        from rospy.tcpros_service import convert_return_to_response
        from test_ros.srv import AddTwoIntsResponse

        cls = AddTwoIntsResponse
        v = cls(3)
        
        # test various ways that a user could reasonable return a
        # value for a single-arg message. This is actually our hardest
        # case.
        self.assertEquals(v, convert_return_to_response(v, cls))
        self.assertEquals(v, convert_return_to_response(3, cls))
        self.assertEquals(v, convert_return_to_response((3), cls))        
        self.assertEquals(v, convert_return_to_response([3], cls))        
        self.assertEquals(v, convert_return_to_response({'sum':3}, cls))
        for bad in [[1, 2, 3], {'fake': 1}]:
            try:
                convert_return_to_response(bad, cls)
                self.fail("should have raised: %s"%str(bad))
            except rospy.ServiceException:
                pass

        # test multi-arg services
        from test_rospy.srv import MultipleAddTwoIntsResponse
        cls = MultipleAddTwoIntsResponse
        v = cls(1, 2)
        self.assertEquals(v, convert_return_to_response(v, cls))
        self.assertEquals(v, convert_return_to_response((1, 2), cls))        
        self.assertEquals(v, convert_return_to_response([1, 2], cls))        
        self.assertEquals(v, convert_return_to_response({'ab':1, 'cd': 2}, cls))
        for bad in [1, AddTwoIntsResponse(), [1, 2, 3], {'fake': 1}]:
            try:
                convert_return_to_response(bad, cls)
                self.fail("should have raised: %s"%str(bad))
            except rospy.ServiceException:
                pass

        # test response with single, array field
        from test_rospy.srv import ListReturnResponse
        cls = ListReturnResponse
        v = cls([1, 2, 3])
        self.assertEquals(v, convert_return_to_response(v, cls))
        self.assertEquals(v, convert_return_to_response(((1, 2, 3),), cls))        
        self.assertEquals(v, convert_return_to_response(([1, 2, 3],), cls))
        self.assertEquals(v, convert_return_to_response([[1, 2, 3]], cls))        
        self.assertEquals(v, convert_return_to_response({'abcd':[1,2,3]}, cls))
        for bad in [[1, 2, 3], {'fake': 1}]:
            try:
                convert_return_to_response(bad, cls)
                self.fail("should have raised: %s"%str(bad))
            except rospy.ServiceException:
                pass

        # test with empty response
        from test_rospy.srv import EmptySrvResponse
        cls = EmptySrvResponse
        v = cls()
        # - only valid return values are None and a response instance
        self.assertEquals(v, convert_return_to_response(v, cls))
        self.assertEquals(v, convert_return_to_response(None, cls))
        
        # #2185: currently empty does not do any checking whatsoever,
        # disabling this test as it is not convert()s fault
        if 0:
            for bad in [1, AddTwoIntsResponse(), [1, 2, 3], {'fake': 1}]:
                try:
                    convert_return_to_response(bad, cls)
                    self.fail("should have raised: %s"%str(bad))
                except rospy.ServiceException:
                    pass
        
    def test_service_connection_handler(self):
        import test_rospy.srv
        from rospy.registration import get_service_manager        
        import rospy.service

        sock = FakeSocket()
        from rospy.tcpros_service import service_connection_handler
        client_addr = '10.0.0.1'

        # check error conditions on missing headers
        self.assert_("Missing" in service_connection_handler(sock, client_addr, {}))
        header = { 'service' : '/service', 'md5sum': '*', 'callerid': '/bob' }
        for k in header:
            c = header.copy()
            del c[k]
            msg = service_connection_handler(sock, client_addr, c)
            self.assert_("Missing" in msg, str(c) + msg)
            self.assert_(k in msg, msg)

        # check error condition on invalid service
        header['service'] = '/doesnotexist'
        msg = service_connection_handler(sock, client_addr, header)
        self.assert_('is not a provider' in msg, msg)

        # check invalid md5sums

        name = '/service'
        sm = get_service_manager()
        fake_service = \
            rospy.service._Service(name, test_rospy.srv.EmptySrv)
        sm.register(name, fake_service)
        
        header['service'] = name
        header['md5sum'] = 'X'

        msg = service_connection_handler(sock, client_addr, header)
        self.assert_('md5sums do not match' in msg, msg)
        
        
    def test_TCPROSServiceClient(self):
        import rospy.transport
        from rospy.tcpros_service import TCPROSServiceClient        
        import test_rospy.srv

        callerid = 'test_TCPROSServiceClient'
        import rospy.names
        rospy.names._set_caller_id(callerid)

        #name, pub_data_class
        name = 'name-%s'%time.time()
        srv_data_class = test_rospy.srv.EmptySrv
        p = TCPROSServiceClient(name, srv_data_class)
        self.assertEquals(name, p.resolved_name)
        self.assertEquals(rospy.transport.BIDIRECTIONAL, p.direction)
        self.assertEquals(srv_data_class, p.service_class)
        self.assert_(p.buff_size > -1)

        p = TCPROSServiceClient(name, srv_data_class, buff_size=1)
        self.assert_(1, p.buff_size)

        fields = p.get_header_fields()
        self.assertEquals(name, fields['service'])
        self.assertEquals(srv_data_class._md5sum, fields['md5sum'])

        # test custom headers
        headers = {'sessionid': '123456', 'persistent': '1'}
        p = TCPROSServiceClient(name, srv_data_class, headers=headers)
        self.assertEquals('123456', p.get_header_fields()['sessionid'])
        self.assertEquals('1', p.get_header_fields()['persistent'])
        # - make sure builtins are still there
        self.assertEquals(name, fields['service'])
        self.assertEquals(srv_data_class._md5sum, fields['md5sum'])
        

    def test_TCPService(self):
        import rospy.transport
        from rospy.tcpros_service import TCPService        
        import test_rospy.srv

        callerid = 'test_TCPService'
        import rospy.names
        rospy.names._set_caller_id(callerid)

        #name, pub_data_class
        name = 'name-%s'%time.time()
        srv_data_class = test_rospy.srv.EmptySrv
        p = TCPService(name, srv_data_class)
        self.assertEquals(name, p.resolved_name)
        self.assertEquals(rospy.transport.BIDIRECTIONAL, p.direction)
        self.assertEquals(srv_data_class, p.service_class)
        self.assert_(p.buff_size > -1)

        p = TCPService(name, srv_data_class, buff_size=1)
        self.assert_(1, p.buff_size)

        fields = p.get_header_fields()
        self.assertEquals(name, fields['service'])
        self.assertEquals(srv_data_class._md5sum, fields['md5sum'])
        self.assertEquals(srv_data_class._type, fields['type'])        
        
            
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_rospy', 'test_rospy_tcpros_service', TestRospyTcprosService, coverage_packages=['rospy.tcpros_service'])
