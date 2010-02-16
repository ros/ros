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
import roslib; roslib.load_manifest('test_roslib')

import os
import sys
import unittest

import roslib.masterapi
import rostest

_ID = '/caller_id'
_MASTER_URI = 'http://localhost:12345'

class MasterMock(object):
    """
    Mock for testing Master without using actual master
    """
    
    def __init__(self):
        self.call = None
        self.return_val = None

    def getMasterUri(self, caller_id):
        self.call = ('getMasterUri', caller_id)
        return self.return_val
    
    def getPid(self, caller_id):
        self.call = ('getPid', caller_id)
        return self.return_val
    
    def registerService(self, caller_id, service, service_api, caller_api):
        self.call = ('registerService', caller_id, service, service_api, caller_api)
        return self.return_val
    
    def lookupService(self, caller_id, service):
        self.call = ('lookupService', caller_id, service)
        return self.return_val

    def unregisterService(self, caller_id, service, service_api):
        self.call = ('unregisterService', caller_id, service, service_api)
        return self.return_val

    def registerSubscriber(self, caller_id, topic, topic_type, caller_api):
        self.call = ('registerSubscriber', caller_id, topic, topic_type, caller_api)
        return self.return_val

    def unregisterSubscriber(self, caller_id, topic, caller_api):
        self.call = ('unregisterSubscriber', caller_id, topic, caller_api)
        return self.return_val
    
    def registerPublisher(self, caller_id, topic, topic_type, caller_api):
        self.call = ('registerPublisher', caller_id, topic, topic_type, caller_api)
        return self.return_val
    
    def unregisterPublisher(self, caller_id, topic, caller_api):
        self.call = ('unregisterPublisher', caller_id, topic, caller_api)
        return self.return_val

    def lookupNode(self, caller_id, node_name):
        self.call = ('lookupNode', caller_id, node_name)
        return self.return_val

    def getPublishedTopics(self, caller_id, subgraph):
        self.call = ('getPublishedTopics', caller_id, subgraph)
        return self.return_val
    
    def getSystemState(self, caller_id):
        self.call = ('getSystemState', caller_id)
        return self.return_val

class MasterApiOfflineTest(unittest.TestCase):
  
    def setUp(self):
        self.m = roslib.masterapi.Master(_ID, master_uri = _MASTER_URI)
        # replace xmlrpclib server proxy with mock
        self.m.handle = MasterMock()

    def test_Master(self):
        # test constructor args
        m = roslib.masterapi.Master(_ID, master_uri = 'http://localhost:12345')
        self.assertEquals(_ID, m.caller_id)
        self.assertEquals(_MASTER_URI, m.master_uri)        

        m = roslib.masterapi.Master(_ID)
        self.assertEquals(os.environ['ROS_MASTER_URI'], m.master_uri)

        id = '/some/other/id'
        m = roslib.masterapi.Master(id)
        self.assertEquals(id, m.caller_id)

    def test_getPid(self):
        h = self.m.handle
        r = 1235
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getPid())
        self.assertEquals(('getPid',_ID), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.getPid()
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.getPid()
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass

    def test_lookupService(self):
        h = self.m.handle
        r = 'rosrpc://localhost:12345'
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.lookupService('/bar/service'))
        self.assertEquals(('lookupService',_ID, '/bar/service'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.lookupService('/bar/service')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.lookupService('/bar/service')
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass

    def test_registerService(self):
        h = self.m.handle
        r = 11
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.registerService('/bar/service', 'rosrpc://localhost:9812', 'http://localhost:893'))
        self.assertEquals(('registerService',_ID, '/bar/service', 'rosrpc://localhost:9812', 'http://localhost:893'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.registerService('/bar/service', 'rosrpc://localhost:9812', 'http://localhost:893')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.registerService('/bar/service', 'rosrpc://localhost:9812', 'http://localhost:893')
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass

    def test_unregisterService(self):
        h = self.m.handle
        r = 1
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.unregisterService('/bar/service', 'rosrpc://localhost:9812'))
        self.assertEquals(('unregisterService',_ID, '/bar/service', 'rosrpc://localhost:9812'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.unregisterService('/bar/service', 'rosrpc://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.unregisterService('/bar/service', 'rosrpc://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass
        
    def test_registerSubscriber(self):
        h = self.m.handle
        r = ['http://localhost:1234', 'http://localhost:98127']
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.registerSubscriber('/foo/node', 'std_msgs/String', 'http://localhost:9812'))
        self.assertEquals(('registerSubscriber',_ID, '/foo/node', 'std_msgs/String', 'http://localhost:9812'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.registerSubscriber('/foo/node', 'std_msgs/String', 'http://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.registerSubscriber('/foo/node', 'std_msgs/String', 'http://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass

    def test_unregisterSubscriber(self):
        h = self.m.handle
        r = 1
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.unregisterSubscriber('/foo/node', 'http://localhost:9812'))
        self.assertEquals(('unregisterSubscriber',_ID, '/foo/node', 'http://localhost:9812'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.unregisterSubscriber('/foo/node', 'http://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.unregisterSubscriber('/foo/node', 'http://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass

    def test_registerPublisher(self):
        h = self.m.handle
        r = 5
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.registerPublisher('/foo/node', 'std_msgs/String', 'http://localhost:9812'))
        self.assertEquals(('registerPublisher',_ID, '/foo/node', 'std_msgs/String', 'http://localhost:9812'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.registerPublisher('/foo/node', 'std_msgs/String', 'http://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.registerPublisher('/foo/node', 'std_msgs/String', 'http://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass

    def test_unregisterPublisher(self):
        h = self.m.handle
        r = 10
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.unregisterPublisher('/foo/node', 'http://localhost:9812'))
        self.assertEquals(('unregisterPublisher',_ID, '/foo/node', 'http://localhost:9812'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.unregisterPublisher('/foo/node', 'http://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.unregisterPublisher('/foo/node', 'http://localhost:9812')
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass

    def test_lookupNode(self):
        h = self.m.handle
        r = 'http://localhost:123'
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.lookupNode('/foo/node'))
        self.assertEquals(('lookupNode',_ID, '/foo/node'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.lookupNode('/foo/node')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.lookupNode('/foo/node')
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass
        
    def test_getPublishedTopics(self):
        h = self.m.handle
        r = [ ['foo', 'bar'], ['baz', 'blah'] ]
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getPublishedTopics('/foo'))
        self.assertEquals(('getPublishedTopics',_ID, '/foo'), h.call)
        try:
            h.return_val = (0, '', r)
            self.m.getPublishedTopics('/baz')
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.getPublishedTopics('/bar')            
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass
        
    def test_getSystemState(self):
        h = self.m.handle
        r = [ [], [], [] ]
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getSystemState())
        self.assertEquals(('getSystemState', _ID), h.call)        
        try:
            h.return_val = (0, '', r)
            self.m.getSystemState()
            self.fail("should have thrown")
        except roslib.masterapi.Failure:
            pass
        try:
            h.return_val = (-1, '', r)
            self.m.getSystemState()            
            self.fail("should have thrown")
        except roslib.masterapi.Error:
            pass
        

if __name__ == '__main__':
    rostest.unitrun('test_roslib', 'test_roslib_masterapi_offline', MasterApiOfflineTest, coverage_packages=['roslib.masterapi'])

