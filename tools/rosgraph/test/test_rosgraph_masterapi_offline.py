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

import os
import sys
import unittest

import rosgraph.masterapi

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

    def getUri(self, caller_id):
        self.call = ('getUri', caller_id)
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

    def getTopicTypes(self, caller_id):
        self.call = ('getTopicTypes', caller_id)
        return self.return_val
    
    def getSystemState(self, caller_id):
        self.call = ('getSystemState', caller_id)
        return self.return_val

    def getParam(self, caller_id, p):
        self.call = ('getParam', caller_id, p)
        return self.return_val

    def hasParam(self, caller_id, p):
        self.call = ('hasParam', caller_id, p)
        return self.return_val

    def deleteParam(self, caller_id, p):
        self.call = ('deleteParam', caller_id, p)
        return self.return_val

    def searchParam(self, caller_id, p):
        self.call = ('searchParam', caller_id, p)
        return self.return_val

    def setParam(self, caller_id, p, v):
        self.call = ('setParam', caller_id, p, v)
        return self.return_val

    def subscribeParam(self, caller_id, api, p):
        self.call = ('subscribeParam', caller_id, api, p)
        return self.return_val

    def unsubscribeParam(self, caller_id, api, p):
        self.call = ('unsubscribeParam', caller_id, api, p)
        return self.return_val
    
    def getParamNames(self, caller_id):
        self.call = ('getParamNames', caller_id)
        return self.return_val
    
class MasterApiOfflineTest(unittest.TestCase):
  
    def setUp(self):
        self.m = rosgraph.masterapi.Master(_ID, master_uri = _MASTER_URI)
        # replace xmlrpclib server proxy with mock
        self.m.handle = MasterMock()

    def throw_failure(self, attr, args, ret_val):
        self.m.handle.return_val = ret_val
        f = getattr(self.m, attr)
        try:
            f(*args)
            self.fail("[%s] should have thrown Failure with args [%s], ret_val [%s]"%(attr, str(args), str(ret_val)))
        except rosgraph.masterapi.Failure:
            pass

    def throw_error(self, attr, args, ret_val):
        self.m.handle.return_val = ret_val
        f = getattr(self.m, attr)
        try:
            f(*args)
            self.fail("[%s] should have thrown Error with args [%s], ret_val [%s]"%(attr, str(args), str(ret_val)))
        except rosgraph.masterapi.Error:
            pass
        
    def test_Master(self):
        # test constructor args
        m = rosgraph.masterapi.Master(_ID, master_uri = 'http://localhost:12345')
        self.assertEquals(_ID, m.caller_id)
        self.assertEquals(_MASTER_URI, m.master_uri)        

        reset_uri = False
        if 'ROS_MASTER_URI' not in os.environ:
            os.environ['ROS_MASTER_URI'] = 'http://localhost:21311'

        try:
            m = rosgraph.masterapi.Master(_ID)
            self.assertEquals(os.environ['ROS_MASTER_URI'], m.master_uri)

            id = '/some/other/id'
            m = rosgraph.masterapi.Master(id)
            self.assertEquals(id, m.caller_id)
        finally:
            if reset_uri:
                del os.environ['ROS_MASTER_URI']
                
    def test_getPid(self):
        h = self.m.handle
        r = 1235
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getPid())
        self.assertEquals(('getPid',_ID), h.call)
        self.throw_failure('getPid', (), (0, '', r))
        self.throw_error('getPid', (), (-1, '', r))

    def test_getPid(self):
        h = self.m.handle
        r = 'http://foo:1234'
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getUri())
        self.assertEquals(('getUri',_ID), h.call)
        self.throw_failure('getUri', (), (0, '', r))
        self.throw_error('getUri', (), (-1, '', r))

    def test_lookupService(self):
        h = self.m.handle
        r = 'rosrpc://localhost:12345'
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.lookupService('/bar/service'))
        self.assertEquals(('lookupService',_ID, '/bar/service'), h.call)
        self.throw_failure('lookupService', ('/bar/service',), (0, '', r))
        self.throw_error('lookupService', ('/bar/service',), (-1, '', r))

    def test_registerService(self):
        h = self.m.handle
        r = 11
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.registerService('/bar/service', 'rosrpc://localhost:9812', 'http://localhost:893'))
        self.assertEquals(('registerService',_ID, '/bar/service', 'rosrpc://localhost:9812', 'http://localhost:893'), h.call)
        args = ('/bar/service', 'rosrpc://localhost:9812', 'http://localhost:893')
        self.throw_failure('registerService', args, (0, '', r))
        self.throw_error('registerService', args, (-1, '', r))

    def test_unregisterService(self):
        h = self.m.handle
        r = 1
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.unregisterService('/bar/service', 'rosrpc://localhost:9812'))
        self.assertEquals(('unregisterService',_ID, '/bar/service', 'rosrpc://localhost:9812'), h.call)
        args = ('/bar/service', 'rosrpc://localhost:9812')
        self.throw_failure('unregisterService', args, (0, '', r))
        self.throw_error('unregisterService', args, (-1, '', r))
        
    def test_registerSubscriber(self):
        h = self.m.handle
        r = ['http://localhost:1234', 'http://localhost:98127']
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.registerSubscriber('/foo/node', 'std_msgs/String', 'http://localhost:9812'))
        self.assertEquals(('registerSubscriber',_ID, '/foo/node', 'std_msgs/String', 'http://localhost:9812'), h.call)
        args = ('/foo/node', 'std_msgs/String', 'http://localhost:9812')
        self.throw_failure('registerSubscriber', args, (0, '', r))
        self.throw_error('registerSubscriber', args, (-1, '', r))

    def test_unregisterSubscriber(self):
        h = self.m.handle
        r = 1
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.unregisterSubscriber('/foo/node', 'http://localhost:9812'))
        self.assertEquals(('unregisterSubscriber',_ID, '/foo/node', 'http://localhost:9812'), h.call)
        args = ('/foo/node', 'http://localhost:9812')
        self.throw_failure('unregisterSubscriber', args, (0, '', r))
        self.throw_error('unregisterSubscriber', args, (-1, '', r))

    def test_registerPublisher(self):
        h = self.m.handle
        r = 5
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.registerPublisher('/foo/node', 'std_msgs/String', 'http://localhost:9812'))
        self.assertEquals(('registerPublisher',_ID, '/foo/node', 'std_msgs/String', 'http://localhost:9812'), h.call)
        args = ('/foo/node', 'std_msgs/String', 'http://localhost:9812')
        self.throw_failure('registerPublisher', args, (0, '', r))
        self.throw_error('registerPublisher', args, (-1, '', r))

    def test_unregisterPublisher(self):
        h = self.m.handle
        r = 10
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.unregisterPublisher('/foo/node', 'http://localhost:9812'))
        self.assertEquals(('unregisterPublisher',_ID, '/foo/node', 'http://localhost:9812'), h.call)
        args = ('/foo/node', 'http://localhost:9812')
        self.throw_failure('unregisterPublisher', args, (0, '', r))
        self.throw_error('unregisterPublisher', args, (-1, '', r))

    def test_lookupNode(self):
        h = self.m.handle
        r = 'http://localhost:123'
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.lookupNode('/foo/node'))
        self.assertEquals(('lookupNode',_ID, '/foo/node'), h.call)
        args = ('/foo/node',)
        self.throw_failure('lookupNode', args, (0, '', r))
        self.throw_error('lookupNode', args, (-1, '', r))
        
    def test_getPublishedTopics(self):
        h = self.m.handle
        r = [ ['foo', 'bar'], ['baz', 'blah'] ]
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getPublishedTopics('/foo'))
        self.assertEquals(('getPublishedTopics',_ID, '/foo'), h.call)
        args = ('/baz',)
        self.throw_failure('getPublishedTopics', args, (0, '', r))
        self.throw_error('getPublishedTopics', args, (-1, '', r))
        
    def test_getTopicTypes(self):
        h = self.m.handle
        r = [ ['/foo', 'std_msgs/String'], ['/baz', 'std_msgs/Int32'] ]
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getTopicTypes())
        self.assertEquals(('getTopicTypes',_ID), h.call)
        self.throw_failure('getTopicTypes', (), (0, '', r))
        self.throw_error('getTopicTypes', (), (-1, '', r))
        

    def test_getSystemState(self):
        h = self.m.handle
        r = [ [], [], [] ]
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getSystemState())
        self.assertEquals(('getSystemState', _ID), h.call)        
        self.throw_failure('getSystemState', (), (0, '', r))
        self.throw_error('getSystemState', (), (-1, '', r))

    ################################################################################
    # PARAM SERVER API TESTS

    def test_getParam(self):
        h = self.m.handle
        r = 1
        h.return_val = (1, '', r)
        p = '/test_param'
        self.assertEquals(r, self.m.getParam(p))
        self.assertEquals(('getParam', _ID, p), h.call)
        args = (p,)
        self.throw_failure('getParam', args, (0, '', r))
        self.throw_error('getParam', args, (-1, '', r))

    def test_getParamNames(self):
        h = self.m.handle
        r = [ '/foo' ]
        h.return_val = (1, '', r)
        self.assertEquals(r, self.m.getParamNames())
        self.assertEquals(('getParamNames', _ID), h.call)        
        self.throw_failure('getParamNames', (), (0, '', r))
        self.throw_error('getParamNames', (), (-1, '', r))
        
    def test_hasParam(self):
        h = self.m.handle
        r = True
        h.return_val = (1, '', r)
        p = '/test_param'
        self.assertEquals(r, self.m.hasParam(p))
        self.assertEquals(('hasParam', _ID, p), h.call)
        self.throw_failure('hasParam', (p,), (0, '', r))
        self.throw_error('hasParam', (p,), (-1, '', r))

    def test_searchParam(self):
        h = self.m.handle
        r = '/foo'
        h.return_val = (1, '', r)
        p = '/test_param'
        self.assertEquals(r, self.m.searchParam(p))
        self.assertEquals(('searchParam', _ID, p), h.call)
        self.throw_failure('searchParam', (p,), (0, '', r))
        self.throw_error('searchParam', (p,), (-1, '', r))

    def test_deleteParam(self):
        h = self.m.handle
        r = '/foo'
        h.return_val = (1, '', r)
        p = '/test_param'
        self.assertEquals(r, self.m.deleteParam(p))
        self.assertEquals(('deleteParam', _ID, p), h.call)        
        self.throw_failure('deleteParam', (p,), (0, '', r))
        self.throw_error('deleteParam', (p,), (-1, '', r))

    def test_is_online(self):
        self.failIf(rosgraph.masterapi.is_online(master_uri="http://fake:12345"))

        self.m.handle.return_val = (1, '', 1235)
        self.assert_(self.m.is_online())
        
    def test_subscribeParam(self):
        h = self.m.handle
        r = 1
        h.return_val = (1, '', r)
        args = ('http://bar:12345', '/test_param')
        self.assertEquals(r, self.m.subscribeParam(*args))
        self.assertEquals(('subscribeParam', _ID, args[0], args[1]), h.call)
        self.throw_failure('subscribeParam', args, (0, '', r))
        self.throw_error('subscribeParam', args, (-1, '', r))

    def test_unsubscribeParam(self):
        h = self.m.handle
        r = 1
        h.return_val = (1, '', r)
        args = ('http://bar:12345', '/test_param')
        self.assertEquals(r, self.m.unsubscribeParam(*args))
        self.assertEquals(('unsubscribeParam', _ID, args[0], args[1]), h.call)
        self.throw_failure('unsubscribeParam', args, (0, '', r))
        self.throw_error('unsubscribeParam', args, (-1, '', r))

    def test_setParam(self):
        h = self.m.handle
        r = 1
        h.return_val = (1, '', r)
        args = ('/test_set_param', 'foo')
        self.assertEquals(r, self.m.setParam(*args))
        self.assertEquals(('setParam', _ID, args[0], args[1]), h.call)
        self.throw_failure('setParam', args, (0, '', r))
        self.throw_error('setParam', args, (-1, '', r))
