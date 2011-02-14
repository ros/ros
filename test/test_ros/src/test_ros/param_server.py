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
# Revision $Id: test_embed_msg.py 1986 2008-08-26 23:57:56Z sfkwc $

## Integration test for empty services to test serializers
## and transport

import sys
import unittest
import xmlrpclib
import math
import datetime
import random
import traceback

import rostest
from roslib.names import make_global_ns, ns_join

from rosclient import TestRosClient

HAS_PARAM = True

## Parameter Server API Test Cases. These tests are individually
## enabled by param server test nodes. Each test assumes a fresh param
## server, so we cannot run within the same roslaunch/rostest session.
class ParamServerTestCase(TestRosClient):
    
    def _setParam(self, ctx, myState, testVals, master):
        ctx = make_global_ns(ctx)
        for type, vals in testVals:
            try:
                callerId = ns_join(ctx, "node")
                count = 0
                for val in vals:
                    key = "%s-%s"%(type,count)
                    #print "master.setParam(%s,%s)"%(callerId, key)
                    master.setParam(callerId, key, val)
                    self.assert_(self.apiSuccess(master.hasParam(callerId, key)))
                    trueKey = ns_join(ctx, key)
                    myState[trueKey] = val
                    count += 1
            except Exception, e:
                assert "getParam failed on type[%s], val[%s]"%(type,val)
        #self._checkParamState(myState)

    def _checkParamState(self, myState):
        master = self.master
        callerId = 'master' #validate from root 
        for (k, v) in myState.iteritems():
            assert self.apiSuccess(master.hasParam(callerId, k))
            #print "verifying parameter %s"%k
            try:
                v2 = self.apiSuccess(master.getParam(callerId, k))
            except:
                raise Exception("Exception raised while calling master.getParam(%s,%s): %s"%(callerId, k, traceback.format_exc()))
            if isinstance(v2, xmlrpclib.DateTime):
                self.assertEquals(xmlrpclib.DateTime(v), v2, "[%s]: %s != %s, %s"%(k, v, v2, v2.__class__))
            elif type(v2) == float:
                self.assertAlmostEqual(v, v2, 3, "[%s]: %s != %s, %s"%(k, v, v2, v2.__class__))
            else:
                self.assertEquals(v, v2)
        paramNames = myState.keys()
        remoteParamNames = self.apiSuccess(master.getParamNames(callerId))
        # filter out the roslaunch params like run id and roslaunch/, which are always set
        remoteParamNames = [p for p in remoteParamNames if not p in ['/run_id', '/rosdistro', '/rosversion']]
        remoteParamNames = [p for p in remoteParamNames if not p.startswith('/roslaunch/')]

        assert not set(paramNames) ^ set(remoteParamNames), "parameter server keys do not match local: %s"%(set(paramNames)^set(remoteParamNames))


    # _testHasParam: test hasParam API
    def _testHasParam(self):
        master = self.master
        caller_id = '/node'
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/new_param')))
        self.apiSuccess(master.setParam(caller_id, '/new_param', 1))
        self.assert_(self.apiSuccess(master.hasParam(caller_id, '/new_param')))
        # test with relative-name resolution
        self.assert_(self.apiSuccess(master.hasParam(caller_id, 'new_param')))

        # test with param in sub-namespace
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/sub/sub2/new_param2')))
        # - verify that parameter tree does not exist yet (#587)
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/sub/sub2/')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/sub/sub2')))        
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/sub/')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/sub')))                        
        self.apiSuccess(master.setParam(caller_id, '/sub/sub2/new_param2', 1))
        self.assert_(self.apiSuccess(master.hasParam(caller_id, '/sub/sub2/new_param2')))
        # - verify that parameter tree now exists (#587)
        self.assert_(self.apiSuccess(master.hasParam(caller_id, '/sub/sub2/')))
        self.assert_(self.apiSuccess(master.hasParam(caller_id, '/sub/sub2')))        
        self.assert_(self.apiSuccess(master.hasParam(caller_id, '/sub/')))
        self.assert_(self.apiSuccess(master.hasParam(caller_id, '/sub')))                
        # test with relative-name resolution
        self.assert_(self.apiSuccess(master.hasParam(caller_id, 'sub/sub2/new_param2')))
        self.assert_(self.apiSuccess(master.hasParam('/sub/node', 'sub2/new_param2')))
        self.assert_(self.apiSuccess(master.hasParam('/sub/sub2/node', 'new_param2')))
        self.assert_(self.apiSuccess(master.hasParam('/sub/node', 'sub2')))
        self.assert_(self.apiSuccess(master.hasParam('/node', 'sub')))           

    # testSearchParam: test upwards-looking parameter search
    def _testSearchParam(self):
        master = self.master
        caller_id = '/node'
        # vals are mostly identical, save some randomness. we want
        # identical structure in order to stress lookup rules
        val1 = { 'level1_p1': random.randint(0, 10000),
                 'level1_p2' : { 'level2_p2': random.randint(0, 10000) }}
        val2 = { 'level1_p1': random.randint(0, 10000),
                 'level1_p2' : { 'level2_p2': random.randint(0, 10000) }}
        val3 = { 'level1_p1': random.randint(0, 10000),
                 'level1_p2' : { 'level2_p2': random.randint(0, 10000) }}
        val4 = { 'level1_p1': random.randint(0, 10000),
                 'level1_p2' : { 'level2_p2': random.randint(0, 10000) }}
        full_dict = {}

        # set the val parameter at three levels so we can validate search
        caller_id = '/root'
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/param')))
        self.apiSuccess(master.setParam(caller_id, '/param', val1))
        # - test param 
        self.assertEquals('/param', self.apiSuccess(master.searchParam(caller_id, 'param')))

        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/level1/param')))
        self.apiSuccess(master.setParam(caller_id, '/level1/param', val2))
        self.assertEquals(val2, self.apiSuccess(master.getParam(caller_id, '/level1/param')))
        # - test search param 
        self.assertEquals('/param',
                          self.apiSuccess(master.searchParam(caller_id, 'param')))
        self.assertEquals('/level1/param',
                          self.apiSuccess(master.searchParam('/level1/node', 'param')))

        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/level1/level2/param')))
        self.apiSuccess(master.setParam(caller_id, '/level1/level2/param', val3))
        # - test search param 
        self.assertEquals('/param',
                          self.apiSuccess(master.searchParam(caller_id, 'param')))
        self.assertEquals('/level1/param',
                          self.apiSuccess(master.searchParam('/level1/node', 'param')))
        self.assertEquals('/level1/level2/param',
                          self.apiSuccess(master.searchParam('/level1/level2/node', 'param')))
        self.assertEquals('/level1/level2/param',
                          self.apiSuccess(master.searchParam('/level1/level2/level3/level4/node', 'param')))
        
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/level1/level2/level3/level4/param')))
        self.apiSuccess(master.setParam(caller_id, '/level1/level2/level3/level4/param', val4))
        # - test search param 
        self.assertEquals('/param',
                          self.apiSuccess(master.searchParam(caller_id, 'param')))
        self.assertEquals('/level1/param',
                          self.apiSuccess(master.searchParam('/level1/node', 'param')))
        self.assertEquals('/level1/level2/param',
                          self.apiSuccess(master.searchParam('/level1/level2/node', 'param')))
        self.assertEquals('/level1/level2/param',
                          self.apiSuccess(master.searchParam('/level1/level2/level3/node', 'param')))
        self.assertEquals('/level1/level2/level3/level4/param',
                          self.apiSuccess(master.searchParam('/level1/level2/level3/level4/node', 'param')))

        # test completely different hierarchy, should go to top
        self.assertEquals('/param', self.apiSuccess(master.searchParam('/not/level1/level2/level3/level4/node', 'param')))
        # test looking for param/sub_param
        tests = [('/param', '/not/level1/level2/level3/level4/node'),
                 ('/level1/param', '/level1/node'),
                 ('/level1/param', '/level1/notlevel2/notlevel3/node'),
                 ('/level1/level2/param', '/level1/level2/node'),
                 ('/level1/level2/param', '/level1/level2/level3/node'),
                 ('/level1/level2/param', '/level1/level2/notlevel3/notlevel3/node'),
                 ('/level1/level2/level3/level4/param', '/level1/level2/level3/level4/node'),
                 ('/level1/level2/level3/level4/param', '/level1/level2/level3/level4/l5/l6/node'),
                 ]
        for pbase, caller_id in tests:
            self.assertEquals(pbase + '/level1_p1',
                              self.apiSuccess(master.searchParam(caller_id, 'param/level1_p1')))
            key = pbase+'/level1_p2/level2_p2'
            self.assertEquals(key,
                              self.apiSuccess(master.searchParam(caller_id, 'param/level1_p2/level2_p2')))
            # delete the sub key and repeat, should get the same result as searchParam does partial matches
            # - we may have already deleted the parameter in a previous iteration, so make sure
            if self.apiSuccess(master.hasParam(caller_id, key)):
                self.apiSuccess(master.deleteParam(caller_id, key))
            self.assertEquals(key,
                              self.apiSuccess(master.searchParam(caller_id, 'param/level1_p2/level2_p2')))
            # to make sure that it didn't work spuriously, search for non-existent key
            self.assertEquals(pbase + '/non_existent',
                              self.apiSuccess(master.searchParam(caller_id, 'param/non_existent')))
            self.assertEquals(pbase + '/level1_p2/non_existent',
                              self.apiSuccess(master.searchParam(caller_id, 'param/level1_p2/non_existent')))


    ## remove common keys that roslaunch places on param server
    def _filterDict(self, d):
        for k in ['run_id', 'roslaunch', 'rosversion', 'rosdistro']:
            if k in d:
                del d[k]
        return d
        
    # testGetParam: test basic getParam behavior. Value encoding verified separately by testParamValues
    def _testGetParam(self):
        master = self.master
        caller_id = '/node'
        val = random.randint(0, 10000)

        full_dict = {}
        
        # very similar to has param sequence
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/new_param')))
        self.apiSuccess(master.setParam(caller_id, '/new_param', val))
        full_dict['new_param'] = val
        self.assertEquals(val, self.apiSuccess(master.getParam(caller_id, '/new_param')))
        # test with relative-name resolution
        self.assertEquals(val, self.apiSuccess(master.getParam(caller_id, 'new_param')))
        # test full get
        ps_full_dict = self.apiSuccess(master.getParam(caller_id, '/'))
        self._filterDict(ps_full_dict)
            
        self.assertEquals(full_dict, ps_full_dict)
        
        # test with param in sub-namespace
        val = random.randint(0, 10000)        
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/sub/sub2/new_param2')))
        self.apiSuccess(master.setParam(caller_id, '/sub/sub2/new_param2', val))
        full_dict['sub'] = {'sub2': { 'new_param2': val }}
        self.assertEquals(val, self.apiSuccess(master.getParam(caller_id, '/sub/sub2/new_param2')))
        # test with relative-name resolution
        self.assertEquals(val, self.apiSuccess(master.getParam(caller_id, 'sub/sub2/new_param2')))
        self.assertEquals(val, self.apiSuccess(master.getParam('/sub/node', 'sub2/new_param2')))
        self.assertEquals(val, self.apiSuccess(master.getParam('/sub/sub2/node', 'new_param2')))
        # test that parameter server allows gets across namespaces (#493)
        self.assertEquals(val, self.apiSuccess(master.getParam('/foo/bar/baz/blah/node', '/sub/sub2/new_param2')))
        # test full get
        ps_full_dict = self.apiSuccess(master.getParam(caller_id, '/'))
        self._filterDict(ps_full_dict)
        self.assertEquals(full_dict, ps_full_dict)


        # test that parameter server namespace-get (#587)
        val1 = random.randint(0, 10000)
        val2 = random.randint(0, 10000)
        val3 = random.randint(0, 10000)
        
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/gains/P')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/gains/I')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/gains/D')))                        
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/gains')))

        self.apiSuccess(master.setParam(caller_id, '/gains/P', val1))
        self.apiSuccess(master.setParam(caller_id, '/gains/I', val2))
        self.apiSuccess(master.setParam(caller_id, '/gains/D', val3))        

        pid = {'P': val1, 'I': val2, 'D': val3}
        full_dict['gains'] = pid
        self.assertEquals(pid,
                          self.apiSuccess(master.getParam(caller_id, '/gains')))
        self.assertEquals(pid,
                          self.apiSuccess(master.getParam(caller_id, '/gains/')))
        ps_full_dict = self.apiSuccess(master.getParam(caller_id, '/'))
        self._filterDict(ps_full_dict)
        self.assertEquals(full_dict, ps_full_dict)

        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/ns/gains/P')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/ns/gains/I')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/ns/gains/D')))                        
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/ns/gains')))
        
        self.apiSuccess(master.setParam(caller_id, '/ns/gains/P', val1))
        self.apiSuccess(master.setParam(caller_id, '/ns/gains/I', val2))
        self.apiSuccess(master.setParam(caller_id, '/ns/gains/D', val3))        
        full_dict['ns'] = {'gains': pid}
        
        self.assertEquals(pid,
                          self.apiSuccess(master.getParam(caller_id, '/ns/gains')))
        self.assertEquals({'gains': pid},
                          self.apiSuccess(master.getParam(caller_id, '/ns/')))
        self.assertEquals({'gains': pid},
                          self.apiSuccess(master.getParam(caller_id, '/ns')))
        ps_full_dict = self.apiSuccess(master.getParam(caller_id, '/'))
        self._filterDict(ps_full_dict)
        self.assertEquals(full_dict, ps_full_dict)
        
        
    # testSetParam: test basic setParam behavior. Value encoding verified separately by testParamValues
    def _testSetParam(self):
        master = self.master
        caller_id = '/node'
        val = random.randint(0, 10000)

        # very similar to has param sequence
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/new_param')))
        self.apiSuccess(master.setParam(caller_id, '/new_param', val))
        self.assertEquals(val, self.apiSuccess(master.getParam(caller_id, '/new_param')))
        # test with relative-name resolution
        self.assertEquals(val, self.apiSuccess(master.getParam(caller_id, 'new_param')))

        # test type mutation, including from dictionary to value and back
        vals = ['a', {'a': 'b'}, 1, 1., 'foo', {'c': 'd'}, 4]
        for v in vals:
            self.apiSuccess(master.setParam(caller_id, '/multi/multi_param', v))
            self.assertEquals(v, self.apiSuccess(master.getParam(caller_id, 'multi/multi_param')))        
        
        # test with param in sub-namespace
        val = random.randint(0, 10000)        
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/sub/sub2/new_param2')))
        self.apiSuccess(master.setParam(caller_id, '/sub/sub2/new_param2', val))
        self.assertEquals(val, self.apiSuccess(master.getParam(caller_id, '/sub/sub2/new_param2')))
        # test with relative-name resolution
        self.assertEquals(val, self.apiSuccess(master.getParam(caller_id, 'sub/sub2/new_param2')))
        self.assertEquals(val, self.apiSuccess(master.getParam('/sub/node', 'sub2/new_param2')))
        self.assertEquals(val, self.apiSuccess(master.getParam('/sub/sub2/node', 'new_param2')))

        # test that parameter server namespace-set (#587)
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/gains/P')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/gains/I')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/gains/D')))                        
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/gains')))

        pid = {'P': random.randint(0, 10000), 'I': random.randint(0, 10000), 'D': random.randint(0, 10000)}
        self.apiSuccess(master.setParam(caller_id, '/gains', pid))
        self.assertEquals(pid,  self.apiSuccess(master.getParam(caller_id, '/gains')))
        self.assertEquals(pid['P'], self.apiSuccess(master.getParam(caller_id, '/gains/P')))
        self.assertEquals(pid['I'], self.apiSuccess(master.getParam(caller_id, '/gains/I')))
        self.assertEquals(pid['D'], self.apiSuccess(master.getParam(caller_id, '/gains/D')))

        subns = {'gains1': pid, 'gains2': pid}
        self.apiSuccess(master.setParam(caller_id, '/ns', subns))
        self.assertEquals(pid['P'], self.apiSuccess(master.getParam(caller_id, '/ns/gains1/P')))
        self.assertEquals(pid['I'], self.apiSuccess(master.getParam(caller_id, '/ns/gains1/I')))
        self.assertEquals(pid['D'], self.apiSuccess(master.getParam(caller_id, '/ns/gains1/D')))
        self.assertEquals(pid, self.apiSuccess(master.getParam(caller_id, '/ns/gains1')))
        self.assertEquals(pid, self.apiSuccess(master.getParam(caller_id, '/ns/gains2')))
        self.assertEquals(subns, self.apiSuccess(master.getParam(caller_id, '/ns/')))

        # test empty dictionary set
        self.apiSuccess(master.setParam(caller_id, '/ns', {}))
        # - param should still exist
        self.assert_(self.apiSuccess(master.hasParam(caller_id, '/ns/')))
        # - value should remain dictionary
        self.assertEquals({}, self.apiSuccess(master.getParam(caller_id, '/ns/')))
        # - value2 below /ns/ should be erased
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/ns/gains1')))
        self.failIf(self.apiSuccess(master.hasParam(caller_id, '/ns/gains1/P')))
        

    # testParamValues: test storage of all XML-RPC compatible types"""
    def _testParamValues(self):
        from xmlrpclib import Binary
        testVals = [
            ['int', [0, 1024, 2147483647, -2147483647]],
            ['boolean', [True, False]],
            #no longer testing null char
            #['string', ['', '\0', 'x', 'hello', ''.join([chr(n) for n in xrange(0, 255)])]],
            ['unicode-string', [u'', u'hello', unicode('Andr\302\202', 'utf-8'), unicode('\377\376A\000n\000d\000r\000\202\000', 'utf-16')]],
            ['string-easy-ascii', [chr(n) for n in xrange(32, 128)]],

            #['string-mean-ascii-low', [chr(n) for n in xrange(9, 10)]], #separate for easier book-keeping
            #['string-mean-ascii-low', [chr(n) for n in xrange(1, 31)]], #separate for easier book-keeping
            #['string-mean-signed', [chr(n) for n in xrange(129, 256)]],
            ['string', ['', 'x', 'hello-there', 'new\nline', 'tab\t']],
            ['double', [0.0, math.pi, -math.pi, 3.4028235e+38, -3.4028235e+38]],
            #TODO: microseconds?
            ['datetime', [datetime.datetime(2005, 12, 6, 12, 13, 14), datetime.datetime(1492, 12, 6, 12, 13, 14)]],
            ['base64', [Binary(''), Binary('\0'), Binary(''.join([chr(n) for n in xrange(0, 255)]))]],
            ['array', [[], [1, 2, 3], ['a', 'b', 'c'], [0.0, 0.1, 0.2, 2.0, 2.1, -4.0],
                       [1, 'a', True], [[1, 2, 3], ['a', 'b', 'c'], [1.0, 2.1, 3.2]]]
             ],
            ]
        master = self.master

        print "Putting parameters onto the server"
        # put our params into the parameter server
        contexts = ['', 'scope1', 'scope/sub1/sub2']
        myState = {}
        failures = []
        for ctx in contexts:
            self._setParam(ctx, myState, testVals, master)
        self._checkParamState(myState)
        
        print "Deleting all of our parameters"
        # delete all of our parameters
        paramKeys = myState.keys()
        ctx = ''
        count = 0
        for key in paramKeys:
            count += 1
            print "deleting [%s], [%s]"%(ctx, key)
            self.apiSuccess(master.deleteParam(ctx, key))
            del myState[key]
            # far too intensive to check every time
            if count % 50 == 0:
                self._checkParamState(myState)
        self._checkParamState(myState)

    def _testEncapsulation(self):
        """testEncapsulation: test encapsulation: setting same parameter at different levels"""
        master = self.master
        myState = {}
        self._checkParamState(myState)

        testContexts = ['', 'en', 'en/sub1', 'en/sub2', 'en/sub1/sub2']
        for c in testContexts:
            c = make_global_ns(c)
            testKey = 'param1'
            testVal = random.randint(-1000000, 100000)
            callerId = ns_join(c, "node")
            trueKey = ns_join(c, testKey)
            master.setParam(callerId, testKey, testVal)
            myState[trueKey] = testVal
            # make sure the master has the parameter under both keys and that they are equal
            v1 = self.apiSuccess(master.getParam('/', trueKey))
            v2 = self.apiSuccess(master.getParam(callerId, testKey))
            assert v1 == v2, "[%s]: %s vs. [%s,%s]: %s"%(trueKey, v1, callerId, testKey, v2)
            assert self.apiSuccess(master.hasParam(callerId, testKey)), testKey
            assert self.apiSuccess(master.hasParam('node', trueKey)), trueKey

        self._checkParamState(myState)

    def _testPrivateNames(self):
        master = self.master
        myState = {}
        self._checkParamState(myState)

        testContexts = ['', 'sub1', 'sub1/sub2', 'sub1/sub2/sub3']
        for c in testContexts:
            c = make_global_ns(c)
            callerId = ns_join(c, "node")
            
            keyStub = "param1"
            testKey = "~%s"%keyStub
            testVal = random.randint(-1000000, 100000)            
            master.setParam(callerId, testKey, testVal)
            trueKey = ns_join(callerId, keyStub)
            myState[trueKey] = testVal

            print "checking", trueKey
            v1 = self.apiSuccess(master.getParam('/', trueKey))
            v2 = self.apiSuccess(master.getParam(callerId, testKey))
            assert v1 == v2, "[%s]: %s vs. [%s,%s]: %s"%(trueKey, v1, callerId, testKey, v2)
            assert self.apiSuccess(master.hasParam(callerId, testKey)), testKey
            assert self.apiSuccess(master.hasParam('node', trueKey)), trueKey

            #test setting a local param on a different node
            testKey = ns_join("altnode","param2")
            testVal = random.randint(-1000000, 100000)            
            master.setParam(callerId, testKey, testVal)
            trueKey = ns_join(c, testKey)
            altCallerId = ns_join(c, "altnode")
            myState[trueKey] = testVal

            v1 = self.apiSuccess(master.getParam(altCallerId, "~param2"))
            v2 = self.apiSuccess(master.getParam(callerId, testKey))
            assert v1 == v2
            assert self.apiSuccess(master.hasParam(callerId, testKey)), testKey
            assert self.apiSuccess(master.hasParam(altCallerId, "~param2"))

        self._checkParamState(myState)

    ## testScopeUp: test that parameter server can chain up scopes to find/delete parameters
    def _testScopeUp(self):
        master = self.master
        myState = {}
        self._checkParamState(myState)
        
        testVal = random.randint(-1000000, 100000)
        master.setParam('/', 'uparam1', testVal)
        myState['/uparam1'] = testVal 
        assert testVal == self.apiSuccess(master.getParam('/node', 'uparam1'))
        assert testVal == self.apiSuccess(master.getParam('/uptest/node', 'uparam1'))
        assert testVal == self.apiSuccess(master.getParam('/uptest/sub1/node', 'uparam1'))
        assert testVal == self.apiSuccess(master.getParam('/uptest/sub1/sub2/node', 'uparam1'))

        testVal = random.randint(-1000000, 100000)
        master.setParam('/uptest2/sub1/node', 'uparam2', testVal)
        myState['/uptest2/sub1/uparam2'] = testVal 
        assert testVal == self.apiSuccess(master.getParam('/uptest2/sub1/node', 'uparam2'))
        assert testVal == self.apiSuccess(master.getParam('/uptest2/sub1/sub2/node', 'uparam2'))
        assert testVal == self.apiSuccess(master.getParam('/uptest2/sub1/sub2/sub3/node', 'uparam2'))

        # make sure we can ascend then descend
        if 0:
            testVal = random.randint(-1000000, 100000)
            master.setParam('/uptest3/node', 'alt2/alt3/uparam3', testVal)
            myState['/uptest3/alt2/alt3/uparam3'] = testVal 
            assert testVal == self.apiSuccess(master.getParam('/uptest3/sub1/node', 'alt2/alt3/uparam3'))
            assert testVal == self.apiSuccess(master.getParam('/uptest3/sub1/sub2/node', 'alt2/alt3/uparam3'))
            assert testVal == self.apiSuccess(master.getParam('/uptest3/sub1/sub2/sub3/node', 'alt2/alt3/uparam3'))
            self._checkParamState(myState)
        
        #verify upwards deletion
        self.apiSuccess(master.deleteParam('/uptest/sub1/sub2/node', 'uparam1'))
        del myState['/uparam1']
        self._checkParamState(myState)        
        self.apiSuccess(master.deleteParam('/uptest2/sub1/sub2/sub3/node', 'uparam2'))
        del myState['/uptest2/sub1/uparam2']
        if 0:
            self.apiSuccess(master.deleteParam('/uptest3/sub1/sub2/sub3/node', 'alt2/alt3/uparam3'))
            del myState['/uptest3/alt2/alt3/uparam3']
        self._checkParamState(myState)        
        
    ## testScopeDown: test scoping rules for sub contexts
    def _testScopeDown(self):
        master = self.master
        myState = {}
        self._checkParamState(myState)

        # test that parameter server down not chain down scopes
        testVal = random.randint(-1000000, 100000)
        master.setParam('/down/one/two/three/node', 'dparam1', testVal)
        myState['/down/one/two/three/dparam1'] = testVal
        assert not self.apiSuccess(master.hasParam('/down/one/node', 'dparam1')) 
        assert not self.apiSuccess(master.hasParam('/down/one/two/node', 'dparam1'))
        self.apiError(master.getParam('/down/one/node', 'dparam1')) 
        self.apiError(master.getParam('/down/one/two/node', 'dparam1'))

        # test that parameter server allows setting of parameters further down (1)
        testVal = random.randint(-1000000, 100000)
        master.setParam('/', '/down2/dparam2', testVal)
        myState['/down2/dparam2'] = testVal         
        assert testVal == self.apiSuccess(master.getParam('/down2/node', 'dparam2'))
        assert testVal == self.apiSuccess(master.getParam('/', 'down2/dparam2'))
        assert not self.apiSuccess(master.hasParam('/down2/node', 'down2/dparam2'))
        self.apiError(master.getParam('/down2/node', 'down2/dparam2'))
        self._checkParamState(myState)
        
        # test that parameter server allows setting of parameters further down (2)
        testVal = random.randint(-1000000, 100000)
        master.setParam('/', '/down3/sub/dparam3', testVal)
        myState['/down3/sub/dparam3'] = testVal
        assert testVal == self.apiSuccess(master.getParam('/down3/sub/node', 'dparam3'))
        assert testVal == self.apiSuccess(master.getParam('/down3/node', 'sub/dparam3'))
        assert testVal == self.apiSuccess(master.getParam('/', 'down3/sub/dparam3'))
        assert testVal == self.apiSuccess(master.getParam('/down3/sub/sub2/node', 'dparam3'))
        assert not self.apiSuccess(master.hasParam('/down3/sub/node', 'sub/dparam3'))
        assert not self.apiSuccess(master.hasParam('/down3/sub/node', 'down3/sub/dparam3'))
        self.apiError(master.getParam('/down3/sub/node', 'sub/dparam3'))
        self.apiError(master.getParam('/down3/sub/node', 'down3/sub/dparam3'))
        self._checkParamState(myState)

        # test downwards deletion
        master.setParam('/', '/down4/sub/dparam4A', testVal)
        self.apiSuccess(master.deleteParam('/down4/sub/node', 'dparam4A'))
        assert not self.apiSuccess(master.hasParam('/down4/sub', 'dparam4A'))
        master.setParam('/', '/down4/sub/dparam4B', testVal)        
        self.apiSuccess(master.deleteParam('/down4/node', 'sub/dparam4B'))
        assert not self.apiSuccess(master.hasParam('/down4/sub', 'dparam4B'))
        master.setParam('/', '/down4/sub/dparam4C', testVal)
        self.apiSuccess(master.deleteParam('/', 'down4/sub/dparam4C'))
        assert not self.apiSuccess(master.hasParam('/down4/sub/node', 'dparam4C'))
        self._checkParamState(myState)

