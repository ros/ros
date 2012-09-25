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
import unittest
import time
import random
import datetime

from rosgraph.names import make_global_ns, ns_join

# mock of subscription tests
class ThreadPoolMock(object):
    def queue_task(*args): pass
    
## Unit tests for rosmaster.paramserver module
class TestRospyParamServer(unittest.TestCase):
    
    def test_compute_param_updates(self):
        from rosmaster.paramserver import compute_param_updates
        # spec requires that subscriptions always have a trailing slash
        tests = [
            # [correct val], (subscribers, param_key, param_value)
            ([],({}, '/foo', 1)),
            ([],({'/bar': 'barapi'}, '/foo/', 1)),
            ([],({'/bar/': 'barapi'}, '/foo/', 1)),
            
            # make sure that it's robust to aliases
            ([('fooapi', '/foo/', 1)], ({'/foo/': 'fooapi'}, '/foo', 1)),
            ([('fooapi', '/foo/', 1)], ({'/foo/': 'fooapi'}, '/foo/', 1)),
            
            # check namespace subscription
            ([('fooapi', '/foo/val/', 1)], ({'/foo/': 'fooapi'}, '/foo/val', 1)),

            # check against dictionary param values
            ([],({'/bar/': 'barapi'}, '/foo/', {'bar': 2})),
            ([('fooapi', '/foo/val/', 1)], ({'/foo/val/': 'fooapi'}, '/foo', {'val' : 1})),

            ([('fooapi', '/foo/bar/val/', 1)], ({'/foo/bar/val/': 'fooapi'}, '/foo', {'bar' : {'val' : 1}})),            
            ([('fooapi', '/foo/bar/', {'val': 1})], ({'/foo/bar/': 'fooapi'}, '/foo', {'bar' : {'val' : 1}})),
            ([('fooapi', '/foo/', {'bar':{'val': 1}})], ({'/foo/': 'fooapi'}, '/foo', {'bar' : {'val' : 1}})),

            ([('fooapi', '/foo/', {'bar': 1, 'baz': 2}), ('foobazapi', '/foo/baz/', 2)],
             ({'/foo/': 'fooapi', '/foo/baz/': 'foobazapi'}, '/foo', {'bar' : 1, 'baz': 2})),

            ([('foobarapi', '/foo/bar/', 1), ('foobazapi', '/foo/baz/', 2)],
             ({'/foo/bar/': 'foobarapi', '/foo/baz/': 'foobazapi'}, '/foo', {'bar' : 1, 'baz': 2})),

            # deletion of higher level tree
            ([('delapi', '/del/bar/', {})],
             ({'/del/bar/': 'delapi'}, '/del', {})),

            ]
        for correct, args in tests:
            val = compute_param_updates(*args)
            self.assertEquals(len(correct), len(val), "Failed: \n%s \nreturned \n%s\nvs correct\n%s"%(str(args), str(val), str(correct)))
            for c in correct:
                self.assert_(c in val, "Failed: \n%s \ndid not include \n%s. \nIt returned \n%s"%(str(args), c, val))


    def notify_task(self, updates):
        self.last_update = updates

    def test_subscribe_param_simple(self):
        from rosmaster.registrations import RegistrationManager
        from rosmaster.paramserver import ParamDictionary

        # setup node and subscriber data
        reg_manager = RegistrationManager(ThreadPoolMock())
        param_server = ParamDictionary(reg_manager)

        # subscribe to parameter that has not been set yet
        self.last_update = None
        self.assertEquals({}, param_server.subscribe_param('/foo', ('node1', 'http://node1:1')))
        param_server.set_param('/foo', 1, notify_task=self.notify_task)
        self.assertEquals([([('node1', 'http://node1:1')], '/foo/', 1), ], self.last_update)
        
        # resubscribe
        self.assertEquals(1, param_server.subscribe_param('/foo', ('node1', 'http://node1:1')))
        param_server.set_param('/foo', 2, notify_task=self.notify_task)
        self.assertEquals([([('node1', 'http://node1:1')], '/foo/', 2), ], self.last_update)

        # resubscribe (test canonicalization of parameter name)
        self.assertEquals(2, param_server.subscribe_param('/foo/', ('node1', 'http://node1:1')))
        param_server.set_param('/foo', 'resub2', notify_task=self.notify_task)
        self.assertEquals([([('node1', 'http://node1:1')], '/foo/', 'resub2'), ], self.last_update)
        
        # change the URI
        self.assertEquals('resub2', param_server.subscribe_param('/foo', ('node1', 'http://node1b:1')))
        self.assertEquals('http://node1b:1', reg_manager.get_node('node1').api)
        param_server.set_param('/foo', 3, notify_task=self.notify_task)
        self.assertEquals([([('node1', 'http://node1b:1')], '/foo/', 3), ], self.last_update)
        
        # multiple subscriptions to same param
        self.assertEquals(3, param_server.subscribe_param('/foo', ('node2', 'http://node2:2')))
        self.assertEquals('http://node2:2', reg_manager.get_node('node2').api)
        param_server.set_param('/foo', 4, notify_task=self.notify_task)
        self.assertEquals([([('node1', 'http://node1b:1'), ('node2', 'http://node2:2')], '/foo/', 4), ], self.last_update)

    def test_subscribe_param_tree(self):
        from rosmaster.registrations import RegistrationManager
        from rosmaster.paramserver import ParamDictionary

        # setup node and subscriber data
        reg_manager = RegistrationManager(ThreadPoolMock())
        param_server = ParamDictionary(reg_manager)

        # Test Parameter Tree Subscriptions

        # simple case - subscribe and set whole tree
        gains = {'p': 'P', 'i': 'I', 'd' : 'D'}
        self.assertEquals({}, param_server.subscribe_param('/gains', ('ptnode', 'http://ptnode:1')))
        param_server.set_param('/gains', gains.copy(), notify_task=self.notify_task)
        self.assertEquals([([('ptnode', 'http://ptnode:1')], '/gains/', gains), ], self.last_update)
        # - test with trailing slash
        param_server.set_param('/gains/', gains.copy(), notify_task=self.notify_task)
        self.assertEquals([([('ptnode', 'http://ptnode:1')], '/gains/', gains), ], self.last_update)

        # change params within tree
        param_server.set_param('/gains/p', 'P2', notify_task=self.notify_task)
        self.assertEquals([([('ptnode', 'http://ptnode:1')], '/gains/p/', 'P2'), ], self.last_update)
        param_server.set_param('/gains/i', 'I2', notify_task=self.notify_task)
        self.assertEquals([([('ptnode', 'http://ptnode:1')], '/gains/i/', 'I2'), ], self.last_update)

        # test overlapping subscriptions
        self.assertEquals('P2', param_server.subscribe_param('/gains/p', ('ptnode2', 'http://ptnode2:2')))
        param_server.set_param('/gains', gains.copy(), notify_task=self.notify_task)
        self.assertEquals([([('ptnode', 'http://ptnode:1')], '/gains/', gains), \
                           ([('ptnode2', 'http://ptnode2:2')], '/gains/p/', 'P'), \
                           ], self.last_update)
        # - retest with trailing slash on subscribe
        self.last_update = None
        self.assertEquals('P', param_server.subscribe_param('/gains/p/', ('ptnode2', 'http://ptnode2:2')))
        param_server.set_param('/gains', gains.copy(), notify_task=self.notify_task)
        self.assertEquals([([('ptnode', 'http://ptnode:1')], '/gains/', gains), \
                           ([('ptnode2', 'http://ptnode2:2')], '/gains/p/', 'P'), \
                           ], self.last_update)
        # test with overlapping (change to sub param)
        param_server.set_param('/gains/p', 'P3', notify_task=self.notify_task)
        # - this is a bit overtuned as a more optimal ps could use one update
        self.assertEquals([([('ptnode2', 'http://ptnode2:2')], '/gains/p/', 'P3'), \
                           ([('ptnode', 'http://ptnode:1')], '/gains/p/', 'P3'), \
                           ], self.last_update)
        
        # virtual deletion: subscribe to subparam, parameter tree reset
        self.last_update = None
        param_server.set_param('/gains2', gains.copy(), notify_task=self.notify_task)
        self.assertEquals('P', param_server.subscribe_param('/gains2/p/', ('ptnode3', 'http://ptnode3:3')))
        # - erase the sub parameters
        param_server.set_param('/gains2', {}, notify_task=self.notify_task)        
        self.assertEquals([([('ptnode3', 'http://ptnode3:3')], '/gains2/p/', {}), ], self.last_update)        

        #Final test: test subscription to entire tree
        self.last_update = None
        param_server.delete_param('/gains')
        param_server.delete_param('/gains2')        
        self.assertEquals({}, param_server.get_param('/'))
        self.assertEquals({}, param_server.subscribe_param('/', ('allnode', 'http://allnode:1')))
        param_server.set_param('/one', 1, notify_task=self.notify_task)
        self.assertEquals([([('allnode', 'http://allnode:1')], '/one/', 1), ], self.last_update)
        param_server.set_param('/two', 2, notify_task=self.notify_task)
        self.assertEquals([([('allnode', 'http://allnode:1')], '/two/', 2), ], self.last_update)
        param_server.set_param('/foo/bar', 'bar', notify_task=self.notify_task)
        self.assertEquals([([('allnode', 'http://allnode:1')], '/foo/bar/', 'bar'), ], self.last_update)
        

    # verify that subscribe_param works with parameter deletion
    def test_subscribe_param_deletion(self):
        from rosmaster.registrations import RegistrationManager
        from rosmaster.paramserver import ParamDictionary

        # setup node and subscriber data
        reg_manager = RegistrationManager(ThreadPoolMock())
        param_server = ParamDictionary(reg_manager)

        # subscription to then delete parameter
        self.assertEquals({}, param_server.subscribe_param('/foo', ('node1', 'http://node1:1')))
        param_server.set_param('/foo', 1, notify_task=self.notify_task)
        param_server.delete_param('/foo', notify_task=self.notify_task)
        self.assertEquals([([('node1', 'http://node1:1')], '/foo/', {}), ], self.last_update)
        
        # subscribe to and delete whole tree
        gains = {'p': 'P', 'i': 'I', 'd' : 'D'}
        self.assertEquals({}, param_server.subscribe_param('/gains', ('deltree', 'http://deltree:1')))
        param_server.set_param('/gains', gains.copy(), notify_task=self.notify_task)
        param_server.delete_param('/gains', notify_task=self.notify_task)
        self.assertEquals([([('deltree', 'http://deltree:1')], '/gains/', {}), ], self.last_update)

        # subscribe to and delete params within subtree
        self.assertEquals({}, param_server.subscribe_param('/gains2', ('deltree2', 'http://deltree2:2')))
        param_server.set_param('/gains2', gains.copy(), notify_task=self.notify_task)
        param_server.delete_param('/gains2/p', notify_task=self.notify_task)
        self.assertEquals([([('deltree2', 'http://deltree2:2')], '/gains2/p/', {}), ], self.last_update)
        param_server.delete_param('/gains2/i', notify_task=self.notify_task)
        self.assertEquals([([('deltree2', 'http://deltree2:2')], '/gains2/i/', {}), ], self.last_update)        
        param_server.delete_param('/gains2', notify_task=self.notify_task)
        self.assertEquals([([('deltree2', 'http://deltree2:2')], '/gains2/', {}), ], self.last_update)
        
        # delete parent tree
        k = '/ns1/ns2/ns3/key'
        self.assertEquals({}, param_server.subscribe_param(k, ('del_parent', 'http://del_parent:1')))
        param_server.set_param(k, 1, notify_task=self.notify_task)
        param_server.delete_param('/ns1/ns2', notify_task=self.notify_task)
        self.assertEquals([([('del_parent', 'http://del_parent:1')], '/ns1/ns2/ns3/key/', {}), ], self.last_update)
    
    def test_unsubscribe_param(self):
        from rosmaster.registrations import RegistrationManager
        from rosmaster.paramserver import ParamDictionary

        # setup node and subscriber data
        reg_manager = RegistrationManager(ThreadPoolMock())
        param_server = ParamDictionary(reg_manager)

        # basic test
        self.last_update = None
        self.assertEquals({}, param_server.subscribe_param('/foo', ('node1', 'http://node1:1')))
        param_server.set_param('/foo', 1, notify_task=self.notify_task)
        self.assertEquals([([('node1', 'http://node1:1')], '/foo/', 1), ], self.last_update)
        # - return value is actually generated by Registrations
        code, msg, val = param_server.unsubscribe_param('/foo', ('node1', 'http://node1:1'))
        self.assertEquals(1, code)
        self.assertEquals(1, val)
        self.last_update = None
        param_server.set_param('/foo', 2, notify_task=self.notify_task)
        self.assertEquals(None, self.last_update)
        # - repeat the unsubscribe
        code, msg, val = param_server.unsubscribe_param('/foo', ('node1', 'http://node1:1'))
        self.assertEquals(1, code)
        self.assertEquals(0, val)
        self.last_update = None
        param_server.set_param('/foo', 2, notify_task=self.notify_task)
        self.assertEquals(None, self.last_update)

        # verify that stale unsubscribe has no effect on active subscription
        self.last_update = None
        self.assertEquals({}, param_server.subscribe_param('/bar', ('barnode', 'http://barnode:1')))
        param_server.set_param('/bar', 3, notify_task=self.notify_task)
        self.assertEquals([([('barnode', 'http://barnode:1')], '/bar/', 3), ], self.last_update)
        code, msg, val = param_server.unsubscribe_param('/foo', ('barnode', 'http://notbarnode:1'))
        self.assertEquals(1, code)
        self.assertEquals(0, val)
        param_server.set_param('/bar', 4, notify_task=self.notify_task)
        self.assertEquals([([('barnode', 'http://barnode:1')], '/bar/', 4), ], self.last_update)
        
    
    def _set_param(self, ctx, my_state, test_vals, param_server):
        ctx = make_global_ns(ctx)
        for type, vals in test_vals:
            try:
                caller_id = ns_join(ctx, "node")
                count = 0
                for val in vals:
                    key = ns_join(caller_id, "%s-%s"%(type,count))
                    param_server.set_param(key, val)
                    self.assert_(param_server.has_param(key))
                    true_key = ns_join(ctx, key)
                    my_state[true_key] = val
                    count += 1
            except Exception, e:
                assert "getParam failed on type[%s], val[%s]"%(type,val)
        #self._check_param_state(my_state)

    def _check_param_state(self, param_server, my_state):
        for (k, v) in my_state.iteritems():
            assert param_server.has_param(k)
            #print "verifying parameter %s"%k
            try:
                v2 = param_server.get_param(k)
            except:
                raise Exception("Exception raised while calling param_server.get_param(%s): %s"%(k, traceback.format_exc()))
            
            self.assertEquals(v, v2)
        param_names = my_state.keys()
        ps_param_names = param_server.get_param_names()
        assert not set(param_names) ^ set(ps_param_names), "parameter server keys do not match local: %s"%(set(param_names)^set(ps_param_names))


    # test_has_param: test has_param API
    def test_has_param(self):
        from rosmaster.paramserver import ParamDictionary
        param_server = ParamDictionary(None)

        self.failIf(param_server.has_param('/new_param'))
        param_server.set_param('/new_param', 1)
        self.assert_(param_server.has_param('/new_param'))

        # test with param in sub-namespace
        self.failIf(param_server.has_param('/sub/sub2/new_param2'))
        # - verify that parameter tree does not exist yet (#587)
        for k in ['/sub/sub2/', '/sub/sub2', '/sub/', '/sub']:
            self.failIf(param_server.has_param(k))
        param_server.set_param('/sub/sub2/new_param2', 1)
        self.assert_(param_server.has_param('/sub/sub2/new_param2'))
        # - verify that parameter tree now exists (#587)
        for k in ['/sub/sub2/', '/sub/sub2', '/sub/', '/sub']:
            self.assert_(param_server.has_param(k))

    
    ## test ^param naming, i.e. upwards-looking get access
    ## @param self
    def test_search_param(self):
        from rosmaster.paramserver import ParamDictionary
        param_server = ParamDictionary(None)

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

        # test invalid input
        for k in ['', None, '~param']:
            try:
                param_server.search_param('/level1/level2', k)
                self.fail("param_server search should have failed on [%s]"%k)
            except ValueError: pass
        for ns in ['', None, 'relative', '~param']:
            try:
                param_server.search_param(ns, 'param')
                self.fail("param_server search should have failed on %s"%k)
            except ValueError: pass

        # set the val parameter at four levels so we can validate search
        
        # - set val1
        self.failIf(param_server.has_param('/level1/param'))
        self.failIf(param_server.search_param('/level1/node', 'param')) 
        param_server.set_param('/level1/param', val1)
        
        # - test param on val1
        for ns in ['/level1/node', '/level1/level2/node', '/level1/level2/level3/node']:
            self.assertEquals('/level1/param', param_server.search_param(ns, 'param'), "failed with ns[%s]"%ns)
            self.assertEquals('/level1/param/', param_server.search_param(ns, 'param/'))
            self.assertEquals('/level1/param/level1_p1', param_server.search_param(ns, 'param/level1_p1'))
            self.assertEquals('/level1/param/level1_p2/level2_p2', param_server.search_param(ns, 'param/level1_p2/level2_p2'))
        self.assertEquals(None, param_server.search_param('/root', 'param'))
        self.assertEquals(None, param_server.search_param('/root', 'param/'))        

        # - set val2
        self.failIf(param_server.has_param('/level1/level2/param'))
        param_server.set_param('/level1/level2/param', val2)

        # - test param on val2
        for ns in ['/level1/level2/node', '/level1/level2/level3/node', '/level1/level2/level3/level4/node']:
            self.assertEquals('/level1/level2/param', param_server.search_param(ns, 'param'))
            self.assertEquals('/level1/level2/param/', param_server.search_param(ns, 'param/'))
        self.assertEquals('/level1/param', param_server.search_param('/level1/node', 'param'))
        self.assertEquals('/level1/param/', param_server.search_param('/level1/node', 'param/'))        
        self.assertEquals(None, param_server.search_param('/root', 'param'))
        
        # - set val3
        self.failIf(param_server.has_param('/level1/level2/level3/param'))
        param_server.set_param('/level1/level2/level3/param', val3)

        # - test param on val3
        for ns in ['/level1/level2/level3/node', '/level1/level2/level3/level4/node']:
            self.assertEquals('/level1/level2/level3/param', param_server.search_param(ns, 'param'))
        self.assertEquals('/level1/level2/param', param_server.search_param('/level1/level2/node', 'param'))
        self.assertEquals('/level1/param', param_server.search_param('/level1/node', 'param'))

        # test subparams before we set val4 on the root
        #  - test looking for param/sub_param

        self.assertEquals(None, param_server.search_param('/root', 'param'))
        self.assertEquals(None, param_server.search_param('/root', 'param/level1_p1'))
        self.assertEquals(None, param_server.search_param('/not/level1/level2/level3/level4/node', 'param/level1_p1'))
        tests = [
            ('/level1/node', '/level1/param/'),
            ('/level1/level2/', '/level1/level2/param/'),
            ('/level1/level2', '/level1/level2/param/'),
            ('/level1/level2/node', '/level1/level2/param/'),
            ('/level1/level2/notlevel3', '/level1/level2/param/'),
            ('/level1/level2/notlevel3/node', '/level1/level2/param/'),
            ('/level1/level2/level3/level4', '/level1/level2/level3/param/'),
            ('/level1/level2/level3/level4/', '/level1/level2/level3/param/'),
            ('/level1/level2/level3/level4/node', '/level1/level2/level3/param/'),
            
            ]
        for ns, pbase in tests:
            self.assertEquals(pbase+'level1_p1',
                              param_server.search_param(ns, 'param/level1_p1'))
            retval = param_server.search_param(ns, 'param/level1_p2/level2_p2')
            self.assertEquals(pbase+'level1_p2/level2_p2', retval,
                              "failed with ns[%s] pbase[%s]: %s"%(ns, pbase, retval))

        # - set val4 on the root
        self.failIf(param_server.has_param('/param'))
        param_server.set_param('/param', val4)
        self.assertEquals('/param', param_server.search_param('/root', 'param'))
        self.assertEquals('/param', param_server.search_param('/notlevel1/node', 'param'))
        self.assertEquals('/level1/param', param_server.search_param('/level1/node', 'param'))
        self.assertEquals('/level1/param', param_server.search_param('/level1', 'param'))
        self.assertEquals('/level1/param', param_server.search_param('/level1/', 'param'))

        # make sure that partial match works
        val5 = { 'level1_p1': random.randint(0, 10000),
                 'level1_p2' : { }}
        
        self.failIf(param_server.has_param('/partial1/param'))
        param_server.set_param('/partial1/param', val5)
        self.assertEquals('/partial1/param', param_server.search_param('/partial1', 'param'))
        self.assertEquals('/partial1/param/level1_p1',
                          param_server.search_param('/partial1', 'param/level1_p1'))
        # - this is the important check, should return key even if it doesn't exist yet based on stem match
        self.assertEquals('/partial1/param/non_existent',
                          param_server.search_param('/partial1', 'param/non_existent'))
        self.assertEquals('/partial1/param/level1_p2/non_existent',
                          param_server.search_param('/partial1', 'param/level1_p2/non_existent'))


    # test_get_param: test basic getParam behavior. Value encoding verified separately by testParamValues
    def test_get_param(self):
        from rosmaster.paramserver import ParamDictionary
        param_server = ParamDictionary(None)

        val = random.randint(0, 10000)

        full_dict = {}
        
        # very similar to has param sequence
        self.failIf(param_server.has_param('/new_param'))
        self.failIf(param_server.has_param('/new_param/'))        
        self.assertGetParamFail(param_server, '/new_param')
        param_server.set_param('/new_param', val)
        full_dict['new_param'] = val
        self.assertEquals(val, param_server.get_param('/new_param'))
        self.assertEquals(val, param_server.get_param('/new_param/'))
        # - test homonym
        self.assertEquals(val, param_server.get_param('/new_param//'))
        
        # test full get
        self.assertEquals(full_dict, param_server.get_param('/'))
        
        # test with param in sub-namespace
        val = random.randint(0, 10000)        
        self.failIf(param_server.has_param('/sub/sub2/new_param2'))
        self.assertGetParamFail(param_server, '/sub/sub2/new_param2')
        param_server.set_param('/sub/sub2/new_param2', val)
        full_dict['sub'] = {'sub2': { 'new_param2': val }}
        self.assertEquals(val, param_server.get_param('/sub/sub2/new_param2'))
        # - test homonym
        self.assertEquals(val, param_server.get_param('/sub///sub2/new_param2/'))
        
        # test full get
        self.assertEquals(full_dict, param_server.get_param('/'))

        # test that parameter server namespace-get (#587)
        val1 = random.randint(0, 10000)
        val2 = random.randint(0, 10000)
        val3 = random.randint(0, 10000)
        
        for k in ['/gains/P', '/gains/I', '/gains/D', '/gains']:
            self.assertGetParamFail(param_server, k)
            self.failIf(param_server.has_param(k))

        param_server.set_param('/gains/P', val1)
        param_server.set_param('/gains/I', val2)
        param_server.set_param('/gains/D', val3)        

        pid = {'P': val1, 'I': val2, 'D': val3}
        full_dict['gains'] = pid
        self.assertEquals(pid,
                          param_server.get_param('/gains'))
        self.assertEquals(pid,
                          param_server.get_param('/gains/'))
        self.assertEquals(full_dict,
                          param_server.get_param('/'))

        self.failIf(param_server.has_param('/ns/gains/P'))
        self.failIf(param_server.has_param('/ns/gains/I'))
        self.failIf(param_server.has_param('/ns/gains/D'))
        self.failIf(param_server.has_param('/ns/gains'))
        
        param_server.set_param('/ns/gains/P', val1)
        param_server.set_param('/ns/gains/I', val2)
        param_server.set_param('/ns/gains/D', val3)
        full_dict['ns'] = {'gains': pid}
        
        self.assertEquals(pid,
                          param_server.get_param('/ns/gains'))
        self.assertEquals({'gains': pid},
                          param_server.get_param('/ns/'))
        self.assertEquals({'gains': pid},
                          param_server.get_param('/ns'))
        self.assertEquals(full_dict,
                          param_server.get_param('/'))
        
        
    def test_delete_param(self):
        from rosmaster.paramserver import ParamDictionary
        param_server = ParamDictionary(None)
        try:
            param_server.delete_param('/fake')
            self.fail("delete_param of non-existent should have failed")
        except: pass
        try:
            param_server.delete_param('/')
            self.fail("delete_param of root should have failed")
        except: pass

        param_server.set_param('/foo', 'foo')
        param_server.set_param('/bar', 'bar')        
        self.assert_(param_server.has_param('/foo'))
        self.assert_(param_server.has_param('/bar'))        
        param_server.delete_param('/foo')
        self.failIf(param_server.has_param('/foo'))
        # - test with trailing slash
        param_server.delete_param('/bar/')
        self.failIf(param_server.has_param('/bar'))

        # test with namespaces
        param_server.set_param("/sub/key/x", 1)
        param_server.set_param("/sub/key/y", 2)
        try:
            param_server.delete_param('/sub/key/z')
            self.fail("delete_param of non-existent should have failed")
        except: pass
        try:
            param_server.delete_param('/sub/sub2/z')
            self.fail("delete_param of non-existent should have failed")
        except: pass

        self.assert_(param_server.has_param('/sub/key/x'))
        self.assert_(param_server.has_param('/sub/key/y'))
        self.assert_(param_server.has_param('/sub/key'))                  
        param_server.delete_param('/sub/key')
        self.failIf(param_server.has_param('/sub/key'))      
        self.failIf(param_server.has_param('/sub/key/x'))
        self.failIf(param_server.has_param('/sub/key/y'))

        # test with namespaces (dictionary vals)
        param_server.set_param('/sub2', {'key': { 'x' : 1, 'y' : 2}})
        self.assert_(param_server.has_param('/sub2/key/x'))
        self.assert_(param_server.has_param('/sub2/key/y'))
        self.assert_(param_server.has_param('/sub2/key'))                  
        param_server.delete_param('/sub2/key')
        self.failIf(param_server.has_param('/sub2/key'))      
        self.failIf(param_server.has_param('/sub2/key/x'))
        self.failIf(param_server.has_param('/sub2/key/y'))

        # test with namespaces: treat value as if its a namespace
        # - try to get the dictionary-of-dictionary code to fail
        #   by descending a value key as if it is a namespace
        param_server.set_param('/a', 'b')
        self.assert_(param_server.has_param('/a'))
        try:
            param_server.delete_param('/a/b/c')
            self.fail_("should have raised key error")
        except: pass
        
        
    # test_set_param: test basic set_param behavior. Value encoding verified separately by testParamValues
    def test_set_param(self):
        from rosmaster.paramserver import ParamDictionary
        param_server = ParamDictionary(None)
        caller_id = '/node'
        val = random.randint(0, 10000)

        # verify error behavior with root
        try:
            param_server.set_param('/', 1)
            self.fail("ParamDictionary allowed root to be set to non-dictionary")
        except: pass

        # very similar to has param sequence
        self.failIf(param_server.has_param('/new_param'))
        param_server.set_param('/new_param', val)
        self.assertEquals(val, param_server.get_param('/new_param'))
        self.assertEquals(val, param_server.get_param('/new_param/'))
        self.assert_(param_server.has_param('/new_param'))

        # test with param in sub-namespace
        val = random.randint(0, 10000)        
        self.failIf(param_server.has_param('/sub/sub2/new_param2'))
        param_server.set_param('/sub/sub2/new_param2', val)
        self.assertEquals(val, param_server.get_param('/sub/sub2/new_param2'))

        # test with param type mutation
        vals = ['a', {'a': 'b'}, 1, 1., 'foo', {'c': 'd'}, 4, {'a': {'b': 'c'}}, 3]
        for v in vals:
            param_server.set_param('/multi/multi_param', v)
            self.assertEquals(v, param_server.get_param('/multi/multi_param'))

        # - set value within subtree that mutates higher level value
        param_server.set_param('/multi2/multi_param', 1)
        self.assertEquals(1, param_server.get_param('/multi2/multi_param'))

        param_server.set_param('/multi2/multi_param/a', 2)
        self.assertEquals(2, param_server.get_param('/multi2/multi_param/a'))
        self.assertEquals({'a': 2}, param_server.get_param('/multi2/multi_param/'))        
        param_server.set_param('/multi2/multi_param/a/b', 3)
        self.assertEquals(3, param_server.get_param('/multi2/multi_param/a/b'))
        self.assertEquals({'b': 3}, param_server.get_param('/multi2/multi_param/a/'))
        self.assertEquals({'a': {'b': 3}}, param_server.get_param('/multi2/multi_param/'))        

        
        # test that parameter server namespace-set (#587)
        self.failIf(param_server.has_param('/gains/P'))
        self.failIf(param_server.has_param('/gains/I'))
        self.failIf(param_server.has_param('/gains/D'))                        
        self.failIf(param_server.has_param('/gains'))

        pid = {'P': random.randint(0, 10000), 'I': random.randint(0, 10000), 'D': random.randint(0, 10000)}
        param_server.set_param('/gains', pid)
        self.assertEquals(pid,  param_server.get_param('/gains'))
        self.assertEquals(pid['P'], param_server.get_param('/gains/P'))
        self.assertEquals(pid['I'], param_server.get_param('/gains/I'))
        self.assertEquals(pid['D'], param_server.get_param('/gains/D'))

        subns = {'gains1': pid, 'gains2': pid}
        param_server.set_param('/ns', subns)
        self.assertEquals(pid['P'], param_server.get_param('/ns/gains1/P'))
        self.assertEquals(pid['I'], param_server.get_param('/ns/gains1/I'))
        self.assertEquals(pid['D'], param_server.get_param('/ns/gains1/D'))
        self.assertEquals(pid, param_server.get_param('/ns/gains1'))
        self.assertEquals(pid, param_server.get_param('/ns/gains2'))
        self.assertEquals(subns, param_server.get_param('/ns/'))

        # test empty dictionary set
        param_server.set_param('/ns', {})
        # - param should still exist
        self.assert_(param_server.has_param('/ns/'))
        # - value should remain dictionary
        self.assertEquals({}, param_server.get_param('/ns/'))
        # - value2 below /ns/ should be erased
        self.failIf(param_server.has_param('/ns/gains1'))
        self.failIf(param_server.has_param('/ns/gains1/P'))
        
        # verify that root can be set and that it erases all values
        param_server.set_param('/', {})
        self.failIf(param_server.has_param('/new_param'))
        param_server.set_param('/', {'foo': 1, 'bar': 2, 'baz': {'a': 'a'}})
        self.assertEquals(1, param_server.get_param('/foo'))
        self.assertEquals(1, param_server.get_param('/foo/'))        
        self.assertEquals(2, param_server.get_param('/bar'))
        self.assertEquals(2, param_server.get_param('/bar/'))
        self.assertEquals('a', param_server.get_param('/baz/a'))
        self.assertEquals('a', param_server.get_param('/baz/a/'))

    # test_param_values: test storage of all XML-RPC compatible types"""
    def test_param_values(self):
        import math
        from rosmaster.paramserver import ParamDictionary
        param_server = ParamDictionary(None)
        test_vals = [
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
            ['array', [[], [1, 2, 3], ['a', 'b', 'c'], [0.0, 0.1, 0.2, 2.0, 2.1, -4.0],
                       [1, 'a', True], [[1, 2, 3], ['a', 'b', 'c'], [1.0, 2.1, 3.2]]]
             ],
            ]

        print "Putting parameters onto the server"
        # put our params into the parameter server
        contexts = ['', 'scope1', 'scope/subscope1', 'scope/sub1/sub2']
        my_state = {}
        failures = []
        for ctx in contexts:
            self._set_param(ctx, my_state, test_vals, param_server)
        self._check_param_state(param_server, my_state)
        
        print "Deleting all of our parameters"
        # delete all of our parameters
        param_keys = my_state.keys()
        count = 0
        for key in param_keys:
            count += 1
            param_server.delete_param(key)
            del my_state[key]
            # far too intensive to check every time
            if count % 50 == 0:
                self._check_param_state(param_server, my_state)
        self._check_param_state(param_server, my_state)

    def assertGetParamFail(self, param_server, param):
        try:
            param_server.get_param(param)
            self.fail("get_param[%s] did not raise KeyError"%(param))
        except KeyError: pass

