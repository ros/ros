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

import roslaunch.pmon
import roslaunch.server
from roslaunch.server import ProcessListener

from xmlrpclib import ServerProxy

import rosgraph
master = rosgraph.Master('test_roslaunch_server')
def get_param(*args):
    return master.getParam(*args)
    
class MockProcessListener(ProcessListener):
    def process_died(self, name, code):
        self.process_name = name
        self.exit_code = code

## Fake Process object
class ProcessMock(roslaunch.pmon.Process):
    def __init__(self, package, name, args, env, respawn=False):
        super(ProcessMock, self).__init__(package, name, args, env, respawn)
        self.stopped = False
    def stop(self):
        self.stopped = True

## Fake ProcessMonitor object
class ProcessMonitorMock(object):
    def __init__(self):
        self.core_procs = []
        self.procs = []
        self.listeners = []
        self.is_shutdown = False
        
    def join(self, timeout=0):
        pass

    def add_process_listener(self, l):
        self.listeners.append(l)

    def register(self, p):
        self.procs.append(p)
        
    def register_core_proc(self, p):
        self.core_procs.append(p)
        
    def registrations_complete(self):
        pass
        
    def unregister(self, p):
        self.procs.remove(p)

    def has_process(self, name):
        return len([p for p in self.procs if p.name == name]) > 0

    def get_process(self, name):
        val = [p for p in self.procs if p.name == name]
        if val:
            return val[0]
        return None

    def has_main_thread_jobs(self):
        return False
    
    def do_main_thread_jobs(self):
        pass
    
    def kill_process(self, name):
        pass
        
    def shutdown(self):
        self.is_shutdown = True
        
    def get_active_names(self):
        return [p.name for p in self.procs]

    def get_process_names_with_spawn_count(self):
        actives = [(p.name, p.spawn_count) for p in self.procs]
        deads = []
        return [actives, deads]

    def mainthread_spin_once(self):
        pass
        
    def mainthread_spin(self):
        pass

    def run(self):
        pass
            
class TestHandler(roslaunch.server.ROSLaunchBaseHandler):
    def __init__(self, pm):
        super(TestHandler, self).__init__(pm)
        self.pinged = None
    def ping(self, val):
        self.pinged = val
        return True

## Test roslaunch.server
class TestRoslaunchServer(unittest.TestCase):

    def setUp(self):
        self.pmon = ProcessMonitorMock()
        roslaunch.core.clear_printlog_handlers()
        roslaunch.core.clear_printerrlog_handlers()

    def test_ChildRoslaunchProcess(self):
        # test that constructor is wired up correctly
        from roslaunch.server import ChildROSLaunchProcess
        name = 'foo-%s'%time.time()
        args = [time.time(), time.time()]
        env = { 'key-%s'%time.time() : str(time.time()) }
        cp = ChildROSLaunchProcess(name, args, env)
        self.assertEquals(name, cp.name)
        self.assertEquals(args, cp.args)
        self.assertEquals(env, cp.env)
        self.assertEquals(None, cp.uri)

        uri = 'http://foo:1234'
        cp.set_uri(uri)
        self.assertEquals(uri, cp.uri)

    def _succeed(self, retval):
        code, msg, val = retval
        self.assertEquals(1, code)
        self.assert_(type(msg) == str)
        return val

    def test_ROSLaunchBaseHandler(self):
        from roslaunch.server import ROSLaunchBaseHandler

        try:
            ROSLaunchBaseHandler(None)
            self.fail("should not allow pm as None")
        except: pass

        pmon = self.pmon
        h = ROSLaunchBaseHandler(pmon)
        self._test_ROSLaunchBaseHandler(h)
        
    # reusable parent class test
    def _test_ROSLaunchBaseHandler(self, h):
        pmon = self.pmon
        # - test get pid
        self.assertEquals(os.getpid(), self._succeed(h.get_pid()))

        # - test list processes
        process_list = self._succeed(h.list_processes())
        self.assertEquals([[], []], process_list)

        p = ProcessMock('pack', 'name', [], {})
        p.spawn_count = 1
        pmon.register(p)
        
        process_list = self._succeed(h.list_processes())
        self.assertEquals([('name', 1),], process_list[0])
        self.assertEquals([], process_list[1])

        p.spawn_count = 2      
        process_list = self._succeed(h.list_processes())
        self.assertEquals([('name', 2),], process_list[0])
        self.assertEquals([], process_list[1])
        # - cleanup our work
        pmon.unregister(p)

        # - test process_info
        code, msg, val = h.process_info('fubar')
        self.assertEquals(-1, code)
        self.assert_(type(msg) == str)
        self.assertEquals({}, val)
        
        pmon.register(p)
        self.assertEquals(p.get_info(), self._succeed(h.process_info(p.name)))
        pmon.unregister(p)

        # - test get_node_names
        #   - reach into instance to get branch-complete
        h.pm = None
        code, msg, val = h.get_node_names()
        self.assertEquals(0, code)
        self.assert_(type(msg) == str)
        self.assertEquals([], val)
        h.pm = pmon
        
        self.assertEquals(pmon.get_active_names(), self._succeed(h.get_node_names()))
        pmon.register(p)
        self.assertEquals(pmon.get_active_names(), self._succeed(h.get_node_names()))        
        pmon.unregister(p)

    def test_ROSLaunchParentHandler(self):
        from roslaunch.server import ROSLaunchParentHandler
        from roslaunch.server import ChildROSLaunchProcess
        try:
            ROSLaunchParentHandler(None, {}, [])
            self.fail("should not allow pm as None")
        except: pass

        pmon = self.pmon
        child_processes = {}
        listeners = []
        h = ROSLaunchParentHandler(pmon, child_processes, listeners)
        self.assertEquals(child_processes, h.child_processes)
        self.assertEquals(listeners, h.listeners)
        self._test_ROSLaunchBaseHandler(h)

        from rosgraph_msgs.msg import Log
        h.log('client-1', Log.FATAL, "message")
        h.log('client-1', Log.ERROR, "message")
        h.log('client-1', Log.DEBUG, "message")        
        h.log('client-1', Log.INFO, "started with pid 1234")

        # test process_died
        # - no listeners
        self._succeed(h.process_died('dead1', -1))
        # - well-behaved listener
        l = roslaunch.pmon.ProcessListener()
        listeners.append(l)
        self._succeed(h.process_died('dead2', -1))
        # - bad listener with exception
        def bad():
            raise Exception("haha")
        l.process_died = bad
        self._succeed(h.process_died('dead3', -1))

        # test register
        
        # - verify clean slate with list_children
        self.assertEquals([], self._succeed(h.list_children()))

        # - first register with unknown
        code, msg, val = h.register('client-unknown', 'http://unroutable:1234')
        self.assertEquals(-1, code)
        self.assertEquals([], self._succeed(h.list_children()))
        
        # - now register with known
        uri = 'http://unroutable:1324'
        child_processes['client-1'] = ChildROSLaunchProcess('foo', [], {})
        val = self._succeed(h.register('client-1', uri))
        self.assert_(type(val) == int)
        self.assertEquals([uri], self._succeed(h.list_children()))        
        
    def test_ROSLaunchChildHandler(self):
        from roslaunch.server import ROSLaunchChildHandler
        pmon = self.pmon
        try:
            # if there is a core up, we have to use its run id
            run_id = get_param('/run_id')
        except:
            run_id = 'foo-%s'%time.time()
        name = 'foo-bob'
        server_uri = 'http://localhost:12345'
        try:
            h = ROSLaunchChildHandler(run_id, name, server_uri, None)
            self.fail("should not allow pm as None")
        except: pass
        try:
            h = ROSLaunchChildHandler(run_id, name, 'http://bad_uri:0', pmon)
            self.fail("should not allow bad uri")
        except: pass
        try:
            h = ROSLaunchChildHandler(run_id, name, None, pmon)
            self.fail("should not allow None server_uri")
        except: pass

        h = ROSLaunchChildHandler(run_id, name, server_uri, pmon)
        self.assertEquals(run_id, h.run_id)
        self.assertEquals(name, h.name)        
        self.assertEquals(server_uri, h.server_uri)
        self._test_ROSLaunchBaseHandler(h)

        # test _log()
        from rosgraph_msgs.msg import Log
        h._log(Log.INFO, 'hello')

        # test shutdown()
        # should uninitialize pm
        h.shutdown()
        self.assert_(pmon.is_shutdown)
        # - this check is mostly to make sure that the launch() call below will exit
        self.assert_(h.pm is None)
        code, msg, val = h.launch('<launch></launch>')
        self.assertEquals(0, code)
        
        # TODO: actual launch. more difficult as we need a core

    def test__ProcessListenerForwarder(self):
        from roslaunch.server import _ProcessListenerForwarder, ProcessListener
        
        # test with bad logic first
        f = _ProcessListenerForwarder(None)
        # - should trap exceptions
        f.process_died("foo", -1)
        # test with actual listener
        l = MockProcessListener()
        f = _ProcessListenerForwarder(l)
        # - should run normally
        f.process_died("foo", -1)
        
        self.assertEquals(l.process_name, 'foo')
        self.assertEquals(l.exit_code, -1)

    def test_ROSLaunchNode(self):
        #exercise the basic ROSLaunchNode API
        from roslaunch.server import ROSLaunchNode

        # - create a node with a handler
        handler = TestHandler(self.pmon)
        node = ROSLaunchNode(handler)

        # - start the node
        node.start()
        self.assert_(node.uri)

        # - call the ping API that we added
        s = ServerProxy(node.uri)
        test_val = 'test-%s'%time.time()
        s.ping(test_val)
        self.assertEquals(handler.pinged, test_val)

        # - call the pid API
        code, msg, pid = s.get_pid()
        self.assertEquals(1, code)
        self.assert_(type(msg) == str)
        self.assertEquals(os.getpid(), pid)
        
        # - shut it down
        node.shutdown('test done')
        
    def test_ROSLaunchParentNode(self):
        from roslaunch.server import ROSLaunchParentNode
        from roslaunch.server import ChildROSLaunchProcess
        from roslaunch.config import ROSLaunchConfig
        from roslaunch.pmon import ProcessListener
        rosconfig = ROSLaunchConfig()
        try:
            ROSLaunchParentNode(rosconfig, None)
            self.fail("should not allow pm as None")
        except: pass
        pmon = self.pmon
        n = ROSLaunchParentNode(rosconfig, pmon)
        self.assertEquals(rosconfig, n.rosconfig)
        self.assertEquals([], n.listeners)
        self.assertEquals({}, n.child_processes)
        self.assertEquals(n.handler.listeners, n.listeners)
        self.assertEquals(n.handler.child_processes, n.child_processes)

        # test add listener
        self.assertEquals(n.handler.listeners, n.listeners)
        l = ProcessListener() 
        n.add_process_listener(l)
        self.assertEquals([l], n.listeners)
        self.assertEquals(n.handler.listeners, n.listeners)

        # now, lets make some xmlrpc calls against it
        import roslaunch.config
        server = roslaunch.server.ROSLaunchParentNode(roslaunch.config.ROSLaunchConfig(), self.pmon)

        # it's really dangerous for logging when both a parent and a
        # child are in the same process, so out of paranoia, clear the
        # logging handlers
        roslaunch.core.clear_printlog_handlers()
        roslaunch.core.clear_printerrlog_handlers()
        
        # - register a fake child with the server so that it accepts registration from ROSLaunchChild
        child_name = 'child-%s'%time.time()
        child_proc = ChildROSLaunchProcess('foo', [], {})
        server.add_child(child_name, child_proc)
        
        try:
            server.start()
            self.assert_(server.uri, "server URI did not initialize")
            s = ServerProxy(server.uri)
            child_uri = 'http://fake-unroutable:1324'
            # - list children should be empty
            val = self._succeed(s.list_children())
            self.assertEquals([], val)
            # - register
            val = self._succeed(s.register(child_name, child_uri))
            self.assertEquals(1, val)
            # - list children
            val = self._succeed(s.list_children())
            self.assertEquals([child_uri], val)
        finally:
            server.shutdown('test done')
        
        
        
    def test_ROSLaunchChildNode(self):
        from roslaunch.server import ROSLaunchChildNode
        from roslaunch.server import ChildROSLaunchProcess
        pmon = self.pmon
        try:
            # if there is a core up, we have to use its run id
            run_id = get_param('/run_id')
        except:
            run_id = 'foo-%s'%time.time()
        name = 'foo-bob'
        server_uri = 'http://localhost:12345'
        try:
            ROSLaunchChildNode(run_id, name, server_uri, None)
            self.fail("should not allow pm as None")
        except: pass
        try:
            ROSLaunchChildNode(run_id, name, 'http://bad_uri:0', pmon)
            self.fail("should not allow bad uri")
        except: pass

        n = ROSLaunchChildNode(run_id, name, server_uri, pmon)
        self.assertEquals(run_id, n.run_id)
        self.assertEquals(name, n.name)        
        self.assertEquals(server_uri, n.server_uri)

        # tests for actual registration with server
        import roslaunch.config
        server = roslaunch.server.ROSLaunchParentNode(roslaunch.config.ROSLaunchConfig(), self.pmon)
        # - register a fake child with the server so that it accepts registration from ROSLaunchChild
        child_proc = ChildROSLaunchProcess('foo', [], {})
        server.add_child(name, child_proc)
        
        try:
            server.start()
            self.assert_(server.uri, "server URI did not initialize")
            s = ServerProxy(server.uri)

            print "SERVER STARTED"
            
            # start the child
            n = ROSLaunchChildNode(run_id, name, server.uri, pmon)            
            n.start()
            print "CHILD STARTED"            
            self.assert_(n.uri, "child URI did not initialize")            

            # verify registration
            print "VERIFYING REGISTRATION"                        
            self.assertEquals(child_proc.uri, n.uri)
            child_uri = 'http://fake-unroutable:1324'
            # - list children
            val = self._succeed(s.list_children())
            self.assertEquals([n.uri], val)
        finally:
            print "SHUTTING DOWN"                                    
            n.shutdown('test done')
            server.shutdown('test done')
        
