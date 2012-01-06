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

import os, sys, unittest
import time
from xmlrpclib import ServerProxy

import rosgraph

import roslaunch.child
import roslaunch.server

## Fake RemoteProcess object
class ChildProcessMock(roslaunch.server.ChildROSLaunchProcess):
    def __init__(self, name, args=[], env={}):
        super(ChildProcessMock, self).__init__(name, args, env)
        self.stopped = False
    def stop(self):
        self.stopped = True

## Fake ProcessMonitor object
class ProcessMonitorMock(object):
    def __init__(self):
        self.core_procs = []
        self.procs = []
        self.listeners = []
        
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
        pass
        
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
            
## Test roslaunch.server
class TestRoslaunchChild(unittest.TestCase):

    def setUp(self):
        self.pmon = ProcessMonitorMock()
        try:
            # if there is a core up, we have to use its run id
            m = rosgraph.Master('/roslaunch')
            self.run_id = m.getParam('/run_id')
        except:
            self.run_id = 'foo-%s'%time.time()

    def test_roslaunchChild(self):
        # this is mainly a code coverage test to try and make sure that we don't
        # have any uninitialized references, etc...

        from roslaunch.child import ROSLaunchChild
        
        name = 'child-%s'%time.time()
        server_uri = 'http://unroutable:1234'
        c = ROSLaunchChild(self.run_id, name, server_uri)
        self.assertEquals(self.run_id, c.run_id)
        self.assertEquals(name, c.name)
        self.assertEquals(server_uri, c.server_uri)
        # - this check tests our assumption about c's process monitor field
        self.assertEquals(None, c.pm)
        self.assertEquals(None, c.child_server)        

        # should be a noop
        c.shutdown()

        # create a new child to test _start_pm() and shutdown()
        c = ROSLaunchChild(self.run_id, name, server_uri)
        
        # - test _start_pm and shutdown logic
        c._start_pm()
        self.assert_(c.pm is not None)
        c.shutdown()

        # create a new child to test run() with a fake process
        # monitor. this requires an actual parent server to be running

        import roslaunch.config
        server = roslaunch.server.ROSLaunchParentNode(roslaunch.config.ROSLaunchConfig(), self.pmon)
        # - register a fake child with the server so that it accepts registration from ROSLaunchChild
        server.add_child(name, ChildProcessMock('foo'))
        try:
            server.start()
            self.assert_(server.uri, "server URI did not initialize")
            
            c = ROSLaunchChild(self.run_id, name, server.uri)
            c.pm = self.pmon
            #  - run should speed through
            c.run()
        finally:
            server.shutdown('test done')

        # one final test for code completness: raise an exception during run()
        c = ROSLaunchChild(self.run_id, name, server_uri)
        def bad():
            raise Exception('haha')
        # - violate some encapsulation here just to make sure the exception happens
        c._start_pm = bad
        try:
            # this isn't really a correctness test, this just manually
            # tests that the exception is logged
            c.run()
        except:
            pass
        
        
def kill_parent(p, delay=1.0):
    # delay execution so that whatever pmon method we're calling has time to enter
    import time
    time.sleep(delay)
    print "stopping parent"
    p.shutdown()
        
