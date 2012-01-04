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
import thread

import roslaunch.server

## handy class for kill_pmon to check whether it was called
class Marker(object):
    def __init__(self):
        self.marked = False
    def mark(self):
        self.marked = True

## Fake ProcessMonitor object
class ProcessMonitorMock(object):
    def __init__(self):
        self.core_procs = []
        self.procs = []
        self.listeners = []
        self.alive = False
        self.is_shutdown = False
        
    def isAlive(self):
        return self.alive

    def join(self, *args):
        return

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
        return self.procs.get(p, None)

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
        retval = [actives, deads]

    def mainthread_spin_once(self):
        pass
        
    def mainthread_spin(self):
        pass

    def run(self):
        pass
            
class ProcessMock(roslaunch.pmon.Process):
    def __init__(self, package, name, args, env, respawn=False):
        super(ProcessMock, self).__init__(package, name, args, env, respawn)
        self.stopped = False
    def stop(self, errors):
        self.stopped = True
        
class RespawnOnceProcessMock(ProcessMock):
    def __init__(self, package, name, args, env, respawn=False):
        super(ProcessMock, self).__init__(package, name, args, env, respawn)
        self.spawn_count = 0

    def is_alive(self):
        return False
    
    def start(self):
        self.spawn_count += 1
        if self.spawn_count > 1:
            self.respawn = False

## Test roslaunch.server
class TestRoslaunchPmon(unittest.TestCase):

    def setUp(self):
        self.pmon = roslaunch.pmon.ProcessMonitor()

    ## test all apis of Process instance. part coverage/sanity test
    def _test_Process(self, p, package, name, args, env, respawn):
        self.assertEquals(package, p.package)
        self.assertEquals(name, p.name)
        self.assertEquals(args, p.args)        
        self.assertEquals(env, p.env)  
        self.assertEquals(respawn, p.respawn)
        self.assertEquals(0, p.spawn_count)        
        self.assertEquals(None, p.exit_code)
        self.assert_(p.get_exit_description())
        self.failIf(p.is_alive())

        info = p.get_info()
        self.assertEquals(package, info['package'])
        self.assertEquals(name, info['name'])
        self.assertEquals(args, info['args'])        
        self.assertEquals(env, info['env'])  
        self.assertEquals(respawn, info['respawn'])
        self.assertEquals(0, info['spawn_count'])        
        self.failIf('exit_code' in info)

        p.start()
        self.assertEquals(1, p.spawn_count)
        self.assertEquals(1, p.get_info()['spawn_count']) 
        p.start()        
        self.assertEquals(2, p.spawn_count)
        self.assertEquals(2, p.get_info()['spawn_count'])

        # noop
        p.stop()

        p.exit_code = 0
        self.assertEquals(0, p.get_info()['exit_code'])
        self.assert_('cleanly' in p.get_exit_description())
        p.exit_code = 1
        self.assertEquals(1, p.get_info()['exit_code'])                        
        self.assert_('[exit code 1]' in p.get_exit_description())

    ## tests to make sure that our Process base class has 100% coverage
    def test_Process(self):
        from roslaunch.pmon import Process
        # test constructor params
        respawn = False
        package = 'foo-%s'%time.time()
        name = 'name-%s'%time.time()
        args = [time.time(), time.time(), time.time()]
        env = { 'key': time.time(), 'key2': time.time() } 

        p = Process(package, name, args, env)
        self._test_Process(p, package, name, args, env, False)
        p = Process(package, name, args, env, True)
        self._test_Process(p, package, name, args, env, True) 
        p = Process(package, name, args, env, False)
        self._test_Process(p, package, name, args, env, False) 
        
    def _test_DeadProcess(self, p0, package, name, args, env, respawn):
        from roslaunch.pmon import DeadProcess
        p0.exit_code = -1
        dp = DeadProcess(p0)
        self.assertEquals(package, dp.package)
        self.assertEquals(name, dp.name)
        self.assertEquals(args, dp.args)        
        self.assertEquals(env, dp.env)  
        self.assertEquals(respawn, dp.respawn)
        self.assertEquals(0, dp.spawn_count)        
        self.assertEquals(-1, dp.exit_code)
        self.failIf(dp.is_alive())

        info = dp.get_info()
        info0 = p0.get_info()
        self.assertEquals(info0['package'], info['package'])
        self.assertEquals(info0['name'], info['name'])
        self.assertEquals(info0['args'], info['args'])        
        self.assertEquals(info0['env'], info['env'])  
        self.assertEquals(info0['respawn'], info['respawn'])
        self.assertEquals(0, info['spawn_count'])        

        try:
            dp.start()
            self.fail("should not be able to start a dead process")
        except: pass
            
        # info should be frozen
        p0.package = 'dead package'
        p0.name = 'dead name'        
        p0.spawn_count = 1
        self.assertEquals(package, dp.package)
        self.assertEquals(name, dp.name)
        self.assertEquals(0, dp.spawn_count)                
        self.assertEquals(package, dp.get_info()['package']) 
        self.assertEquals(name, dp.get_info()['name']) 
        self.assertEquals(0, dp.get_info()['spawn_count']) 
        p0.start()        
        self.assertEquals(0, dp.spawn_count)
        self.assertEquals(0, dp.get_info()['spawn_count'])

        # noop
        p0.stop()


    def test_DeadProcess(self):
        from roslaunch.pmon import Process, DeadProcess
        # test constructor params
        respawn = False
        package = 'foo-%s'%time.time()
        name = 'name-%s'%time.time()
        args = [time.time(), time.time(), time.time()]
        env = { 'key': time.time(), 'key2': time.time() } 

        p = Process(package, name, args, env)
        self._test_DeadProcess(p, package, name, args, env, False)
        p = Process(package, name, args, env, True)
        self._test_DeadProcess(p, package, name, args, env, True) 
        p = Process(package, name, args, env, False)
        self._test_DeadProcess(p, package, name, args, env, False) 

    def test_start_shutdown_process_monitor(self):
        def failer():
            raise Exception("oops")
        # noop
        self.failIf(roslaunch.pmon.shutdown_process_monitor(None))

        # test with fake pmon so we can get branch-complete
        pmon = ProcessMonitorMock()
        # - by setting alive to true, shutdown fails, though it can't really do anything about it
        pmon.alive = True
        self.failIf(roslaunch.pmon.shutdown_process_monitor(pmon))
        
        # make sure that exceptions get trapped        
        pmon.shutdown = failer
        # should cause an exception during execution, but should get caught 
        self.failIf(roslaunch.pmon.shutdown_process_monitor(pmon))
        
        # Test with a real process monitor
        pmon = roslaunch.pmon.start_process_monitor()
        self.assert_(pmon.isAlive())
        self.assert_(roslaunch.pmon.shutdown_process_monitor(pmon))
        self.failIf(pmon.isAlive())

        # fiddle around with some state that would shouldn't be
        roslaunch.pmon._shutting_down = True
        pmon = roslaunch.pmon.start_process_monitor()
        if pmon != None:
            self.failIf(roslaunch.pmon.shutdown_process_monitor(pmon))
            self.fail("start_process_monitor should fail if during shutdown sequence")
            
    def test_pmon_shutdown(self):
        # should be noop
        roslaunch.pmon.pmon_shutdown()
        
        # start two real process monitors and kill them
        # pmon_shutdown
        pmon1 = roslaunch.pmon.start_process_monitor()
        pmon2 = roslaunch.pmon.start_process_monitor()
        self.assert_(pmon1.isAlive())
        self.assert_(pmon2.isAlive())        

        roslaunch.pmon.pmon_shutdown()
        
        self.failIf(pmon1.isAlive())
        self.failIf(pmon2.isAlive())        
        
    def test_add_process_listener(self):
        # coverage test, not a functionality test as that would be much more difficult to simulate
        from roslaunch.pmon import ProcessListener
        l = ProcessListener()
        self.pmon.add_process_listener(l)

    def test_kill_process(self):
        from roslaunch.core import RLException
        pmon = self.pmon

        # should return False
        self.failIf(pmon.kill_process('foo'))
        
        p1 = ProcessMock('foo', 'name1', [], {})
        p2 = ProcessMock('bar', 'name2', [], {})
        pmon.register(p1)
        pmon.register(p2)        
        self.failIf(p1.stopped)
        self.failIf(p2.stopped)
        self.assert_(p1.name in pmon.get_active_names())
        self.assert_(p2.name in pmon.get_active_names())

        # should fail as pmon API is string-based
        try:
            self.assert_(pmon.kill_process(p1))
            self.fail("kill_process should have thrown RLException")
        except RLException: pass
        
        self.assert_(pmon.kill_process(p1.name))
        self.assert_(p1.stopped)
        
        # - pmon should not have removed yet as the pmon thread cannot catch the death
        self.assert_(pmon.has_process(p1.name))
        self.assert_(p1.name in pmon.get_active_names())
        
        self.failIf(p2.stopped)        
        self.assert_(p2.name in pmon.get_active_names())       
        pmon.kill_process(p2.name)
        self.assert_(p2.stopped)
        
        # - pmon should not have removed yet as the pmon thread cannot catch the death
        self.assert_(pmon.has_process(p2.name))
        self.assert_(p2.name in pmon.get_active_names())

        p3 = ProcessMock('bar', 'name3', [], {})
        def bad(x):
            raise Exception("ha ha ha")
        p3.stop = bad
        pmon.register(p3)
        # make sure kill_process is safe
        pmon.kill_process(p3.name)
        def f():
            return False

        p1.is_alive = f
        p2.is_alive = f        
        p3.is_alive = f

        # Now that we've 'killed' all the processes, we should be able
        # to run through the ProcessMonitor run loop and it should
        # exit.  But first, important that we check that pmon has no
        # other extra state in it
        self.assertEquals(3, len(pmon.get_active_names()))

        # put pmon into exitable state
        pmon.registrations_complete()

        # and run it -- but setup a safety timer to kill it if it doesn't exit
        marker = Marker()
        thread.start_new_thread(kill_pmon, (self.pmon,marker, 10.))
        
        pmon.run()
        
        self.failIf(marker.marked, "pmon had to be externally killed")

        
        self.assert_(pmon.done)
        self.failIf(pmon.has_process(p1.name))
        self.failIf(pmon.has_process(p2.name))
        alive, dead = pmon.get_process_names_with_spawn_count()
        self.failIf(alive)
        self.assert_((p1.name, p1.spawn_count) in dead)
        self.assert_((p2.name, p2.spawn_count) in dead)


    def test_run(self):
        # run is really hard to test...
        pmon = self.pmon

        # put pmon into exitable state
        pmon.registrations_complete()

        # give the pmon a process that raises an exception when it's
        # is_alive is checked. this should be marked as a dead process
        p1 = ProcessMock('bar', 'name1', [], {})
        def bad():
            raise Exception('ha ha')
        p1.is_alive = bad
        pmon.register(p1)
        
        # give pmon a well-behaved but dead process
        p2 = ProcessMock('bar', 'name2', [], {})        
        def f():
            return False
        p2.is_alive = f
        pmon.register(p2)

        # give pmon a process that wants to respawn once
        p3 = RespawnOnceProcessMock('bar', 'name3', [], {})        
        pmon.register(p3)
        
        # test assumptions about pmon's internal data structures
        # before we begin test
        self.assert_(p1 in pmon.procs)
        self.assert_(pmon._registrations_complete)
        self.failIf(pmon.is_shutdown)
        
        # and run it -- but setup a safety timer to kill it if it doesn't exit
        marker = Marker()
        thread.start_new_thread(kill_pmon, (self.pmon,marker, 10.))
        
        pmon.run()
        
        self.failIf(marker.marked, "pmon had to be externally killed")        

        # retest assumptions
        self.failIf(pmon.procs)
        self.assert_(pmon.is_shutdown)        

        pmon.is_shutdown = False
                
    def test_get_process_names_with_spawn_count(self):
        p1 = ProcessMock('foo', 'name1', [], {})
        p2 = ProcessMock('bar', 'name2', [], {})

        pmon = self.pmon
        self.assertEquals([[], []], pmon.get_process_names_with_spawn_count())
        pmon.register(p1)
        self.assertEquals([[('name1', 0),], []], pmon.get_process_names_with_spawn_count())        
        pmon.register(p2)
        alive, dead = pmon.get_process_names_with_spawn_count()
        self.assertEquals([], dead)
        self.assert_(('name1', 0) in alive)
        self.assert_(('name2', 0) in alive)        
        
        import random
        p1.spawn_count = random.randint(1, 10000)
        p2.spawn_count = random.randint(1, 10000)
        
        alive, dead = pmon.get_process_names_with_spawn_count()
        self.assertEquals([], dead)
        self.assert_((p1.name, p1.spawn_count) in alive)
        self.assert_((p2.name, p2.spawn_count) in alive)

        #TODO figure out how to test dead_list

        
    ## Tests ProcessMonitor.register(), unregister(), has_process(), and get_process()
    def test_registration(self):
        from roslaunch.core import RLException
        from roslaunch.pmon import Process
        pmon = self.pmon
        
        p1 = Process('foo', 'name1', [], {})
        p2 = Process('bar', 'name2', [], {})
        corep1 = Process('core', 'core1', [], {})        
        corep2 = Process('core', 'core2', [], {})        

        pmon.register(p1)
        self.assert_(pmon.has_process('name1'))
        self.assertEquals(p1, pmon.get_process('name1'))
        self.failIf(pmon.has_process('name2'))
        self.assertEquals(['name1'], pmon.get_active_names())
        try:
            pmon.register(Process('foo', p1.name, [], {}))
            self.fail("should not allow duplicate process name")
        except RLException: pass
        
        pmon.register(p2)
        self.assert_(pmon.has_process('name2'))
        self.assertEquals(p2, pmon.get_process('name2'))
        self.assertEquals(set(['name1', 'name2']), set(pmon.get_active_names()))
        
        pmon.register_core_proc(corep1)
        self.assert_(pmon.has_process('core1'))
        self.assertEquals(corep1, pmon.get_process('core1'))
        self.assertEquals(set(['name1', 'name2', 'core1']), set(pmon.get_active_names()))        

        pmon.register_core_proc(corep2)
        self.assert_(pmon.has_process('core2'))
        self.assertEquals(corep2, pmon.get_process('core2'))
        self.assertEquals(set(['name1', 'name2', 'core1', 'core2']), set(pmon.get_active_names()))                


        pmon.unregister(p2)
        self.failIf(pmon.has_process('name2'))                
        pmon.unregister(p1)
        self.failIf(pmon.has_process('name1'))        
        pmon.unregister(corep1)
        self.failIf(pmon.has_process('core1'))                
        pmon.unregister(corep2)
        self.failIf(pmon.has_process('core2'))

        pmon.shutdown()
        try:
            pmon.register(Process('shutdown_fail', 'shutdown_fail', [], {}))
            self.fail("registration should fail post-shutdown")
        except RLException: pass
                      
        
        
    def test_mainthread_spin_once(self):
        # shouldn't do anything
        self.pmon.done = False
        self.pmon.mainthread_spin_once()
        self.pmon.done = True        
        self.pmon.mainthread_spin_once()

    def test_mainthread_spin(self):
        # can't test actual spin as that would go forever
        self.pmon.done = False
        thread.start_new_thread(kill_pmon, (self.pmon,Marker()))
        self.pmon.mainthread_spin()

def kill_pmon(pmon, marker, delay=1.0):
    # delay execution so that whatever pmon method we're calling has time to enter
    time.sleep(delay)
    if not pmon.is_shutdown:
        marker.mark()
    print "stopping pmon"
    # pmon has two states that need to be set, as many of the tests do not start the actual process monitor
    pmon.shutdown()
    pmon.done = True
