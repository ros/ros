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
# Revision $Id$

"""
Process monitor
"""

from __future__ import with_statement

import os
import sys
import time
import traceback
try:
    from queue import Empty, Queue
except ImportError:
    from Queue import Empty, Queue
import atexit
from threading import Thread, RLock, Lock

from .core import printlog, printlog_bold, printerrlog

class PmonException(Exception): pass

class FatalProcessLaunch(PmonException):
    """
    Exception to indicate that a process launch has failed in a fatal
    manner (i.e. relaunch is unlikely to succeed)
    """
    pass

# start/shutdown ################################################

_pmons = []
_pmon_counter = 0
_shutting_down = False
def start_process_monitor():
    global _pmon_counter
    if _shutting_down:
        return None
    _pmon_counter += 1
    name = "ProcessMonitor-%s"%_pmon_counter
    process_monitor = ProcessMonitor(name)
    with _shutdown_lock:
        # prevent race condition with pmon_shutdown() being triggered
        # as we are starting a ProcessMonitor (i.e. user hits ctrl-C
        # during startup)
        _pmons.append(process_monitor)
        process_monitor.start()
    return process_monitor

def shutdown_process_monitor(process_monitor):
    """
    @param process_monitor: process monitor to kill
    @type  process_monitor: L{ProcessMonitor}
    @return: True if process_monitor was successfully
    shutdown. False if it could not be shutdown cleanly or if there is
    a problem with process_monitor
    parameter. shutdown_process_monitor() does not throw any exceptions
    as this is shutdown-critical code.
    @rtype: bool
    """
    try:
        if process_monitor is None or process_monitor.is_shutdown:
            return False
        
        process_monitor.shutdown()
        process_monitor.join(20.0)
        if process_monitor.isAlive():
            return False
        else:
            return True
    except Exception as e:
        return False

_shutdown_lock = Lock()
def pmon_shutdown():
    global _pmons
    with _shutdown_lock:
        if not _pmons:
            return
        for p in _pmons:
            shutdown_process_monitor(p)
        del _pmons[:]

atexit.register(pmon_shutdown)

# ##############################################################

class Process(object):
    """
    Basic process representation for L{ProcessMonitor}. Must be subclassed
    to provide actual start()/stop() implementations.
    """

    def __init__(self, package, name, args, env, respawn=False, required=False):
        self.package = package
        self.name = name
        self.args = args
        self.env = env
        self.respawn = respawn
        self.required = required
        self.lock = Lock()
        self.exit_code = None
        # for keeping track of respawning
        self.spawn_count = 0

    def __str__(self):
        return "Process<%s>"%(self.name)

    # NOTE: get_info() is going to have to be sufficient for
    # generating respawn requests, so we must be complete about it
        
    def get_info(self):
        """
        Get all data about this process in dictionary form
        @return: dictionary of all relevant process properties
        @rtype: dict { str: val }
        """
        info = {
            'spawn_count': self.spawn_count,
            'args': self.args,
            'env': self.env,
            'package': self.package,
            'name': self.name,
            'alive': self.is_alive(),
            'respawn': self.respawn,
            'required': self.required,
            }
        if self.exit_code is not None:
            info['exit_code'] = self.exit_code
        return info

    def start(self):
        self.spawn_count += 1

    def is_alive(self):
        return False

    def stop(self, errors=[]):
        """
        Stop the process. Record any significant error messages in the errors parameter
        
        @param errors: error messages. stop() will record messages into this list.
        @type  errors: [str]
        """
        pass

    def get_exit_description(self):
        if self.exit_code is not None:
            if self.exit_code:
                return 'process has died [exit code %s]'%self.exit_code
            else:
                # try not to scare users about process exit
                return 'process has finished cleanly'
        else:
            return 'process has died'

class DeadProcess(Process):
    """
    Container class to maintain information about a process that has died. This
    container allows us to delete the actual Process but still maintain the metadata
    """
    def __init__(self, p):
        super(DeadProcess, self).__init__(p.package, p.name, p.args, p.env, p.respawn)
        self.exit_code = p.exit_code
        self.lock = None
        self.spawn_count = p.spawn_count
        self.info = p.get_info()
    def get_info(self):
        return self.info
    def start(self):
        raise Exception("cannot call start on a dead process!")
    def is_alive(self):
        return False

class ProcessListener(object):
    """
    Listener class for L{ProcessMonitor}
    """
    
    def process_died(self, process_name, exit_code):
        """
        Notifies listener that process has died. This callback only
        occurs for processes that die during normal process monitor
        execution -- processes that are forcibly killed during
        ProcessMonitor shutdown are not reported.
        @param process_name: name of process
        @type  process_name: str
        @param exit_code: exit code of process. If None, it means
        that ProcessMonitor was unable to determine an exit code.
        @type  exit_code: int
        """
        pass
    
class ProcessMonitor(Thread):

    def __init__(self, name="ProcessMonitor"):
        Thread.__init__(self, name=name)
        self.procs = []
        self.plock = RLock()
        self.is_shutdown = False
        self.done = False        
        self.setDaemon(True)
        self.listeners = []
        self.dead_list = []
        # #885: ensure core procs
        self.core_procs = []
        # #642: flag to prevent process monitor exiting prematurely
        self._registrations_complete = False
        
    def add_process_listener(self, l):
        """
        Listener for process events. MUST be called before
        ProcessMonitor is running.See ProcessListener class.
        @param l: listener instance
        @type  l: L{ProcessListener}
        """
        self.listeners.append(l)

    def register(self, p):
        """
        Register process with L{ProcessMonitor}
        @param p: Process
        @type  p: L{Process}
        @raise PmonException: if process with same name is already registered
        """
        e = None
        with self.plock:
            if self.has_process(p.name):
                e = PmonException("cannot add process with duplicate name '%s'"%p.name)
            elif self.is_shutdown:
                e = PmonException("cannot add process [%s] after process monitor has been shut down"%p.name)
            else:
                self.procs.append(p)
        if e:
            raise e

    def register_core_proc(self, p):
        """
        Register core process with ProcessMonitor. Coreprocesses
        have special shutdown semantics. They are killed after all
        other processes, in reverse order in which they are added.
        @param p Process
        @type  p: L{Process}
        @raise PmonException: if process with same name is already registered
        """
        self.register(p)
        self.core_procs.append(p)
        
    def registrations_complete(self):
        """
        Inform the process monitor that registrations are complete.
        After the registrations_complete flag is set, process monitor
        will exit if there are no processes left to monitor.
        """
        self._registrations_complete = True
        
    def unregister(self, p):
        with self.plock:
            self.procs.remove(p)

    def has_process(self, name):
        """
        @return: True if process is still be monitored. If False, process
        has died or was never registered with process
        @rtype: bool
        """
        return len([p for p in self.procs if p.name == name]) > 0

    def get_process(self, name):
        """
        @return: process registered under \a name, or None
        @rtype: L{Process}
        """
        with self.plock:
            v = [p for p in self.procs if p.name == name]
        if v:
            return v[0]

    def kill_process(self, name):
        """
        Kill process that matches name. NOTE: a killed process will
        continue to show up as active until the process monitor thread
        has caught that it has died.
        @param name: Process name
        @type  name: str
        @return: True if a process named name was removed from
        process monitor. A process is considered killed if its stop()
        method was called.
        @rtype: bool
        """
        if not isinstance(name, basestring):
            raise PmonException("kill_process takes in a process name but was given: %s"%name)
        printlog("[%s] kill requested"%name)
        with self.plock:
            p = self.get_process(name)
            if p:
                try:
                    # no need to accumulate errors, so pass in []
                    p.stop([])
                except Exception as e:
                    printerrlog("Exception: %s"%(str(e)))
                return True
            else:
                return False
        
    def shutdown(self):
        """
        Shutdown the process monitor thread
        """
        self.is_shutdown = True
        
    def get_active_names(self):
        """
        @return [str]: list of active process names
        """
        with self.plock:
            retval = [p.name for p in self.procs]
        return retval

    def get_process_names_with_spawn_count(self):
        """
        @return: Two lists, where first
        list of active process names along with the number of times
        that process has been spawned. Second list contains dead process names
        and their spawn count.
        @rtype: [[(str, int),], [(str,int),]]
        """
        with self.plock:
            actives = [(p.name, p.spawn_count) for p in self.procs]
            deads = [(p.name, p.spawn_count) for p in self.dead_list]
            retval = [actives, deads]
        return retval

    def run(self):
        """
        thread routine of the process monitor. 
        """
        try:
            #don't let exceptions bomb thread, interferes with exit
            try:
                self._run()
            except:
                traceback.print_exc()
        finally:
            self._post_run()
            
    def _run(self):
        """
        Internal run loop of ProcessMonitor
        """
        plock = self.plock
        dead = []
        respawn = []
        while not self.is_shutdown:
            with plock: #copy self.procs
                procs = self.procs[:]
            if self.is_shutdown:
                break

            for p in procs:
                try:
                    if not p.is_alive():
                        exit_code_str = p.get_exit_description()
                        if p.respawn:
                            printlog_bold("[%s] %s\nrespawning..."%(p.name, exit_code_str))
                            respawn.append(p)
                        elif p.required:
                            printerrlog('='*80+"REQUIRED process [%s] has died!\n%s\nInitiating shutdown!\n"%(p.name, exit_code_str)+'='*80)
                            self.is_shutdown = True
                        else:
                            if p.exit_code:
                                printerrlog("[%s] %s"%(p.name, exit_code_str))
                            else:
                                printlog_bold("[%s] %s"%(p.name, exit_code_str))
                            dead.append(p)
                            
                        ## no need for lock as we require listeners be
                        ## added before process monitor is launched
                        for l in self.listeners:
                            l.process_died(p.name, p.exit_code)

                except Exception as e:
                    traceback.print_exc()
                    #don't respawn as this is an internal error
                    dead.append(p)
                if self.is_shutdown:
                    break #stop polling
            for d in dead:
                try:
                    self.unregister(d)
                    # stop process, don't accumulate errors
                    d.stop([])

                    # save process data to dead list 
                    with plock:
                        self.dead_list.append(DeadProcess(d))
                except Exception as e:
                    printerrlog("Exception: %s"%(str(e)))
                    
            # dead check is to make sure that ProcessMonitor at least
            # waits until its had at least one process before exiting
            if self._registrations_complete and dead and not self.procs and not respawn:
                printlog("all processes on machine have died, roslaunch will exit")
                self.is_shutdown = True
            del dead[:]
            for r in respawn: 
                try:
                    if self.is_shutdown:
                        break
                    printlog("[%s] restarting process"%r.name)
                    # stop process, don't accumulate errors
                    r.stop([])
                    r.start()
                except:
                    traceback.print_exc()
            del respawn[:]
            time.sleep(0.1) #yield thread
        #moved this to finally block of _post_run
        #self._post_run() #kill all processes

    def _post_run(self):
        # this is already true entering, but go ahead and make sure
        self.is_shutdown = True
        # killall processes on run exit

        q = Queue()
        q.join()
        
        with self.plock:
            # make copy of core_procs for threadsafe usage
            core_procs = self.core_procs[:]

            # enqueue all non-core procs in reverse order for parallel kill
            # #526/885: ignore core procs
            [q.put(p) for p in reversed(self.procs) if not p in core_procs]

        # use 10 workers
        killers = []
        for i in range(10):
            t = _ProcessKiller(q, i)
            killers.append(t)
            t.start()

        # wait for workers to finish
        q.join()
        shutdown_errors = []

        # accumulate all the shutdown errors
        for t in killers:
            shutdown_errors.extend(t.errors)
        del killers[:]
            
        # #526/885: kill core procs last
        # we don't want to parallelize this as the master has to be last
        for p in reversed(core_procs):
            _kill_process(p, shutdown_errors)

        # delete everything except dead_list
        with self.plock:
            del core_procs[:]
            del self.procs[:]
            del self.core_procs[:]
            
        self.done = True

        if shutdown_errors:
            printerrlog("Shutdown errors:\n"+'\n'.join([" * %s"%e for e in shutdown_errors]))

def _kill_process(p, errors):
    """
    Routine for kill Process p with appropriate logging to screen and logfile
    
    @param p: process to kill
    @type  p: Process
    @param errors: list of error messages from killed process
    @type  errors: [str]
    """
    try:
        printlog("[%s] killing on exit"%p.name)
        # we accumulate errors from each process so that we can print these at the end
        p.stop(errors)
    except Exception as e:
        printerrlog("Exception: %s"%(str(e)))
    
class _ProcessKiller(Thread):
    
    def __init__(self, q, i):
        Thread.__init__(self, name="ProcessKiller-%s"%i)
        self.q = q
        self.errors = []
        
    def run(self):
        q = self.q
        while not q.empty():
            try:
                p = q.get(False)
                _kill_process(p, self.errors)
                q.task_done()
            except Empty:
                pass

        
    
