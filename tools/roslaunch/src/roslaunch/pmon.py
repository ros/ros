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

import os
import sys
import time
import traceback
import logging
import signal
import atexit
from threading import Thread, RLock, Lock

from roslaunch.core import printlog, printlog_bold, printerrlog, RLException

logger = logging.getLogger("roslaunch.pmon")          

## Exception to indicate that a process launch has failed in a fatal
## manner (i.e. relaunch is unlikely to succeed)
class FatalProcessLaunch(RLException): pass

# start/shutdown ################################################

_pmons = []
_pmon_counter = 0
def start_process_monitor():
    global _pmon_counter
    if _shutting_down:
        #logger.error("start_process_monitor: cannot start new ProcessMonitor (shutdown initiated)")
        return None
    _pmon_counter += 1
    name = "ProcessMonitor-%s"%_pmon_counter
    logger.info("start_process_monitor: creating ProcessMonitor")
    process_monitor = ProcessMonitor(name)
    try:
        # prevent race condition with pmon_shutdown() being triggered
        # as we are starting a ProcessMonitor (i.e. user hits ctrl-C
        # during startup)
        _shutdown_lock.acquire()
        _pmons.append(process_monitor)
        process_monitor.start()
        logger.info("start_process_monitor: ProcessMonitor started")
    finally:
        _shutdown_lock.release()

    return process_monitor

## @param process_monitor ProcessMonitor: process monitor to kill
## @return bool: True if process_monitor was successfully
## shutdown. False if it could not be shutdown cleanly or if there is
## a problem with \a process_monitor
## parameter. shutdown_process_monitor() does not throw any exceptions
## as this is shutdown-critical code.
def shutdown_process_monitor(process_monitor):
    try:
        if process_monitor is None or process_monitor.is_shutdown:
            return False
        
        # I've decided to comment out use of logger until after
        # critical section is done, just in case logger is already
        # being torn down
        
        #logger.debug("shutdown_process_monitor: shutting down ProcessMonitor")
        process_monitor.shutdown()
        #logger.debug("shutdown_process_monitor: joining ProcessMonitor")
        process_monitor.join(20.0)
        if process_monitor.isAlive():
            logger.error("shutdown_process_monitor: ProcessMonitor shutdown failed!")
            return False
        else:
            logger.debug("shutdown_process_monitor: ProcessMonitor shutdown succeeded")
            return True
    except Exception, e:
        print >> sys.stderr, "exception in shutdown_process_monitor: %s"%e
        traceback.print_exc()
        return False

_shutdown_lock = Lock()
def pmon_shutdown():
    global _pmons
    try:
        _shutdown_lock.acquire()
        try:
            if not _pmons:
                return
            for p in _pmons:
                shutdown_process_monitor(p)
            del _pmons[:]
        except:
            print "exception in pmon_shutdown"
            traceback.print_exc()
    finally:
        _shutdown_lock.release()
    
_signal_chain = {}
_shutting_down = False
def rl_signal(sig, stackframe):
    global _shutting_down
    if _shutting_down:
        return #prevent infinite callbacks
    _shutting_down = True
    pmon_shutdown()
    prev_handler = _signal_chain.get(sig, None)
    if prev_handler and prev_handler not in [signal.SIG_IGN, signal.SIG_DFL, rl_signal]:
        try:
            prev_handler(sig, stackframe)
        except KeyboardInterrupt:
            pass #filter out generic keyboard interrupt handler
        
_signal_chain[signal.SIGTERM] = signal.signal(signal.SIGTERM, rl_signal)
_signal_chain[signal.SIGINT]  = signal.signal(signal.SIGINT, rl_signal)
_signal_chain[signal.SIGHUP]  = signal.signal(signal.SIGHUP, rl_signal)        
atexit.register(pmon_shutdown)

# ##############################################################

## Basic process representation for ProcessMonitor. Must be subclassed
## to provide actual start()/stop() implementations.
class Process(object):

    def __init__(self, package, name, args, env, respawn=False):
        self.package = package
        self.name = name
        self.args = args
        self.env = env
        self.respawn = respawn
        self.lock = Lock()
        self.exit_code = None
        # for keeping track of respawning
        self.spawn_count = 0

    # NOTE: get_info() is going to have to be sufficient for
    # generating respawn requests, so we must be complete about it
        
    ## Get all data about this process in dictionary form
    ## @return dict { str: val }: dictionary of all relevant process properties
    def get_info(self):
        info = {
            'spawn_count': self.spawn_count,
            'args': self.args,
            'env': self.env,
            'package': self.package,
            'name': self.name,
            'alive': self.is_alive(),
            'respawn': self.respawn,
            }
        if self.exit_code is not None:
            info['exit_code'] = self.exit_code
        return info

    def start(self):
        self.spawn_count += 1

    def is_alive(self):
        return False

    def stop(self):
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

## Container class to maintain information about a process that has died. This
## container allows us to delete the actual Process but still maintain the metadata
class DeadProcess(Process):
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

## Listener class for ProcessMonitor
class ProcessListener(object):
    
    ## Notifies listener that process has died. This callback only
    ## occurs for processes that die during normal process monitor
    ## execution -- processes that are forcibly killed during
    ## ProcessMonitor shutdown are not reported.
    ## @param self
    ## @param process_name str: name of process
    ## @param exit_code int: exit code of process. If None, it means
    ## that ProcessMonitor was unable to determine an exit code.
    def process_died(self, process_name, exit_code):
        pass
    
class ProcessMonitor(Thread):

    def __init__(self, name="ProcessMonitor"):
        Thread.__init__(self, name=name)
        self.procs = []
        self.plock = RLock()
        self.is_shutdown = False
        self.done = False        
        #self.setDaemon(True)
        self.reacquire_signals = set()
        self.listeners = []
        self.dead_list = []
        # #885: ensure core procs
        self.core_procs = []
        # #642: flag to prevent process monitor exiting prematurely
        self._registrations_complete = False
        
        logger.info("created process monitor %s"%self)
        
    ## Listener for process events. MUST be called before
    ## ProcessMonitor is running.See ProcessListener class.
    ## @param l ProcessListener : listener instance
    def add_process_listener(self, l):
        self.listeners.append(l)

    ## Register process with ProcessMonitor
    ## @param self
    ## @param p Process
    ## @throws RLException if process with same name is already registered
    def register(self, p):
        logger.info("ProcessMonitor.register[%s]"%p.name)
        e = None
        try:
            self.plock.acquire()
            if self.has_process(p.name):
                e = RLException("cannot add process with duplicate name '%s'"%p.name)
            elif self.is_shutdown:
                e = RLException("cannot add process [%s] after process monitor has been shut down"%p.name)
            else:
                self.procs.append(p)
        finally:
            self.plock.release()
        if e:
            logger.error("ProcessMonitor.register[%s] failed %s"%(p.name, e))
            raise e
        else:
            logger.info("ProcessMonitor.register[%s] complete"%p.name)

    ## Register core process with ProcessMonitor. Coreprocesses
    ## have special shutdown semantics. They are killed after all
    ## other processes, in reverse order in which they are added.
    ## @param self
    ## @param p Process
    ## @throws RLException if process with same name is already registered
    def register_core_proc(self, p):
        self.register(p)
        self.core_procs.append(p)
        
    ## Inform the process monitor that registrations are complete.
    ## After the registrations_complete flag is set, process monitor
    ## will exit if there are no processes left to monitor.
    def registrations_complete(self):
        self._registrations_complete = True
        logger.info("registrations completed %s"%self)
        
    def unregister(self, p):
        logger.info("ProcessMonitor.unregister[%s] starting"%p.name)                
        try:
            self.plock.acquire()
            self.procs.remove(p)
        finally:
            self.plock.release()
        logger.info("ProcessMonitor.unregister[%s] complete"%p.name)             

    ## @return bool: True if process is still be monitored. If False, process
    ## has died or was never registered with process
    def has_process(self, name):
        return len([p for p in self.procs if p.name == name]) > 0

    ## @return Process: process registered under \a name, or None
    def get_process(self, name):
        try:
            self.plock.acquire()
            v = [p for p in self.procs if p.name == name]
        finally:
            self.plock.release()            
        if v:
            return v[0]

    ## @return True if ProcessMonitor has tasks that need to be run in the main thread
    def has_main_thread_jobs(self):
        return len(self.reacquire_signals)
    
    ## Execute tasks that need to be run in the main thread. Must be
    ## called from main thread.
    def do_main_thread_jobs(self):
        #not entirely threadsafe
        sigs = [s for s in self.reacquire_signals]
        for s in sigs:
            _signal_chain[s] = signal.signal(s, rl_signal)
            self.reacquire_signals.remove(s)
    
    ## kill process that matches \a name. NOTE: a killed process will
    ## continue to show up as active until the process monitor thread
    ## has caught that it has died.
    ## @param self
    ## @param name str: Process name
    ## @return True if a process named \a name was removed from
    ## process monitor. A process is considered killed if its stop()
    ## method was called. 
    def kill_process(self, name):
        if not isinstance(name, basestring):
            raise RLException("kill_process takes in a process name but was given: %s"%name)
        logger.debug("ProcessMonitor.kill_process[%s]"%name)
        printlog("[%s] kill requested"%name)
        try:
            self.plock.acquire()
            p = self.get_process(name)
            if p:
                try:
                    p.stop()
                except:
                    logger.error(traceback.format_exc())
                return True
            else:
                return False
        finally:
            self.plock.release()
        
    ## Shutdown the process monitor thread
    ## @param self
    def shutdown(self):
        logger.info("ProcessMonitor.shutdown %s"%self)
        self.is_shutdown = True
        
    ## @param self
    ## @return [str]: list of active process names
    def get_active_names(self):
        try:
            self.plock.acquire()
            retval = [p.name for p in self.procs]
        finally:
            self.plock.release()
        return retval

    ## @param self
    ## @return [[(str, int),], [(str,int),]]: Two lists, where first
    ## list of active process names along with the number of times
    ## that process has been spawned. Second list contains dead process names
    ## and their spawn count.
    def get_process_names_with_spawn_count(self):
        try:
            self.plock.acquire()
            actives = [(p.name, p.spawn_count) for p in self.procs]
            deads = [(p.name, p.spawn_count) for p in self.dead_list]
            retval = [actives, deads]
        finally:
            self.plock.release()
        return retval

    ## run() occurs in a separate thread and cannot do certain signal-related
    ## work. The main thread of the application must call mainthread_spin()
    ## or mainthread_spin_once() in order to perform these jobs.
    ## @param self
    def mainthread_spin_once(self):
        if not self.done:
            if self.has_main_thread_jobs():
                self.do_main_thread_jobs()
            return True
        else:
            return False
        
    ## run() occurs in a separate thread and cannot do certain signal-related
    ## work. The main thread of the application must call mainthread_spin()
    ## or mainthread_spin_once() in order to perform these jobs. mainthread_spin()
    ## blocks until the process monitor is complete.
    ## @param self
    def mainthread_spin(self):
        while not self.done:
            time.sleep(0.1)
            if self.has_main_thread_jobs():
                self.do_main_thread_jobs()

    ## thread routine of the process monitor. NOTE: you must still
    ## call mainthread_spin or mainthread_spin_once() from the main
    ## thread in order to pick up main thread work from the process
    ## monitor.
    ## @param self
    def run(self):
        try:
            #don't let exceptions bomb thread, interferes with exit
            try:
                self._run()
            except:
                logger.error(traceback.format_exc())
                traceback.print_exc()
        finally:
            self._post_run()
            
    ## Internal run loop of ProcessMonitor
    ## @param self
    def _run(self):        
        plock = self.plock
        dead = []
        respawn = []
        while not self.is_shutdown:
            try: #copy self.procs
                plock.acquire()
                procs = self.procs[:]
            finally:
                plock.release()
            if self.is_shutdown:
                break

            # check current signal handlers to see if children have stolen them away
            # TODO: this code may not be necessary anymore (have to test)
            for s in [signal.SIGTERM, signal.SIGINT, signal.SIGHUP]:
                if signal.getsignal(s) !=  rl_signal:
                    self.reacquire_signals.add(s)

            for p in procs:
                try:
                    if not p.is_alive():
                        logger.debug("Process[%s] has died, respawn=%s, exit_code=%s",p.name, p.respawn, p.exit_code)
                        exit_code_str = p.get_exit_description()
                        if p.respawn:
                            printlog("[%s] %s, will respawn"%(p.name, exit_code_str))
                            respawn.append(p)
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

                except Exception, e:
                    traceback.print_exc()
                    #don't respawn as this is an internal error
                    dead.append(p)
                if self.is_shutdown:
                    break #stop polling
            for d in dead:
                try:
                    self.unregister(d)
                    d.stop()

                    # save process data to dead list 
                    plock.acquire()
                    try:
                        self.dead_list.append(DeadProcess(d))
                    finally:
                        plock.release()
                    
                except:
                    logger.error(traceback.format_exc())
                    
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
                    r.stop()
                    r.start()
                except:
                    traceback.print_exc()
                    logger.error("Restart failed %s",traceback.format_exc())
            del respawn[:]
            time.sleep(0.01) #yield thread
        #moved this to finally block of _post_run
        #self._post_run() #kill all processes

    def _post_run(self):
        logger.info("ProcessMonitor._post_run %s"%self)
        # this is already true entering, but go ahead and make sure
        self.is_shutdown = True
        # killall processes on run exit
        try:
            self.plock.acquire()
            procs = self.procs[:]
            core_procs = self.core_procs[:]
        finally:
            self.plock.release()
        logger.info("ProcessMonitor._post_run %s: remaining procs are %s"%(self, procs))
        
        # kill in reverse order
        for p in reversed(procs):
            
            # #526/885: ignore core procs
            if p in core_procs:
                continue
            try:
                logger.info("ProcessMonitor exit: killing %s", p.name)
                printlog("[%s] killing on exit"%p.name)
                p.stop()
            except:
                traceback.print_exc()
                logger.error(traceback.format_exc())

        # #526/885: kill core procs last
        for p in reversed(core_procs):
            try:
                logger.info("ProcessMonitor exit: killing %s", p.name)
                printlog("[%s] killing on exit"%p.name)
                p.stop()
            except:
                traceback.print_exc()
                logger.error(traceback.format_exc())

        # delete everything except dead_list
        logger.info("ProcessMonitor exit: cleaning up data structures and signals")
        try:
            self.plock.acquire()
            del procs[:]
            del core_procs[:]
            del self.procs[:]
            del self.core_procs[:]
        finally:
            self.plock.release()
            
        reacquire_signals = self.reacquire_signals
        if reacquire_signals:
            reacquire_signals.clear() 
        logger.info("ProcessMonitor exit: pmon has shutdown")
        self.done = True
