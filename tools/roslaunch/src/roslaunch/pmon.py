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
import time
import traceback
import logging
import signal
import atexit
from threading import Thread, RLock, Lock

from core import printlog, printlog_bold, printerrlog

logger = logging.getLogger("roslaunch.pmon")          

# start/shutdown ################################################

pmons = []
def start_process_monitor():
    if _shutting_down:
        logger.error("start_process_monitor: cannot start new ProcessMonitor (shutdown initiated)")
        return None
    logger.debug("start_process_monitor: creating ProcessMonitor")
    processMonitor = ProcessMonitor()
    try:
        _shutdown_lock.acquire()
        pmons.append(processMonitor)
    finally:
        _shutdown_lock.release()
        
    if not _shutting_down:
        processMonitor.start()
        logger.debug("start_process_monitor: ProcessMonitor started")
        return processMonitor
    else:
        logger.error("start_process_monitor: aborting startup as shutdown has been initiated")

def shutdown_process_monitor(process_monitor):
    try:
        if process_monitor is None or process_monitor.is_shutdown:
            return
        logger.debug("shutdown_process_monitor: shutting down ProcessMonitor")
        process_monitor.shutdown()
        logger.debug("shutdown_process_monitor: joining ProcessMonitor")
        process_monitor.join(5.0)
        logger.debug("shutdown_process_monitor: ProcessMonitor shutdown")        
    except:
        print "exception in shutdown_process_monitor"
        traceback.print_exc()

_shutdown_lock = Lock()
def pmon_shutdown():
    try:
        try:
            _shutdown_lock.acquire()
            if not pmons:
                return
            for p in pmons:
                try:
                    shutdown_process_monitor(p)
                except:
                    print "exception in pmon_shutdown"
                    traceback.print_exc()
            del pmons[:]
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

class Process(object):

    def __init__(self, package, name, args, env, respawn=False):
        self.package = package
        self.name = name
        self.args = args
        self.env = env
        self.respawn = respawn
        self.lock = Lock()
        self.exit_code = None
    
    def start(self):
        pass

    def is_alive(self):
        return False

    def stop(self):
        pass

    def get_exit_description(self):
        if self.exit_code is not None:
            if self.exit_code:
                return ' [exit code %s]'%self.exit_code
            else:
                return ' cleanly'
        else:
            return ''

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

    def __init__(self):
        Thread.__init__(self, name="ProcessMonitor")
        self.procs = []
        self.plock = RLock()
        self.is_shutdown = False
        self.done = False        
        #self.setDaemon(True)
        self.reacquire_signals = set()
        self.listeners = []
        # #885: ensure core procs
        self.core_procs = []
        # #642: flag to prevent process monitor exiting prematurely
        self._registrations_complete = False
        
    ## Listener for process events. MUST be called before
    ## ProcessMonitor is running.See ProcessListener class.
    ## @param l ProcessListener : listener instance
    def add_process_listener(self, l):
        self.listeners.append(l)

    ## Register process with ProcessMonitor
    ## @param self
    ## @param p Process
    def register(self, p):
        logger.debug("ProcessMonitor.register[%s]"%p.name) 
        try:
            self.plock.acquire()
            self.procs.append(p)
        finally:
            self.plock.release()

    ## Register core process with ProcessMonitor. Coreprocesses
    ## have special shutdown semantics. They are killed after all
    ## other processes, in reverse order in which they are added.
    ## @param self
    ## @param p Process
    def register_core_proc(self, p):
        self.register(p)
        self.core_procs.append(p)
        
    ## Inform the process monitor that registrations are complete.
    ## After the registrations_complete flag is set, process monitor
    ## will exit if there are no processes left to monitor.
    def registrations_complete(self):
        self._registrations_complete = True
        
    def unregister(self, p):
        logger.debug("ProcessMonitor.unregister[%s]"%p.name)                
        try:
            self.plock.acquire()
            self.procs.remove(p)
        finally:
            self.plock.release()            

    ## @return bool: True if process is still be monitored. If False, process
    ## has died or was never registered with process
    def has_process(self, name):
        return len([p for p in self.procs if p.name == name]) > 0

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
    
    ## kill all processes that match name
    ## @param self
    ## @param name str: Process name            
    def kill_process(self, name):
        logger.debug("ProcessMonitor.kill_process[%s]"%name)
        printlog("[%s] kill requested"%name)
        try:
            self.plock.acquire()
            matches = [p for p in self.procs if p.name == name]
            for p in matches:
                try:
                    p.stop()
                except:
                    logger.error(traceback.format_exc())                    
                self.unregister(p)
        finally:
            self.plock.release()            
        
    ## Shutdown the process monitor thread
    ## @param self
    def shutdown(self):
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
            self._run()
        except:
            self._post_run()
            logger.error(traceback.format_exc())
            traceback.print_exc()
            
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
                return

            # check current signal handlers to see if children have stolen them away
            for s in [signal.SIGTERM, signal.SIGINT, signal.SIGHUP]:
                if signal.getsignal(s) !=  rl_signal:
                    self.reacquire_signals.add(s)

            for p in procs:
                try:
                    if not p.is_alive():
                        logger.debug("Process[%s] has died, respawn=%s, exit_code=%s",p.name, p.respawn, p.exit_code)
                        exit_code_str = p.get_exit_description()
                        if p.respawn:
                            printlog("[%s] process has died%s, will respawn"%(p.name, exit_code_str))
                            respawn.append(p)
                        else:
                            if p.exit_code:
                                printerrlog("[%s] process has died%s"%(p.name, exit_code_str))
                            else:
                                printlog_bold("[%s] process has died%s"%(p.name, exit_code_str))
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
                except:
                    logger.error(traceback.format_exc())
            if self._registrations_complete and dead and not self.procs and not respawn:
                printlog("all processes on machine have died, roslaunch will exit")
                self.is_shutdown = True
            del dead[:]
            for r in respawn: #get out of this loop
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
        self._post_run() #kill all processes

    def _post_run(self):
        #killall processes on run exit
        procs = self.procs[:]
        core_procs = self.core_procs[:]        
        
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

        del procs[:]
        del core_procs[:]
        del self.procs[:]
        del self.core_procs[:]        
        
        reacquire_signals = self.reacquire_signals
        if reacquire_signals:
            reacquire_signals.clear() 
        logger.debug("ProcessMonitor has shutdown")
        self.done = True
