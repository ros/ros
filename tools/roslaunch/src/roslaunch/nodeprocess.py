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
import signal
import socket
import subprocess 
import time
import traceback

import roslib.rosenv 
import roslib.network 

from roslaunch.core import *
from roslaunch.node_args import create_local_process_env, create_local_process_args
from roslaunch.pmon import Process, FatalProcessLaunch

import logging
_logger = logging.getLogger("roslaunch")

_TIMEOUT_SIGINT  = 5.0 #seconds
_TIMEOUT_SIGTERM = 2.0 #seconds

_counter = 0
def _next_counter():
    global _counter
    _counter += 1
    return _counter

## Launch a master
## @param type str: name of master executable (currently just Master.ZENMASTER)
## @param ros_root str: ROS_ROOT environment setting
## @param port int: port to launch master on
## @param log_output bool: if True, output goes to log file. Else, output goes to screen.
## @throws RLException if \a type or \a port is invalid
def create_master_process(run_id, type, ros_root, port, log_output=False):
    if port < 1 or port > 65535:
        raise RLException("invalid port assignment: %s"%port)

    _logger.info("create_master_process: %s, %s, %s", type, ros_root, port)
    master = os.path.join(ros_root, 'bin', type)
    # only support zenmaster now that botherder is gone
    if type == Master.ZENMASTER:
        package = 'rospy'        
        args = [master, '--core', '-p', str(port)]
    else:
        raise RLException("unknown master type: %s"%type)

    _logger.info("process[master]: launching with args [%s]"%args)
    p = LocalProcess(run_id, package, 'master', args, os.environ, log_output, None)
    return p

## Factory for generating processes for launching local ROS
## nodes. Also registers the process with the ProcessMonitor so that
## events can be generated when the process dies.
## @param run_id str: run_id of launch
## @param node Node: node to launch
## @param master_uri str: API URI for master node
## @return LocalProcess local process instance
## @raise NodeParamsException If the node's parameters are improperly specific
def create_node_process(run_id, node, master_uri):
    _logger.info("create_node_process: package[%s] type[%s] machine[%s] master_uri[%s]", node.package, node.type, node.machine, master_uri)
    # check input args
    machine = node.machine
    if machine is None:
        raise RLException("Internal error: no machine selected for node of type [%s/%s]"%(node.package, node.type))
    
    # - setup env for process (vars must be strings for os.environ)
    env = create_local_process_env(node, machine, master_uri)
    if node.name:
        # we have to include the counter to prevent potential name
        # collisions between the two branches
        name = "%s-%s"%(node.name, _next_counter())
    else:
        name = "%s-%s"%(node.type, _next_counter())

    _logger.info('process[%s]: env[%s]', name, env)

    args = create_local_process_args(node, machine)
    _logger.info('process[%s]: args[%s]', name, args)        

    log_output = node.output == 'log'
    _logger.debug('process[%s]: returning LocalProcess wrapper')
    return LocalProcess(run_id, node.package, name, args, env, log_output, respawn=node.respawn, required=node.required, cwd=node.cwd)

## Process launched on local machine
class LocalProcess(Process):
    
    ## @param run_id str: unique run ID for this roslaunch. Used to
    ##   generate log directory location. run_id may be None if this
    ##   feature is not being used.
    ## @param package str: name of package process is part of
    ## @param name str: name of process
    ## @param args [str]: list of arguments to process
    ## @param env dict: environment for process
    ## @param log_output bool: if True, log output streams of process
    ## @param respawn bool: respawn process if it dies (default is False)
    ## @param cwd str: working directory of process, or None
    def __init__(self, run_id, package, name, args, env, log_output, respawn=False, required=False, cwd=None):
        super(LocalProcess, self).__init__(package, name, args, env, respawn, required)
        self.run_id = run_id
        self.popen = None
        self.log_output = log_output
        self.started = False
        self.stopped = False
        self.cwd = cwd
        self.log_dir = None
        self.pid = -1

    # NOTE: in the future, info() is going to have to be sufficient for relaunching a process
    ## Get all data about this process in dictionary form
    def get_info(self):
        info = super(LocalProcess, self).get_info()
        info['pid'] = self.pid
        if self.run_id:
            info['run_id'] = self.run_id
        info['log_output'] = self.log_output
        if self.cwd is not None:
            info['cwd'] = self.cwd
        return info

    ## Configure logging of node's log file and stdout/stderr
    ## @param self
    ## @return str, str: stdout log file name, stderr log file
    ## name. Values are None if stdout/stderr are not logged.
    def _configure_logging(self):
        log_dir = roslib.roslogging.get_log_dir(os.environ)
        if self.run_id:
            log_dir = os.path.join(log_dir, self.run_id)
        if not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir)
            except OSError, (errno, msg):
                if errno == 13:
                    raise RLException("unable to create directory for log file [%s].\nPlease check permissions."%log_dir)
                else:
                    raise RLException("unable to create directory for log file [%s]: %s"%(log_dir, msg))
        # #973: save log dir for error messages
        self.log_dir = log_dir

        # send stdout/stderr to file. in the case of respawning, we have to
        # open in append mode
        # note: logfileerr: disabling in favor of stderr appearing in the console.
        # will likely reinstate once roserr/rosout is more properly used.
        logfileout = logfileerr = None

        if self.log_output:
            outf, errf = [os.path.join(log_dir, '%s-%s.log'%(self.name, n)) for n in ['stdout', 'stderr']]
            if self.respawn:
                mode = 'a'
            else:
                mode = 'w'
            logfileout = open(outf, mode)
            if is_child_mode():
                logfileerr = open(errf, mode)

        # #986: pass in logfile name to node
        node_log_file = log_dir
        # #1595: on respawn, these keep appending
        self.args = _cleanup_remappings(self.args, '__log:=')
        self.args.append("__log:=%s"%os.path.join(log_dir, "%s.log"%self.name))

        return logfileout, logfileerr

    ## start the process.
    ## @throws FatalProcessLaunch: if process cannot be started and it
    ## is not likely to ever succeed
    def start(self):
        super(LocalProcess, self).start()
        try:
            self.lock.acquire()
            if self.started:
                _logger.info("process[%s]: restarting os process", self.name)
            else:
                _logger.info("process[%s]: starting os process", self.name)
            self.started = self.stopped = False

            full_env = self.env

            # _configure_logging() can mutate self.args
            try:
                logfileout, logfileerr = self._configure_logging()
            except Exception, e:
                _logger.error(traceback.format_exc())
                printerrlog("[%s] ERROR: unable to configure logging [%s]"%(self.name, str(e)))
                # it's not safe to inherit from this process as
                # rostest changes stdout to a StringIO, which is not a
                # proper file.
                logfileout, logfileerr = subprocess.PIPE, subprocess.PIPE

            if self.cwd == 'node':
                cwd = os.path.dirname(self.args[0])
            else:
                cwd = get_ros_root()

            _logger.info("process[%s]: start w/ args [%s]", self.name, self.args)
            _logger.info("process[%s]: cwd will be [%s]", self.name, cwd)

            try:
                self.popen = subprocess.Popen(self.args, cwd=cwd, stdout=logfileout, stderr=logfileerr, env=full_env, close_fds=True, preexec_fn=os.setsid)
            except OSError, (errno, msg):
                self.started = True # must set so is_alive state is correct
                _logger.error("OSError(%d, %s)", errno, msg)
                if errno == 8: #Exec format error
                    raise FatalProcessLaunch("Unable to launch [%s]. \nIf it is a script, you may be missing a '#!' declaration at the top."%self.name)
                elif errno ==2: #no such file or directory
                    raise FatalProcessLaunch("""Roslaunch got a '%s' error while attempting to run:

%s

Please make sure that all the executables in this command exist and have
executable permission. This is often caused by a bad launch-prefix."""%(msg, ' '.join(self.args)))
                
            self.started = True
            if self.popen.poll() is None:
                self.pid = self.popen.pid
                printlog_bold("process[%s]: started with pid [%s]"%(self.name, self.pid))
                return True
            else:
                printerrlog("failed to start local process: %s"%(' '.join(self.args)))
                return False
        finally:
            self.lock.release()

    ## @param self
    ## @return bool: True if process is still running
    def is_alive(self):
        if not self.started: #not started yet
            return True
        if self.stopped or self.popen is None:
            return False
        self.exit_code = self.popen.poll()
        if self.exit_code is not None:
            return False
        return True

    ## @param self
    ## @return str: human-readable description of exit state 
    def get_exit_description(self):
        # #973: include location of output location in message
        if self.exit_code is not None:
            if self.exit_code:
                if self.log_dir:
                    return 'process has died [pid %s, exit code %s].\nlog files: %s*.log'%(self.pid, self.exit_code, os.path.join(self.log_dir, self.name))
                else:
                    return 'process has died [pid %s, exit code %s]'%(self.pid, self.exit_code)
            else:
                if self.log_dir:
                    return 'process has finished cleanly.\nlog file: %s*.log'%(os.path.join(self.log_dir, self.name))
                else:
                    return 'process has finished cleanly'
        else:
            return 'process has died'

    ## kill UNIX process
    ## @param self
    def _stop_unix(self):
        self.exit_code = self.popen.poll() 
        if self.exit_code is not None:
            _logger.debug("process[%s].stop(): process has already returned %s", self.name, self.exit_code)
            #print "process[%s].stop(): process has already returned %s"%(self.name, self.exit_code)                
            self.popen = None
            self.stopped = True
            return
        pid = self.popen.pid
        pgid = os.getpgid(pid)
        _logger.info("process[%s]: killing os process with pid[%s] pgid[%s]", self.name, pid, pgid)
        try:
            # Start with SIGINT and escalate from there.
            _logger.info("[%s] sending SIGINT to pgid [%s]", self.name, pgid)                                    
            os.killpg(pgid, signal.SIGINT)
            _logger.info("[%s] sent SIGINT to pgid [%s]", self.name, pgid)
            #time.sleep(0.5) #I'm not sure why this was here - kwc
            timeout_t = time.time() + _TIMEOUT_SIGINT
            retcode = self.popen.poll()                
            while time.time() < timeout_t and retcode is None:
                time.sleep(0.1)
                retcode = self.popen.poll()
            # Escalate non-responsive process
            if retcode is None:
                printerrlog("[%s] escalating to SIGTERM"%self.name)
                timeout_t = time.time() + _TIMEOUT_SIGTERM
                os.killpg(pgid, signal.SIGTERM)                
                _logger.info("[%s] sent SIGTERM to pgid [%s]"%(self.name, pgid))
                retcode = self.popen.poll()
                while time.time() < timeout_t and retcode is None:
                    time.sleep(0.2)
                    _logger.debug('poll for retcode')
                    retcode = self.popen.poll()
                if retcode is None:
                    printerrlog("[%s] escalating to SIGKILL"%self.name)
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        _logger.info("[%s] sent SIGKILL to pgid [%s]"%(self.name, pgid))
                        #self.popen.wait()
                        os.wait()
                        _logger.info("process[%s]: SIGKILL killed", self.name)
                    except OSError, e:
                        if e.args[0] == 3:
                            printerrlog("no [%s] process with pid [%s]"%(self.name, pid))
                        else:
                            printerrlog("errors shutting down [%s], see log for details"%self.name)
                            _logger.error(traceback.format_exc())
                else:
                    _logger.info("process[%s]: SIGTERM killed with return value %s", self.name, retcode)
            else:
                _logger.info("process[%s]: SIGINT killed with return value %s", self.name, retcode)
                
        finally:
            self.popen = None
        
    ## Stop the process
    ## @param self
    def stop(self):
        super(LocalProcess, self).stop()
        self.lock.acquire()        
        try:
            try:
                _logger.debug("process[%s].stop() starting", self.name)
                if self.popen is None:
                    _logger.debug("process[%s].stop(): popen is None, nothing to kill") 
                    return
                #NOTE: currently POSIX-only. Need to add in Windows code once I have a test environment:
                # http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/347462
                self._stop_unix()
            except:
                #traceback.print_exc() 
                _logger.error("[%s] EXCEPTION %s", self.name, traceback.format_exc())                                
        finally:
            self.stopped = True
            self.lock.release()


# #1595: remove all instances of args that start with \a prefix. This is used to
# remove args that were previously added (and are now being regenerated due to respawning)
def _cleanup_remappings(args, prefix):
    existing_args = [a for a in args if a.startswith(prefix)]
    for a in existing_args:
        args.remove(a)
    return args
