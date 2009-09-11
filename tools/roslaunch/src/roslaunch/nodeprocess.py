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
import shlex
import signal
import socket
import subprocess 
import time
import traceback

import roslib.rosenv 
import roslib.network 
import roslib.packages
import roslib.substitution_args

from roslaunch.core import *
from roslaunch.pmon import Process

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
## @param type str: name of master executable (Master.BOTHERDER or Master.ZENMASTER)
## @param ros_root str: ROS_ROOT environment setting
## @param port int: port to launch master on
## @param log_output bool: if True, output goes to log file. Else, output goes to screen.
## @throws RLException if \a type or \a port is invalid
def create_master_process(type, ros_root, port, log_output=False):
    if port < 1 or port > 65535:
        raise RLException("invalid port assignment: %s"%port)

    _logger.info("create_master_process: %s, %s, %s", type, ros_root, port)
    master = os.path.join(ros_root, 'bin', type)
    # botheder and zenmaster have different command-line args for specifying the port
    if type == Master.BOTHERDER:
        package = 'roscpp'
        args = [master, str(port)]
    elif type == Master.ZENMASTER:
        package = 'rospy'        
        args = [master, '--core', '-p', str(port)]
    else:
        raise RLException("unknown master type: %s"%type)

    _logger.info("process[master]: launching with args [%s]"%args)
    p = LocalProcess(package, 'master', args, os.environ, log_output, None)
    return p

## subroutine for creating node arguments
## @return list: arguments for node process
## @raise NodeParamsException: if args cannot be constructed for Node
## as specified (e.g. the node type does not exist)
def _construct_args(node, machine):
    # - Construct rosrun command
    remap_args = ["%s:=%s"%(src,dst) for src, dst in node.remap_args]
    if node.name:
        remap_args.append('__name:=%s'%node.name)
        
    #resolve args evaluates substitution commands
    #shlex parses a command string into a list of args
    resolved = roslib.substitution_args.resolve_args(node.args)
    if type(resolved) == unicode:
        resolved = resolved.encode('UTF-8') #attempt to force to string for shlex/subprocess
    args = shlex.split(resolved) + remap_args

    start_t = time.time()
    try:
        cmd = roslib.packages.find_node(node.package, node.type,\
                                            machine.ros_root, machine.ros_package_path)
    except roslib.packages.ROSPkgException, e:
        # multiple nodes, invalid package
        raise NodeParamsException(str(e))
    end_t = time.time()
    _logger.info('find_node(%s, %s, %s, %s) took %ss'%(node.package, node.type, machine.ros_root, machine.ros_package_path, (end_t - start_t)))
    if not cmd:
        raise NodeParamsException("Cannot locate node of type [%s] in package [%s]"%(node.type, node.package))
    return [cmd] + args        
    
class NodeParamsException(Exception): pass

## Setup environment for locally launched process. The local
## environment includes the default os environment, with any
## ROS-specific environment variables overriding this enviornment.
## @return dict : environment variables
def setup_local_process_env(node, machine, master_uri, env=os.environ):

    # #1029: generate environment for the node. unset
    # #ROS-related environment vars before
    # update() so that extra environment variables don't end
    # up in the call.
    full_env = env.copy()

    for evar in [
        roslib.rosenv.ROS_MASTER_URI,
        roslib.rosenv.ROS_ROOT,
        roslib.rosenv.ROS_PACKAGE_PATH,
        roslib.rosenv.ROS_IP,
        'PYTHONPATH',
        roslib.rosenv.ROS_NAMESPACE]:
        if evar in full_env:
            del full_env[evar]

    proc_env = setup_env(node, machine, master_uri)
    full_env.update(proc_env)
    return full_env

## Factory for generating processes for launching local ROS
## nodes. Also registers the process with the ProcessMonitor so that
## events can be generated when the process dies.
## @param node Node: node to launch
## @param master_uri str: API URI for master node
## @return LocalProcess local process instance
## @raise NodeParamsException If the node's parameters are improperly specific
def create_node_process(node, master_uri):
    _logger.info("create_node_process: package[%s] type[%s] machine[%s] master_uri[%s]", node.package, node.type, node.machine, master_uri)
    # check input args
    machine = node.machine
    if machine is None:
        raise RLException("Internal error: no machine selected for node of type [%s/%s]"%(node.package, node.type))
    
    # - setup env for process (vars must be strings for os.environ)
    env = setup_local_process_env(node, machine, master_uri)
    name = "%s-%s"%(node.type, _next_counter())
    _logger.info('process[%s]: env[%s]', name, env)

    args = _construct_args(node, machine)
    _logger.info('process[%s]: args[%s]', name, args)        

    log_output = node.output == 'log'
    _logger.debug('process[%s]: returning LocalProcess wrapper')
    return LocalProcess(node.package, name, args, env, log_output, respawn=node.respawn, cwd=node.cwd)

class LocalProcess(Process):
    def __init__(self, package, name, args, env, log_output, respawn=False, cwd=None):
        super(LocalProcess, self).__init__(package, name, args, env, respawn)
        self.popen = None
        self.log_output = log_output
        self.started = False
        self.stopped = False
        self.cwd = cwd
        self.stderr_filename = self.stdout_filename = None
        self.pid = -1
        
    def _validate(self):
        try:
            # verify package exists
            roslib.packages.get_pkg_dir(self.package, True)
        except:
            printerrlog("[%s]: cannot start, package [%s] could not be found"%(self.name, self.package))
            return False
        
        paths = [self.env.get(roslib.rosenv.ROS_ROOT, None), self.args[0]]
        paths.extend(self.env.get(roslib.rosenv.ROS_PACKAGE_PATH, '').split(os.pathsep))
        paths = [p for p in paths if p and not os.path.exists(p)] 
        if paths:
            printerrlog("process[%s]: warning paths [%s] do not exist"%(self.name, paths))
        return True
    
    def start(self):
        if not self._validate():
            return False
        try:
            self.lock.acquire()
            if self.started:
                _logger.info("process[%s]: restarting os process", self.name)
            else:
                _logger.info("process[%s]: starting os process", self.name)
            self.started = self.stopped = False

            full_env = self.env

            # send stdout/stderr to file. in the case of respawning, we have to
            # open in append mode
            # note: logfileerr: disabling in favor of stderr appearing in the console.
            # will likely reinstate once roserr/rosout is more properly used.
            outf = errf = None
            logfileout = logfileerr = None
            if self.log_output:
                if "ROS_LOG_DIR" in os.environ:
                    log_dir = os.environ["ROS_LOG_DIR"]
                else:
                    log_dir = os.path.join(get_ros_root(), 'log')
                outf, errf = [os.path.join(log_dir, '%s-%s.log'%(self.name, n)) for n in ['stdout', 'stderr']]
                if self.respawn:
                    mode = 'a'
                else:
                    mode = 'w'
                logfileout = open(outf, mode)
                if is_child_mode():
                    logfileerr = open(errf, mode)

            _logger.info("process[%s]: start w/ args [%s]", self.name, self.args)
            if self.cwd == 'node':
                cwd = os.path.dirname(self.args[0])
            else:
                cwd = get_ros_root()
            _logger.info("process[%s]: cwd will be [%s]", self.name, cwd)

            try:
                if not logfileerr:
                    self.popen = subprocess.Popen(self.args, cwd=cwd, stdout=logfileout, env=full_env, close_fds=True, preexec_fn=os.setsid)
                else:
                    self.popen = subprocess.Popen(self.args, cwd=cwd, stdout=logfileout, stderr=logfileerr, env=full_env, close_fds=True, preexec_fn=os.setsid)
            except OSError, (errno, msg):
                self.started = True # must set so is_alive state is correct
                _logger.error("OSError(%d, %s)", errno, msg)
                if errno == 8: #Exec format error
                    printerrlog("Unable to launch [%s]. \nIf it is a script, you may be missing a '#!' declaration at the top."%self.name)
                return False
                
                
            # #973: save logfile locations so we can include them in
            # error messages
            if logfileerr:
                self.stderr_filename = errf
            if logfileout:                
                self.stdout_filename = outf
            
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

    def is_alive(self):
        if not self.started: #not started yet
            return True
        if self.stopped or self.popen is None:
            return False
        self.exit_code = self.popen.poll()
        if self.exit_code is not None:
            return False
        return True

    def get_exit_description(self):
        # #973: include location of output location in message
        if self.exit_code is not None:
            if self.exit_code:
                if self.stderr_filename:
                    return ' [pid %s, exit code %s]. stderr is in [%s]'%(self.pid, self.exit_code, self.stderr_filename)
                elif self.stdout_filename:
                    return ' [pid %s, exit code %s]. stdout is in [%s]'%(self.pid, self.exit_code, self.stdout_filename)
                else:
                    return ' [pid %s, exit code %s]'%(self.pid, self.exit_code)
            else:
                return ' cleanly'
        else:
            return ''

    ## kill UNIX process 
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
        
    def stop(self):
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


