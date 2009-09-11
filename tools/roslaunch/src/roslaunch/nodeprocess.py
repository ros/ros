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

from roslib.rosenv import ROS_MASTER_URI, ROS_NAMESPACE, ROS_ROOT, ROS_PACKAGE_PATH, ROS_IP
import roslib.network 
import roslib.packages

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
## @param type str: name of master executable (botherder or zenmaster)
## @param ros_root str: ROS_ROOT environment setting
## @param port int: port to launch master on
## @param log_output bool: if True, output goes to log file. Else, output goes to screen.
def create_master_process(type, ros_root, port, log_output=False):
    master = os.path.join(ros_root, 'bin', type)
    # botheder and zenmaster have different command-line args for specifying the port
    if type == Master.BOTHERDER:
        package = 'roscpp'
        args = [master, str(port)]
    else:
        package = 'rospy'        
        args = [master, '--core', '-p', str(port)]        
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
    resolved = resolve_args(node.args)
    if type(resolved) == unicode:
        resolved = resolved.encode('UTF-8') #attempt to force to string for shlex/subprocess
    import shlex
    args = shlex.split(resolved) + remap_args

    cmd = roslib.packages.find_node(node.package, node.type,\
                                      machine.ros_root, machine.ros_package_path)
    if not cmd:
        raise NodeParamsException("Cannot locate node of type [%s] in package [%s]"%(node.type, node.package))
    return [cmd] + args        
    
class NodeParamsException(Exception): pass

## Factory for generating processes for launching local ROS
## nodes. Also registers the process with the ProcessMonitor so that
## events can be generated when the process dies.
## @param node Node: node to launch
## @param master_uri str: API URI for master node
## @return LocalProcess local process instance
## @raise NodeParamsException If the node's parameters are improperly specific
def create_node_process(node, master_uri):
    # check input args
    machine = node.machine
    if machine is None:
        raise RLException("Internal error: no machine selected for node of type [%s/%s]"%(node.package, node.type))
    
    # - setup env for process (vars must be strings for os.environ)
    env = setup_env(node, machine, master_uri)
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
        
    def _validate(self):
        try:
            # verify package exists
            roslib.packages.get_pkg_dir(self.package, True)
        except:
            printerrlog("[%s]: cannot start, package [%s] could not be found"%(self.name, self.package))
            return False
        
        paths = [self.env.get(ROS_ROOT, None), self.args[0]]
        paths.extend(self.env.get(ROS_PACKAGE_PATH, '').split(os.pathsep))
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

            full_env = os.environ.copy()
            full_env.update(self.env)
            # send stdout/stderr to file. in the case of respawning, we have to
            # open in append mode
            
            # note: logfileerr: disabling in favor of stderr appearing in the console.
            # will likely reinstate once roserr/rosout is more properly used.
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
                
            if not logfileerr:
                self.popen = subprocess.Popen(self.args, cwd=cwd, stdout=logfileout, env=full_env, close_fds=True, preexec_fn=os.setsid)
            else:
                self.popen = subprocess.Popen(self.args, cwd=cwd, stdout=logfileout, stderr=logfileerr, env=full_env, close_fds=True, preexec_fn=os.setsid)
            self.started = True
            if self.popen.poll() is None:
                pid = self.popen.pid
                printlog_bold("process[%s]: started with pid [%s]"%(self.name, pid))
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
        _logger.info("process[%s]: killing os process with pid[%s]", self.name, pid)
        try:
            # Start with SIGINT and escalate from there.
            _logger.info("[%s] sending SIGINT", self.name)                                    
            os.killpg(os.getpgid(pid), signal.SIGINT)
            time.sleep(0.5)
            timeout_t = time.time() + _TIMEOUT_SIGINT
            retcode = self.popen.poll()                
            while time.time() < timeout_t and retcode is None:
                time.sleep(0.1)
                retcode = self.popen.poll()
            # Escalate non-responsive process
            if retcode is None:
                printerrlog("[%s] escalating to SIGTERM"%self.name)
                timeout_t = time.time() + _TIMEOUT_SIGTERM
                os.killpg(os.getpgid(pid), signal.SIGTERM)                
                #self.popen.wait()
                retcode = self.popen.poll()
                while time.time() < timeout_t and retcode is None:
                    retcode = self.popen.poll()
                if retcode is None:
                    printerrlog("[%s] escalating to SIGKILL"%self.name)
                    try:
                        os.killpg(os.getpgid(pid), signal.SIGKILL)
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


