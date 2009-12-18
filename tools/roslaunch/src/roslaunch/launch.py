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
Top-level implementation of launching processes. Coordinates
lower-level libraries.
"""

import os
import logging
import sys
import time

import roslib.network 

from roslaunch.core import *
from roslaunch.config import ROSLaunchConfig
from roslaunch.nodeprocess import create_master_process, create_node_process
from roslaunch.pmon import start_process_monitor, ProcessListener, FatalProcessLaunch

from roslaunch.rlutil import update_terminal_name

_TIMEOUT_MASTER_START = 10.0 #seconds
_TIMEOUT_MASTER_STOP  = 10.0 #seconds

_ID = '/roslaunch'

class RLTestTimeoutException(RLException): pass

class _ROSLaunchListeners(ProcessListener):
    """
    Helper class to manage distributing process events. It functions as
    a higher level aggregator of events. In particular, events about
    remote and local processes require different mechanisms for being
    caught and reported.
    """
    def __init__(self):
        self.process_listeners = []

    def add_process_listener(self, l):
        """
        Add listener to list of listeners. Not threadsafe.
        @param l: listener
        @type  l: L{ProcessListener}
        """
        self.process_listeners.append(l)

    def process_died(self, process_name, exit_code):
        """
        ProcessListener callback
        """
        for l in self.process_listeners:
            try:
                l.process_died(process_name, exit_code)
            except Exception, e:
                import traceback
                logging.getLogger('roslaunch').error(traceback.format_exc())
                
class ROSLaunchListener(object):
    """
    Listener interface for events related to ROSLaunch.
    ROSLaunchListener is currently identical to the lower-level
    L{roslaunch.pmon.ProcessListener} interface, but is separate for
    architectural reasons. The lower-level
    L{roslaunch.pmon.ProcessMonitor} does not provide events about
    remotely running processes.
    """
    
    def process_died(self, process_name, exit_code):
        """
        Notifies listener that process has died. This callback only
        occurs for processes that die during normal process monitor
        execution -- processes that are forcibly killed during
        L{roslaunch.pmon.ProcessMonitor} shutdown are not reported.
        @param process_name: name of process
        @type  process_name: str
        @param exit_code int: exit code of process. If None, it means
            that L{roslaunch.pmon.ProcessMonitor} was unable to determine an exit code.
        @type  exit_code: int
        """
        pass
    
class ROSLaunchRunner(object):
    """
    Runs a roslaunch. The normal sequence of API calls is L{launch()}
    followed by L{spin()}. An external thread can call L{stop()}; otherwise
    the runner will block until an exit signal. Another usage is to
    call L{launch()} followed by repeated calls to L{spin_once()}. This usage
    allows the main thread to continue to do work while processes are
    monitored.
    """
    
    def __init__(self, run_id, config, server_uri=None, pmon=None, is_core=False, remote_runner=None, is_child=False):
        """
        @param run_id: /run_id for this launch. If the core is not
            running, this value will be used to initialize /run_id. If
            the core is already running, this value will be checked
            against the value stored on the core. L{ROSLaunchRunner} will
            fail during L{launch()} if they do not match.
        @type  run_id: str            
        @param config: roslauch instance to run
        @type  config: L{ROSLaunchConfig}
        @param server_uri: XML-RPC URI of roslaunch server. 
        @type  server_uri: str
        @param pmon: optionally override the process
            monitor the runner uses for starting and tracking processes
        @type  pmon: L{ProcessMonitor}
    
        @param is_core: if True, this runner is a roscore
            instance. This affects the error behavior if a master is
            already running -- aborts if is_core is True and a core is
            detected.
        @type  is_core: bool
        @param remote_runner: remote roslaunch process runner
        """            
        if run_id is None:
            raise RLException("run_id is None")
        self.run_id = run_id
        self.config = config
        self.server_uri = server_uri
        self.is_child = is_child
        self.is_core = is_core
        self.logger = logging.getLogger('roslaunch')
        self.pm = pmon or start_process_monitor()

        # wire in ProcessMonitor events to our listeners
        # aggregator. We similarly wire in the remote events when we
        # create the remote runner.
        self.listeners = _ROSLaunchListeners()
        if self.pm is None:
            raise RLException("unable to initialize roslaunch process monitor")
        if self.pm.is_shutdown:
            raise RLException("bad roslaunch process monitor initialization: process monitor is already dead")
        
        self.pm.add_process_listener(self.listeners)
        
        self.remote_runner = remote_runner
                
    def add_process_listener(self, l):
        """
        Add listener to list of listeners. Not threadsafe. Must be
        called before processes started.
        @param l: listener
        @type  l: L{ProcessListener}
        """
        self.listeners.add_process_listener(l)

    def _load_parameters(self):
        """
        Load parameters onto the parameter server
        """
        self.logger.info("load_parameters starting ...")
        config = self.config
        param_server = config.master.get()
        p = None
        try:
            for p in config.clear_params:
                if param_server.hasParam(_ID, p)[2]:
                    printlog("deleting parameter [%s]"%p)
                    code, msg, _ = param_server.deleteParam(_ID, p)
                    if code != 1:
                        raise RLException("Failed to clear parameter [%s]: %s"%(p, msg))
            for p in config.params.itervalues():
                # suppressing this as it causes too much spam
                #printlog("setting parameter [%s]"%p.key)
                code, msg, _ = param_server.setParam(_ID, p.key, p.value)
                if code != 1:
                    raise RLException("Failed to set parameter [%s] to [%s]"%(p.key, p.value))
        except RLException:
            raise
        except Exception, e:
            printerrlog("load_parameters: unable to set parameters (last param was [%s]): %s"%(p,e))
            raise #re-raise as this is fatal
        self.logger.info("... load_parameters complete")            

    def _launch_nodes(self):
        """
        Launch all the declared nodes/master
        @return: two lists of node names where the first
        is the nodes that successfully launched and the second is the
        nodes that failed to launch.
        @rtype: [[str], [str]]
        """        
        config = self.config
        succeeded = []
        failed = []
        self.logger.info("launch_nodes: launching local nodes ...")
        local_nodes = config.nodes

        # don't launch remote nodes
        local_nodes = [n for n in config.nodes if is_machine_local(n.machine)]
            
        for node in local_nodes:
            name, success = self._launch_node(node)
            if success:
                succeeded.append(name)
            else:
                failed.append(name)

        if self.remote_runner:
            self.logger.info("launch_nodes: launching remote nodes ...")
            r_succ, r_fail = self.remote_runner.launch_remote_nodes()
            succeeded.extend(r_succ)
            failed.extend(r_fail)            
                
        self.logger.info("... launch_nodes complete")
        return succeeded, failed

    def _setup_master(self):
        """
        Validates master configuration and changes the master URI if
        necessary. Also shuts down any existing master.
        @raise RLException: if existing master cannot be killed
        """
        m = self.config.master
        self.logger.info("initial ROS_MASTER_URI is %s", m.uri)     
        if m.auto in [m.AUTO_START, m.AUTO_RESTART]:
            running = m.is_running() #save state as calls are expensive
            if m.auto == m.AUTO_RESTART and running:
                print "shutting down existing master"
                try:
                    m.get().shutdown(_ID, 'roslaunch restart request')
                except:
                    pass
                timeout_t = time.time() + _TIMEOUT_MASTER_STOP
                while m.is_running() and time.time() < timeout_t:
                    time.sleep(0.1)
                if m.is_running():
                    raise RLException("ERROR: could not stop existing master")
                running = False
            if not running:
                # force the master URI to be for this machine as we are starting it locally
                olduri = m.uri
                m.uri = remap_localhost_uri(m.uri, True)

                # this block does some basic DNS checks so that we can
                # warn the user in the _very_ common case that their
                # hostnames are not configured properly
                hostname, _ = roslib.network.parse_http_host_and_port(m.uri)
                local_addrs = roslib.network.get_local_addresses()
                import socket
                reverse_ip = socket.gethostbyname(hostname)
                if reverse_ip not in local_addrs:
                    self.logger.warn("IP address %s local hostname '%s' not in local addresses (%s)."%(reverse_ip, hostname, ','.join(local_addrs)))
                    print >> sys.stderr, \
"""WARNING: IP address %s for local hostname '%s' does not appear to match
any local IP address (%s). Your ROS nodes may fail to communicate.

Please use ROS_IP to set the correct IP address to use."""%(reverse_ip, hostname, ','.join(local_addrs))

                if m.uri != olduri:
                    self.logger.info("changing ROS_MASTER_URI to [%s] for starting master locally", m.uri)
                    print "changing ROS_MASTER_URI to [%s] for starting master locally"%m.uri

    def _launch_master(self):
        """
        Launches master if requested. Must be run after L{_setup_master()}.
        @raise RLException: if master launch fails
        """
        m = self.config.master
        auto = m.auto
        is_running = m.is_running()

        if self.is_core and is_running:
            raise RLException("roscore cannot run as another roscore/master is already running. \nPlease kill other roscore/zenmaster processes before relaunching")

        self.logger.debug("launch_master [%s]", auto)
        if auto in [m.AUTO_START, m.AUTO_RESTART] and not is_running:
            if auto == m.AUTO_START:
                printlog("starting new master (master configured for auto start)")
            elif auto == m.AUTO_RESTART:
                printlog("starting new master (master configured for auto restart)")
                
            _, urlport = roslib.network.parse_http_host_and_port(m.uri)
            if urlport <= 0:
                raise RLException("ERROR: master URI is not a valid XML-RPC URI. Value is [%s]"%m.uri)

            p = create_master_process(self.run_id, m.type, get_ros_root(), urlport, m.log_output)
            self.pm.register_core_proc(p)
            success = p.start()
            if not success:
                raise RLException("ERROR: unable to auto-start master process")
            timeout_t = time.time() + _TIMEOUT_MASTER_START
            while not m.is_running() and time.time() < timeout_t:
                time.sleep(0.1)

        if not m.is_running():
            raise RLException("ERROR: could not contact master [%s]"%m.uri)
        
        printlog_bold("ROS_MASTER_URI=%s"%m.uri)
        # TODO: although this dependency doesn't cause anything bad,
        # it's still odd for launch to know about console stuff. This
        # really should be an event.
        update_terminal_name(m.uri)

        # Param Server config params
        param_server = m.get()
        
        # #773: unique run ID
        self._check_and_set_run_id(param_server, self.run_id)

        if self.server_uri:
            # store parent XML-RPC URI on param server
            # - param name is the /roslaunch/hostname:port so that multiple roslaunches can store at once
            hostname, port = roslib.network.parse_http_host_and_port(self.server_uri)
            self.logger.info("setting /roslaunch/uris/%s:%s' to %s"%(hostname, port, self.server_uri))
            param_server.setParam(_ID, '/roslaunch/uris/%s:%s'%(hostname, port),self.server_uri)

    def _check_and_set_run_id(self, param_server, run_id):
        """
        Initialize self.run_id to existing value or setup parameter
        server with /run_id set to default_run_id
        @param default_run_id: run_id to use if value is not set
        @type  default_run_id: str
        @param param_server: parameter server proxy
        @type  param_server: xmlrpclib.ServerProxy
        """
        code, _, val = param_server.hasParam(_ID, '/run_id')
        if code == 1 and not val:
            printlog_bold("setting /run_id to %s"%run_id)
            param_server.setParam('/roslaunch', '/run_id', run_id)
        else:
            # verify that the run_id we have been set to matches what's on the parameter server
            code, _, val = param_server.getParam('/roslaunch', '/run_id')
            if code != 1:
                #could only happen in a bad race condition with
                #someone else restarting core
                raise RLException("ERROR: unable to retrieve /run_id from parameter server")
            if run_id != val:
                raise RLException("run_id on parameter server does not match declared run_id: %s vs %s"%(val, run_id))
            #self.run_id = val
            #printlog_bold("/run_id is %s"%self.run_id)            

    def _launch_executable(self, e):
        """
        Launch a single L{Executable} object. Blocks until executable finishes.
        @param e: Executable
        @type  e: L{Executable}
        @raise RLException: if exectuable fails. Failure includes non-zero exit code.
        """
        try:
            #kwc: I'm still debating whether shell=True is proper
            cmd = e.command
            if isinstance(e, RosbinExecutable):
                cmd = os.path.join(get_ros_root(), 'bin', cmd)
            cmd = "%s %s"%(cmd, ' '.join(e.args))
            print "running %s"%cmd
            local_machine = self.config.machines['']
            import roslaunch.node_args
            env = roslaunch.node_args.create_local_process_env(None, local_machine, self.config.master.uri)
            import subprocess
            retcode = subprocess.call(cmd, shell=True, env=env)
            if retcode < 0:
                raise RLException("command [%s] failed with exit code %s"%(cmd, retcode))
        except OSError, e:
            raise RLException("command [%s] failed: %s"%(cmd, e))
        
    #TODO: _launch_run_executables, _launch_teardown_executables
    #TODO: define and implement behavior for remote launch
    def _launch_setup_executables(self):
        """
        @raise RLException: if exectuable fails. Failure includes non-zero exit code.
        """
        exes = [e for e in self.config.executables if e.phase == PHASE_SETUP]
        for e in exes:
            self._launch_executable(e)
    
    def _launch_core_nodes(self):
        """
        launch any core services that are not already running. master must
        be already running
        @raise RLException: if core launches fail
        """
        import roslib.names

        config = self.config
        master = config.master.get()
        tolaunch = []
        for node in config.nodes_core:
            node_name = roslib.names.ns_join(node.namespace, node.name)
            code, msg, _ = master.lookupNode(_ID, node_name)
            if code == -1:
                tolaunch.append(node)
            elif code == 1:
                print "core service [%s] found"%node_name
            else:
                print >> sys.stderr, "WARN: master is not behaving well (unexpected return value when looking up node)"
                self.logger.error("ERROR: master return [%s][%s] on lookupNode call"%(code,msg))
                
        for node in tolaunch:
            node_name = roslib.names.ns_join(node.namespace, node.name)
            name, success = self._launch_node(node, core=True)
            if success:
                print "started core service [%s]"%node_name
            else:
                raise RLException("failed to start core service [%s]"%node_name)

    def _launch_node(self, node, core=False):
        """
        Launch a single node locally. Remote launching is handled separately by the remote module.
        @param node Node: node to launch
        @param core bool: if True, core node
        @return str, bool: node process name, successful launch
        """
        self.logger.info("... preparing to launch node of type [%s/%s]", node.package, node.type)
        master = self.config.master
        import roslaunch.node_args
        try:
            process = create_node_process(self.run_id, node, master.uri)
        except roslaunch.node_args.NodeParamsException, e:
            self.logger.error(e)
            if node.package == 'rosout' and node.type == 'rosout':
                printerrlog("\n\n\nERROR: rosout is not built. Please run 'rosmake rosout'\n\n\n")
            else:
                printerrlog("ERROR: cannot launch node of type [%s/%s]: %s"%(node.package, node.type, str(e)))
            if node.name:
                return node.name, False
            else:
                return "%s/%s"%(node.package,node.type), False                

        self.logger.info("... created process [%s]", process.name)
        if core:
            self.pm.register_core_proc(process)
        else:
            self.pm.register(process)            
        node.process_name = process.name #store this in the node object for easy reference
        self.logger.info("... registered process [%s]", process.name)

        # note: this may raise FatalProcessLaunch, which aborts the entire launch
        success = process.start()
        if not success:
            if node.machine.name:
                printerrlog("launch of %s/%s on %s failed"%(node.package, node.type, node.machine.name))
            else:
                printerrlog("local launch of %s/%s failed"%(node.package, node.type))   
        else:
            self.logger.info("... successfully launched [%s]", process.name)
        return process.name, success
        
    def is_node_running(self, node):
        """
        Check for running node process.
        @param node Node: node object to check
        @return bool: True if process associated with node is running (launched && !dead)
        """
        #process_name is not set until node is launched.
        return node.process_name and self.pm.has_process(node.process_name)
    
    def spin_once(self):
        """
        Same as spin() but only does one cycle. must be run from the main thread.
        """
        if not self.pm:
            return False
        return self.pm.mainthread_spin_once()
        
    def spin(self):
        """
        spin() must be run from the main thread. spin() is very
        important for roslaunch as it picks up jobs that the process
        monitor need to be run in the main thread.
        """
        self.logger.info("spin")

        # #556: if we're just setting parameters and aren't launching
        # any processes, exit.
        if not self.pm or not self.pm.get_active_names():
            printlog_bold("No processes to monitor")
            self.stop()
            return # no processes
        self.pm.mainthread_spin()
        #self.pm.join()
        self.logger.info("process monitor is done spinning, initiating full shutdown")
        self.stop()
        printlog_bold("done")
    
    def stop(self):
        """
        Stop the launch and all associated processes. not thread-safe.
        """        
        self.logger.info("runner.stop()")
        if self.pm is not None:
            printlog("shutting down processing monitor...")
            self.logger.info("shutting down processing monitor %s"%self.pm)            
            self.pm.shutdown()
            self.pm.join()
            self.pm = None
            printlog("... shutting down processing monitor complete")
        else:
            self.logger.info("... roslaunch runner has already been stopped")

    def _setup(self):
        """
        Setup the state of the ROS network, including the parameter
        server state and core services
        """
        # this may have already been done, but do just in case
        self.config.assign_machines()
        
        # have to do setup on master before launching remote roslaunch
        # children as we may be changing the ROS_MASTER_URI.
        self._setup_master()

        if self.remote_runner:
            # hook in our listener aggregator
            self.remote_runner.add_process_listener(self.listeners)            

        # start up the core: master + core nodes defined in core.xml
        self._launch_master()
        self._launch_core_nodes()        
        
        # run exectuables marked as setup period. this will block
        # until these executables exit. setup executable have to run
        # *before* parameters are uploaded so that commands like
        # rosparam delete can execute.
        self._launch_setup_executables()

        # no parameters for a child process
        if not self.is_child:
            self._load_parameters()

    def launch(self):
        """
        Run the launch. Depending on usage, caller should call
        spin_once or spin as appropriate after launch().
        @return ([str], [str]): tuple containing list of nodes that
            successfully launches and list of nodes that failed to launch
        @rtype: ([str], [str])
        @raise RLException: if launch fails (e.g. run_id parameter does
        not match ID on parameter server)
        """        
        self._setup()        
        succeeded, failed = self._launch_nodes()
        # inform process monitor that we are done with process registration
        self.pm.registrations_complete()
        return succeeded, failed 

    def run_test(self, test):
        """
        Run the test node. Blocks until completion or timeout.
        @param test: test node to run
        @type  test: Test
        @raise RLTestTimeoutException: if test fails to launch or test times out
        """
        self.logger.info("... preparing to run test [%s] of type [%s/%s]", test.test_name, test.package, test.type)
        name, success = self._launch_node(test)
        if not success:
            raise RLException("test [%s] failed to launch"%test.test_name)

        #poll until test terminates or alloted time exceed
        timeout_t = time.time() + test.time_limit
        pm = self.pm
        while pm.mainthread_spin_once() and self.is_node_running(test):
            #test fails on timeout
            if time.time() > timeout_t:
                raise RLTestTimeoutException("test max time allotted")
            time.sleep(0.1)
        
# NOTE: the mainly exists to prevent implicit circular dependency as
# the runner needs to invoke methods on the remote API, which depends
# on launch.

class ROSRemoteRunnerIF(object):
    """
    API for remote running
    """
    def __init__(self):
        pass
    def setup(self):
        pass
    def add_process_listener(self, l):
        """
        Listen to events about remote processes dying. Not
        threadsafe. Must be called before processes started.
        @param l: listener
        @type  l: L{ProcessListener}
        """
        pass

    def launch_remote_nodes(self):
        """
        Contact each child to launch remote nodes
        @return: succeeded, failed
        @rtype: [str], [str]
        """
        pass
