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
Integrates roslaunch remote process launching capabilities.
"""

import logging
import socket
import time

import rosgraph.network as network

import roslaunch.config 
import roslaunch.remoteprocess 
from roslaunch.remoteprocess import SSHChildROSLaunchProcess
import roslaunch.launch
import roslaunch.server #ROSLaunchParentNode hidden dep
from roslaunch.core import RLException, is_machine_local, printerrlog, printlog

_CHILD_REGISTER_TIMEOUT = 10.0 #seconds
    
class ROSRemoteRunner(roslaunch.launch.ROSRemoteRunnerIF):
    """
    Manages the running of remote roslaunch children
    """
    
    def __init__(self, run_id, rosconfig, pm, server):
        """
        :param run_id: roslaunch run_id of this runner, ``str``
        :param config: launch configuration, ``ROSConfig``
        :param pm process monitor, ``ProcessMonitor``
        :param server: roslaunch parent server, ``ROSLaunchParentNode``
        """
        self.run_id = run_id
        self.rosconfig = rosconfig
        self.server = server
        self.pm = pm
        self.logger = logging.getLogger('roslaunch.remote')
        self.listeners = []
        
        self.machine_list = []
        self.remote_processes = []

    def add_process_listener(self, l):
        """
        Listen to events about remote processes dying. Not
        threadsafe. Must be called before processes started.

        :param l: ProcessListener 
        """
        self.listeners.append(l)

    def _start_child(self, server_node_uri, machine, counter):
        # generate a name for the machine. don't use config key as
        # it's too long to easily display
        name = "%s-%s"%(machine.address, counter)

        self.logger.info("remote[%s] starting roslaunch", name)
        printlog("remote[%s] starting roslaunch"%name)
            
        p = SSHChildROSLaunchProcess(self.run_id, name, server_node_uri, machine, self.rosconfig.master.uri)
        success = p.start()
        self.pm.register(p)
        if not success: #treat as fatal
            raise RLException("unable to start remote roslaunch child: %s"%name)
        self.server.add_child(name, p)
        return p

    def start_children(self):
        """
        Start the child roslaunch processes
        """
        server_node_uri = self.server.uri
        if not server_node_uri:
            raise RLException("server URI is not initialized")
        
        # TODOXXX: break out table building code into a separate
        # routine so we can unit test it _start_child() should not be
        # determining the process name

        # Build table of unique machines that we are going to launch on
        machines = {}
        for n in self.rosconfig.nodes:
            if not is_machine_local(n.machine):
                machines[n.machine.config_key()] = n.machine

        # Launch child roslaunch processes on remote machines
        counter = 0
        #  - keep a list of procs so we can check for those that failed to launch
        procs = []
        for m in machines:
            p = self._start_child(server_node_uri, machines[m], counter)
            procs.append(p)
            counter += 1
        
        # Wait for all children to call register() callback. The machines can have
        # non-uniform registration timeouts. We consider the failure to occur once
        # one of the machines has failed to meet it's timeout.
        start_t = time.time() 
        while True:
            pending = []
            for p in procs:
                if not p.is_alive():
                    raise RLException("remote roslaunch failed to launch: %s"%p.machine.name)
                elif not p.uri:
                    pending.append(p.machine)
            if not pending:
                break
            # timeout is the minimum of the remaining timeouts of the machines
            timeout_t = start_t + min([m.timeout for m in pending])
            if time.time() > timeout_t:
                break
            time.sleep(0.1)
        if pending:
            raise RLException(
                """The following roslaunch remote processes failed to register: 
%s

If this is a network latency issue, you may wish to consider setting 
  <machine timeout="NUMBER OF SECONDS" ... />
in your launch"""%'\n'.join([" * %s (timeout %ss)"%(m.name, m.timeout) for m in pending]))

        # convert machine dictionary to a list
        self.machine_list = machines.values()
        # save a list of the remote processes
        self.remote_processes = procs


    def _assume_failed(self, nodes, failed):
        """
        Utility routine for logging/recording nodes that failed

        :param nodes: list of nodes that are assumed to have failed, ``Node``
        :param failed: list of names of nodes that have failed to extend, ``[str]``
        """
        str_nodes = ["%s/%s"%(n.package, n.type) for n in nodes]
        failed.extend(str_nodes)
        printerrlog("Launch of the following nodes most likely failed: %s"%', '.join(str_nodes))
        
    def launch_remote_nodes(self):
        """
        Contact each child to launch remote nodes
        """
        succeeded = []
        failed = []
        
        # initialize remote_nodes. we use the machine config key as
        # the key for the dictionary so that we can bin the nodes.
        self.remote_nodes = {}
        for m in self.machine_list:
            self.remote_nodes[m.config_key()] = []
            
        # build list of nodes that will be launched by machine
        nodes = [x for x in self.rosconfig.nodes if not is_machine_local(x.machine)]
        for n in nodes:
            self.remote_nodes[n.machine.config_key()].append(n)
            
        for child in self.remote_processes:
            nodes = self.remote_nodes[child.machine.config_key()]
            body = '\n'.join([n.to_remote_xml() for n in nodes])
            # #3799: force utf-8 encoding 
            xml = '<?xml version="1.0" encoding="utf-8"?>\n<launch>\n%s</launch>'%body 
                
            api = child.getapi()
            # TODO: timeouts
            try:
                self.logger.debug("sending [%s] XML [\n%s\n]"%(child.uri, xml))
                code, msg, val = api.launch(xml)
                if code == 1:
                    c_succ, c_fail = val
                    succeeded.extend(c_succ)
                    failed.extend(c_fail)
                else:
                    printerrlog('error launching on [%s, uri %s]: %s'%(child.name, child.uri, msg))
                    self._assume_failed(nodes, failed)
            except socket.error as e:
                errno, msg = e
                printerrlog('error launching on [%s, uri %s]: %s'%(child.name, child.uri, str(msg)))
                self._assume_failed(nodes, failed)

            except socket.gaierror as e:
                errno, msg = e
                # usually errno == -2. See #815. 
                child_host, _ = network.parse_http_host_and_port(child.uri)
                printerrlog("Unable to contact remote roslaunch at [%s]. This is most likely due to a network misconfiguration with host lookups. Please make sure that you can contact '%s' from this machine"%(child.uri, child_host))
                self._assume_failed(nodes, failed)

            except Exception as e:
                printerrlog('error launching on [%s, uri %s]: %s'%(child.name, child.uri, str(e)))
                self._assume_failed(nodes, failed)

        return succeeded, failed

