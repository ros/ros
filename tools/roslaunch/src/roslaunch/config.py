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
# Revision $Id: launch.py 2165 2008-09-17 22:38:49Z sfkwc $

## Defines the ROSLaunchConfig/ROSLaunchRunner object, which holds and executes the state
## of the roslaunch file

import os
import logging
import sys
import traceback
import socket
import random
import types

import roslib.network
import roslib.packages
import roslib.rosenv

from roslaunch.core import Master, local_machine, get_ros_root, is_machine_local, RLException
import roslaunch.xmlloader

## Load roscore configuration into the ROSLaunchConfig using the specified XmlLoader
## @param config ROSLaunchConfig
## @param loader XmlLoader
def load_roscore(loader, config, verbose=True):
    f_roscore = os.path.join(roslib.packages.get_pkg_dir('roslaunch'), 'roscore.xml')
    logging.getLogger('roslaunch').info('loading roscore config file %s'%f_roscore)            
    loader.load(f_roscore, config, core=True, verbose=verbose)    
        
## generate summary label for node based on its package, type, and name
def _summary_name(node):
    if node.name:
        return "%s (%s/%s)"%(node.name, node.package, node.type)
    else:
        return "%s/%s"%(node.package, node.type)
    
## ROSLaunchConfig is the container for the loaded roslaunch file state. It also
## is responsible for validating then executing the desired state. 
class ROSLaunchConfig(object):

    ## ctor
    ## @param self
    def __init__(self):
        self.master = Master()
        self.nodes_core = [] 
        self.nodes    = [] #nodes are unnamed
        
        # list of resolved node names. This is so that we can check for naming collisions
        self.resolved_node_names = []
        
        self.tests    = [] 
        self.machines = {} #key is name
        self.params   = {} #key is name
        self.clear_params = []
        self.executables = []

        # for tools like roswtf
        self.config_errors = []
        
        m = local_machine() #for local exec
        self.machines[m.name] = m
        self._assign_machines_complete = False
        self._remote_nodes_present = None

        self.logger = logging.getLogger('roslaunch')

    ## Report human-readable error message related to configuration error
    ## @param msg: error message
    def add_config_error(self, msg):
        self.config_errors.append(msg)

    ## Set the master configuration
    ## @param self
    ## @param m Master            
    def set_master(self, m):
        self.master = m

    ## @param self
    ## @return bool True if roslaunch will launch nodes on a remote machine
    def has_remote_nodes(self):
        if not self._assign_machines_complete:
            raise Exception("ERROR: has_remote_nodes() cannot be called until prelaunch check is complete")
        return self._remote_nodes_present
    
    ## Assign nodes to machines and determine whether or not there are any remote machines
    def assign_machines(self):
        # don't repeat machine assignment
        if self._assign_machines_complete:
            return
        
        machine_unify_dict = {}
        
        self._assign_machines_complete = True
        # #653: current have to set all core nodes to local launch
        local_machine = self.machines['']
        for n in self.nodes_core:
            n.machine = local_machine

        #for n in self.nodes_core + self.nodes + self.tests:
        for n in self.nodes + self.tests:
            m = self._select_machine(n)
            
            # if machines have the same config keys it means that they are identical except
            # for their name. we unify the machine assignments so that we don't use
            # extra resources.
            config_key = m.config_key()
            if config_key in machine_unify_dict:
                new_m = machine_unify_dict[config_key]
                if m != new_m:
                    self.logger.info("... changing machine assignment from [%s] to [%s] as they are equivalent", m.name, new_m.name)
                    m = new_m
            else:
                machine_unify_dict[config_key] = m
            n.machine = m
            self.logger.info("... selected machine [%s] for node of type [%s/%s]", m.name, n.package, n.type)

        # determine whether or not there are any machines we will need
        # to setup remote roslaunch clients for
        self._remote_nodes_present = False 
        if [m for m in machine_unify_dict.itervalues() if not is_machine_local(m)]:
            self._remote_nodes_present = True

    ## Perform basic checks on the local ROS environment, master, and core services.
    ## master will be launched if configured to do so. Core services will be launched regardless.
    ## if they are not already running.
    ##  @throws RLException if validation fails
    def validate(self):
        ros_root = get_ros_root()
        if not os.path.isdir(ros_root):
            raise RLException("ERROR: ROS_ROOT is not configured properly. Value is [%s]"%ros_root)
        
    ## Get a human-readable string summary of the launch
    ## @param self
    ## @param local bool: if True, only print local nodes
    ## @return str: summary
    def summary(self, local=False):
        summary = '\nSUMMARY\n========'
        if self.clear_params:
            summary += '\n\nCLEAR PARAMETERS\n' + '\n'.join([' * %s'%p for p in self.clear_params])
        if self.params:
            summary += '\n\nPARAMETERS\n' + '\n'.join([' * %s'%k for k in self.params])
        if not local:
            summary += '\n\nMACHINES\n' + '\n'.join([' * %s'%k for k in self.machines if k])
        summary += '\n\nNODES\n'
        namespaces = {}
        if local:
            nodes = [n for n in self.nodes if is_machine_local(n.machine)]
        else:
            nodes = self.nodes
        for n in nodes:
            ns = n.namespace
            if ns not in namespaces:
                namespaces[ns] = [n]
            else:
                namespaces[ns].append(n)
        for k,v in namespaces.iteritems():
            summary += '  %s\n'%k + '\n'.join(['    %s'%_summary_name(n) for n in v])
            summary += '\n'
        return summary

    ## Declare an exectuable to be run during the launch
    ## @param self
    ## @param exe Executable
    def add_executable(self, exe):
        if not exe:
            raise ValueError("exe is None")
        self.executables.append(exe)
        
    ## Declare a parameter to be cleared before new parameters are set
    ## @param self
    ## @param param str: parameter to clear
    def add_clear_param(self, param):
        self.clear_params.append(param)

    ## Declare a parameter to be set on the param server before launching nodes
    ## @param self
    ## @param p Param: parameter instance
    def add_param(self, p, filename=None, verbose=True):
        key = p.key
        if key in self.params and self.params[key] != p:
            if filename:
                self.logger.debug("[%s] overriding parameter [%s]"%(filename, p.key))
            else:
                self.logger.debug("overriding parameter [%s]"%p.key)                
        self.params[key] = p
        if verbose:
            print "Added parameter [%s]"%key
        t = type(p.value)
        if t in [str, unicode, types.InstanceType]:
            self.logger.debug("add_param[%s]: type [%s]"%(p.key, t))
        else:
            self.logger.debug("add_param[%s]: type [%s] value [%s]"%(p.key, t, p.value))
            
    ## Declare a machine and associated parameters so that it can be used for
    ## running nodes.
    ## @param self
    ## @param m Machine: machine instance
    ## @return bool: True if new machine added, False if machine already specified.
    ## @throws RLException if cannot add machine as specified
    def add_machine(self, m, verbose=True):
        name = m.name
        if m.address == 'localhost': #simplify address comparison
            address = roslib.network.get_local_address()
            self.logger.info("addMachine[%s]: remapping localhost address to %s"%(name, address))
        if name in self.machines:
            if m != self.machines[name]:
                raise RLException("Machine [%s] already added and does not match duplicate entry"%name)
            return False
        else:
            self.machines[name] = m
            if verbose:
                print "Added machine [%s]"%name
            return True

    ## Add test declaration. Used by rostest
    ## @param self
    ## @param test Test: test node instance to add to launch
    def add_test(self, test, verbose=True):
        self.tests.append(test)

    ## Add node declaration
    ## @param self
    ## @param node Node: node instance to add to launch
    ## @param core bool: if True, node is a ROS core node
    ## @raise RLException if ROS core node is missing required name
    def add_node(self, node, core=False, verbose=True):
        if node.name:
            # check for duplicates
            resolved_name = roslib.names.ns_join(node.namespace, node.name)
            matches = [n for n in self.resolved_node_names if n == resolved_name]
            if matches:
                raise RLException("roslaunch file contains multiple nodes named [%s].\nPlease check all <node> 'name' attributes to make sure they are unique.\nAlso check that $(anon id) use different ids."%resolved_name)
            else:
                self.resolved_node_names.append(resolved_name)
        
        if not core:
            self.nodes.append(node)
            if verbose:
                print "Added node of type [%s/%s] in namespace [%s]"%(node.package, node.type, node.namespace)
            self.logger.info("Added node of type [%s/%s] in namespace [%s]", node.package, node.type, node.namespace)
        else:
            if not node.name:
                raise RLException("ROS core nodes must have a name. [%s/%s]"%(node.package, node.type))
            self.nodes_core.append(node)
            if verbose:
                print "Added core node of type [%s/%s] in namespace [%s]"%(node.package, node.type, node.namespace)
            self.logger.info("Added core node of type [%s/%s] in namespace [%s]", node.package, node.type, node.namespace)
            
    ## Select a machine for a node to run on. For nodes that are
    ## already assigned to a machine, this will map the string name to
    ## a Machine instance. If the node isn't already tagged with a
    ## particular machine, one will be selected for it.
    ## @param self
    ## @param node Node: node to assign machine for
    ## @return Machine: machine to run on
    ## @throws RLException If machine state is improperly configured
    def _select_machine(self, node):
        machine = node.machine_name
        #Lookup machine
        if machine:
            if not machine in self.machines:
                raise RLException("ERROR: unknown machine [%s]"%machine)
            return self.machines[machine]
        else:
            # assign to local machine
            return self.machines['']            


        
## Base routine for creating a ROSLaunchConfig from a set of \a
## roslaunch_files and or launch XML strings and initializing it. This
## config will have a core definition and also set the master to run
## on \a port.
## @param roslaunch_files [str]
## @param port int: roscore/master port
## @param roslaunch_strs [str]: roslaunch XML strings to load
## @return ROSLaunchConfig initialized rosconfig instance
def load_config_default(roslaunch_files, port, roslaunch_strs=None, loader=None, verbose=True):
    logger = logging.getLogger('roslaunch.config')
    
    # This is the main roslaunch server process. Load up the
    # files specified on the command line and launch the
    # requested resourcs.
            
    config = ROSLaunchConfig()
    loader = loader or roslaunch.xmlloader.XmlLoader()

    # load the roscore file first. we currently have
    # last-declaration wins rules.  roscore is just a
    # roslaunch file with special load semantics
    load_roscore(loader, config, verbose=verbose)

    # load the roslaunch_files into the config
    for f in roslaunch_files:
        try:
            logger.info('loading config file %s'%f)
            loader.load(f, config, verbose=verbose)
        except roslaunch.xmlloader.XmlParseException, e:
            raise RLException(e)
        except roslaunch.xmlloader.XmlLoadException, e:
            raise RLException(e)
        
    # we need this for the hardware test systems, which builds up
    # roslaunch launch files in memory
    if roslaunch_strs:
        for launch_str in roslaunch_strs:
            try:
                logger.info('loading config file from string')
                loader.load_string(launch_str, config)
            except roslaunch.xmlloader.XmlParseException, e:
                raise RLException('Launch string: %s\nException: %s'%(launch_str, e))
            except roslaunch.xmlloader.XmlLoadException, e:
                raise RLException('Launch string: %s\nException: %s'%(launch_str, e))

    if port:
        logger.info("overriding master port to %s"%port)
        config.master.set_port(port)

    # make sure our environment is correct
    config.validate()

    # choose machines for the nodes 
    config.assign_machines()
    return config
    
