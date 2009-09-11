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

import logging
import sys
import time
import traceback
import xmlrpclib

import roslib.network as network
import roslib.xmlrpc as xmlrpc

import roslaunch.pmon
import roslaunch.xmlloader

from roslaunch.config import ROSLaunchConfig
from roslaunch.launch import ROSLaunchRunner
from roslaunch.core import Node, Test, RLException, setup_env, is_machine_local, \
     add_printlog_handler, add_printerrlog_handler, printlog, printerrlog, printlog_bold
from roslaunch.remoteprocess import RemoteRoslaunchProcess

#For using Log message level constants
from roslib.msg import Log

# Uses camel-case network-API naming conventions
## XML-RPC API for the roslaunch server node
class ROSLaunchServerHandler(xmlrpc.XmlRpcHandler):
    
    ## ctor.
    ## @param remote_processes dict: {name : RemoteRoslaunchProcess}.
    ## Map of remote processes so that server can update processes
    ## with information as children register. Handler will not modify
    ## keys.
    ## @param listeners [roslaunch.pmon.ProcessListener]: list of
    ## listeners to notify when process_died events occur.
    def __init__(self, remote_processes, listeners):
        self.remote_processes = remote_processes
        self.listeners = listeners
        
    ## Registration callback from newly launched roslaunch clients
    ## @param self
    ## @param client str: name of client
    ## @param uri str: XML-RPC URI of client    
    ## @return int, str, int: code, msg, ignore
    def register(self, client, uri):
        if client not in self.remote_processes:
            logging.getLogger("roslaunch.remote").error("Unknown child [%s] registered with server", client)
            return -1, "unknown child [%s]"%client, 0
        else:
            self.remote_processes[client].set_uri(uri)
        return 1, "registered", 1

    ## List the roslaunch child processes.
    ## @return int, str, [str]: code, msg, list of the roslaunch children URIS
    def list_children(self):
        return 1, 'roslaunch children', [" * %s\n"%v.uri for v in self.remote_processes.iteritems()]
            
    ## Inform roslaunch server that a remote process has died
    ## @param process_name str: name of process that died
    ## @param exit_code int: exit code of remote process
    def process_died(self, process_name, exit_code):
        for l in self.listeners:
            try:
                l.process_died(process_name, exit_code)
            except:
                logging.getLogger("roslaunch.remote").error(traceback.format_exc())
        
    ## Report a log message to the server
    ## @param client str: name of client
    ## @param level int log level (uses roslib.msg.Log levels)
    ## @param message str: message to log
    def log(self, client, level, message):
        try:
            if level >= Log.ERROR:
                printerrlog("[%s]: %s"%(client, message))
            else:
                #hack due to the fact that we only have one INFO level
                if 'started with pid' in message:
                    printlog_bold("[%s]: %s"%(client, message))
                else:
                    printlog("[%s]: %s"%(client, message))
        except:
            # can't trust the logging system at this point, so just dump to screen
            traceback.print_exc()
        return 1, '', 1

    ## xmlrpc.XmlRpcHandler API: inform handler of shutdown
    ## @param reason str: human-readable shutdown reason
    def _shutdown(self, reason):
        return 1, '', 1
    
## XML-RPC API implementation for child roslaunches
## NOTE: the client handler runs a process monitor so that
## it can track processes across requests
class ROSLaunchClientHandler(xmlrpc.XmlRpcHandler):
    
    ## ctor
    ## @param self
    ## @param server_uri str: XML-RPC URI of server
    ## @param pm ProcessMonitor: process monitor to use
    ## @throws RLException If parameters are invalid
    def __init__(self, name, server_uri, pm):
        if pm is None:
            raise RLException("process monitor is not initialized")
        if server_uri is None:
            raise RLException("server_uri is not initialized")
        
        # parse the URI to make sure it's valid
        _, urlport = network.parse_http_host_and_port(server_uri)
        if urlport <= 0:
            raise RLException("ERROR: roslaunch server URI is not a valid XML-RPC URI. Value is [%s]"%m.uri)

        self.name = name
        self.pm = pm
        self.server_uri = server_uri
        self.server = xmlrpclib.ServerProxy(server_uri)
        self.logger = logging.getLogger('roslaunch.remote')

    ## xmlrpc.XmlRpcHandler API: inform handler of shutdown
    ## @param reason str: human-readable shutdown reason
    def _shutdown(self, reason):
        if self.pm is not None:
            self.pm.shutdown()
            self.pm.join()
            self.pm = None
        
    ## @param self
    ## @return int, str, int: code, msg, ignore
    def shutdown(self):
        self._shutdown("external call")
        return 1, "success", 1
    
    ## log message to log file and roslaunch server
    ## @param level int: log level
    ## @param message str: message to log
    def _log(self, level, message):
        try:
            if self.logger is not None:
                self.logger.debug(message)
            if self.server is not None:
                self.server.log(str(self.name), level, str(message))
        except:
            self.logger.error(traceback.format_exc())
    
    ## Launch the roslaunch XML file. Because this is a child
    ## roslaunch, it will not set parameters nor manipulate the
    ## master. Call blocks until launch is complete
    ## @param self
    ## @param xml str: roslaunch XML file to launch
    ## @return int, str, [ [str], [str] ] : code, msg, [ [ successful launches], [failed launches] ]
    def launch(self, launch_xml):
        if self.pm is None:
            return 0, "uninitialized", -1

        rosconfig = ROSLaunchConfig()
        try:
            roslaunch.xmlloader.XmlLoader().load_string(launch_xml, rosconfig)
        except roslaunch.xmlloader.XmlParseException, e:
            return -1, "ERROR: %s"%e, [[], []]

        # roslaunch clients try to behave like normal roslaunches as much as possible. It's
        # mainly the responsibility of the roslaunch server to not give us any XML that might
        # cause conflict (e.g. master tags, param tags, etc...).
        self._log(Log.INFO, "launching nodes...")
        runner = ROSLaunchRunner(rosconfig, server_uri=self.server_uri, pmon=self.pm)
        succeeded, failed = runner.launch()
        self._log(Log.INFO, "... done launching nodes")
        # enable the process monitor to exit of all processes die
        self.pm.registrations_complete()
        return 1, "launched", [ succeeded, failed ]
    
    ## @param self
    ## @return int, str, int: code, msg, pid
    def getPid(self):
        pid = os.getpid()
        return 1, str(pid), pid
    
    ## @param self
    ## @return int, str, [str]: code, msg, list of node names
    def getNodeNames(self):
        if self.pm is None:
            return 0, "uninitialized", []
        return 1, "node names", self.pm.get_active_names()

## XML-RPC server node
class ROSLaunchNode(xmlrpc.XmlRpcNode):
    
    ## Node constructor
    ## @param self
    ## @param handler xmlrpc.XmlRpcHandler: XML-RPC API handler for node. 
    def __init__(self, handler):
        super(ROSLaunchNode, self).__init__(0, handler)

    def run(self):
        try:
            super(ROSLaunchNode, self).run()
        except:
            logging.getLogger("roslaunch.remote").error(traceback.format_exc())
            print >> sys.stderr, "ERROR: failed to launch XML-RPC server for roslaunch"
    
_STARTUP_TIMEOUT = 5.0 #seconds
_CHILD_REGISTER_TIMEOUT = 10.0 #seconds

## Manages to running of remote roslaunch children
class ROSRemoteRunner(object):
    
    ## @param config ROSConfig launch configuration
    ## @param pm process monitor
    def __init__(self, rosconfig, pm):
        self.rosconfig = rosconfig
        self.server_node = None
        self.pm = pm
        # remote_processes { remote-name : RemoteRoslaunchProcess}.
        # NOTE: remote-name is not the same as the machine config key.
        self.remote_processes = {}
        self.logger = logging.getLogger('roslaunch.remote')
        self.listeners = []

    ## Listen to events about remote processes dying. Not
    ## threadsafe. Must be called before processes started.
    ## @param l ProcessListener
    def add_process_listener(self, l):
        self.listeners.append(l)

    ## prepare remote launch. will start XML-RPC services locally and remotely
    ## @param self
    def setup(self):
        self._start_server()
        self._start_children()
        
    ## Startup roslaunch server XML-RPC services
    ## @return str: XML-RPC URI
    def _start_server(self):
        self.logger.info("starting roslaunch XML-RPC server")
        server_node = ROSLaunchNode(ROSLaunchServerHandler(self.remote_processes, self.listeners))
        server_node.start()
        # wait for server_node to initialize
        timeout_t = time.time() + _STARTUP_TIMEOUT
        self.logger.info("waiting for roslaunch XML-RPC server to initialize")        
        while not server_node.uri and time.time() < timeout_t:
            time.sleep(0.01)
        if not server_node.uri:
            raise RLException("XML-RPC initialization failed")
        print "started roslaunch XML-RPC server %s"%server_node.uri
        self.logger.info("started roslaunch XML-RPC server %s",server_node.uri)
        self.server_node = server_node
    
    ## start the child roslaunch processes
    ## @param self
    def _start_children(self):
        server_node_uri = self.server_node.uri
        if not server_node_uri:
            raise RLException("server URI is not initialized")
        
        machines = {}
        for n in self.rosconfig.nodes:
            if not is_machine_local(n.machine):
                machines[n.machine.config_key()] = n.machine

        #print "\nRemote roslaunch children:"
        #print '\n'.join([" * %s"%m for m in machines])
        #print ''
        
        counter = 0
        for m in machines:
            machine = machines[m]
            # generate a name for the machine. don't use config key as
            # it's too long to easily display
            name = "%s-%s"%(machine.address, counter)
            counter += 1
            self.logger.info("remote[%s] starting roslaunch", name)
            print "remote[%s] starting roslaunch"%name
            
            env_dict = setup_env(None, machine, self.rosconfig.master.uri)
            p = RemoteRoslaunchProcess(name, server_node_uri, env_dict, machine)
            success = p.start()
            self.pm.register(p)
            if not success: #treat as fatal
                raise RLException("unable to start remote roslaunch child: %s"%m)
            self.remote_processes[name] = p

        # wait for all children to call register() callback
        timeout_t = time.time() + _CHILD_REGISTER_TIMEOUT

        while True:
            pending = []
            for k,v in self.remote_processes.iteritems():
                if not v.is_alive():
                    raise RLException("remote roslaunch failed to launch: %s"%k)
                elif not v.uri:
                    pending.append(k)
            if time.time() > timeout_t or not pending:
                break
            time.sleep(0.1)
        if pending:
            raise RLException("The following roslaunch remote processes failed to register: %s"%', '.join(pending))

    ## Contact each child to launch remote nodes
    ## @param self
    def launch_remote_nodes(self):
        succeeded = []
        failed = []
        
        # initialize remote_nodes. we use the machine config key as
        # the key for the dictionary so that we can bin the nodes.
        self.remote_nodes = {}
        for p in self.remote_processes.itervalues():
            self.remote_nodes[p.machine.config_key()] = []
            
        # build list of nodes that will be launched by machine
        nodes = [x for x in self.rosconfig.nodes if not is_machine_local(x.machine)]
        for n in nodes:
            self.remote_nodes[n.machine.config_key()].append(n)
            
        for m in self.remote_processes:
            child = self.remote_processes[m]
            nodes = self.remote_nodes[child.machine.config_key()]
            body = '\n'.join([remote_node_xml(n) for n in nodes])
            xml = '<launch>\n%s</launch>'%body
            if 0:
                print xml
                
            api = child.getapi()
            # TODO: timeouts
            import socket
            try:
                code, msg, val = api.launch(xml)
                c_succ, c_fail = val
                succeeded.extend(c_succ)
                failed.extend(c_fail)
            except socket.gaierror, (errno, msg):
                # usually errno == -2. See #815. 
                child_host, _ = network.parse_http_host_and_port(child.uri)
                printerrlog("Unable to contact remote roslaunch at [%s]. This is most likely due to a network misconfiguration with host lookups. Please make sure that you can contact '%s' from this machine"%(child.uri, child_host))                

        return succeeded, failed
            
## convert node data structure to XML for remote launch
def remote_node_xml(node):
    respawn_str = test_name_str = name_str = cwd_str = ''

    if isinstance(node, Test):
        t = 'test'
        test_name_str = ' test-name="%s"'%node.test_name
    elif isinstance(node, Node):
        t = "node"
    else:
        raise RLException("unsupported node type: %s"%node)

    # serialize all node attributes except machine name
    if t != 'test':
        if node.respawn:
            respawn_str = ' respawn="true"'
        else:
            respawn_str = ' respawn="false"'
    if node.name:
        name_str = ' name="%s"'%node.name
    if node.cwd:
        cwd_str = ' cwd="%s"'%node.cwd

    xmlstr = '<%s%s pkg="%s" type="%s" ns="%s" args="%s" output="%s"%s%s%s>\n'%(\
        t, test_name_str, node.package, node.type, node.namespace, node.args, node.output, cwd_str, respawn_str, name_str)
        
    xmlstr += ''.join(['  <remap from="%s" to="%s" />\n'%tuple(r) for r in node.remap_args])
    xmlstr += ''.join(['  <env name="%s" value="%s" />\n'%tuple(e) for e in node.env_args])
    xmlstr += "</%s>"%t
    return xmlstr
    
## Simple listener that forwards ProcessListener events to a roslaunch server
class _ProcessListenerForwarder(roslaunch.pmon.ProcessListener):
    def __init__(self, server):
        self.server = server
    def process_died(self, process_name, exit_code):
        try:
            self.server.process_died(process_name, exit_code)
        except Exception, e:
            logger = logging.getLogger("roslaunch.remote")            
            logger.error(traceback.format_exc())

## Startup roslaunch remote client XML-RPC services. Blocks until shutdown
## @param name str: name of remote client
## @param server_uri str: XML-RPC URI of roslaunch server
## @return str: XML-RPC URI
def run_child(name, server_uri):
    logger = logging.getLogger("roslaunch.remote")
    logger.info("starting roslaunch child process [%s], server URI is [%s]", name, server_uri)
    # start process monitor and XML-RPC node
    pm = roslaunch.pmon.start_process_monitor()
    if pm is None:
        # this should only happen if a shutdown signal is received during startup
        logger.error("cannot startup remote child: unable to start process monitor.")
        return
    logger.debug("started process monitor")
    child_node = None
    try:
        try:
            handler = ROSLaunchClientHandler(name, server_uri, pm)
            child_node = ROSLaunchNode(handler)
            child_node.start()

            logger.debug("started XML RPC node")
    
            # wait for node to initialize
            timeout_t = time.time() + _STARTUP_TIMEOUT
            while not child_node.uri and time.time() < timeout_t:
                time.sleep(0.0001)
            if not child_node.uri:
                logger.error("failed to contact XML RPC node within time limit")
                raise RLException("XML-RPC initialization failed")

            logger.debug("XML-RPC node initialized")
            
            # register it with server
            server = xmlrpclib.ServerProxy(server_uri)
            code, msg, _ = server.register(name, child_node.uri)
            if code != 1:
                raise RLException("unable to register with roslaunch server: %s"%msg)

            logger.debug("child registered with server")

            # register printlog handler so messages are funneled to remote
            def serverlog(msg):
                server.log(name, Log.INFO, msg)
            def servererrlog(msg):
                server.log(name, Log.ERROR, msg)
            add_printlog_handler(serverlog)
            add_printerrlog_handler(servererrlog)            

            # register process listener to forward process death events to main server
            pm.add_process_listener(_ProcessListenerForwarder(server))

            # block until process monitor is shutdown
            pm.mainthread_spin()
        except:
            err = traceback.format_exc()
            print err
            logger.error(err)
    finally:
        if pm:
            pm.shutdown()
            pm.join()
        if child_node:
            child_node.shutdown('run_child complete')
