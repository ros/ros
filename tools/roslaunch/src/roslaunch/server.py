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
XML-RPC servers for parent and children

Following typical XmlRpcNode code, code is divided into:

 a) Handlers: these actually define and execute the XML-RPC API
 b) Nodes: these run the XML-RPC server

In this code you'll find 'Parent' and 'Child' code. The parent node
is the original roslaunch process. The child nodes are the child
processes it launches in order to handle remote launching (or
execution as a different user).
"""

import logging
import os
import socket
import sys
import time
import traceback
import urlparse
import xmlrpclib

import rosgraph.network as network
import rosgraph.xmlrpc as xmlrpc

import roslaunch.config 
from roslaunch.pmon import ProcessListener, Process
import roslaunch.xmlloader

from roslaunch.launch import ROSLaunchRunner
from roslaunch.core import RLException, \
     add_printlog_handler, add_printerrlog_handler, printlog, printerrlog, printlog_bold

#For using Log message level constants
from rosgraph_msgs.msg import Log

# interface class so that we don't have circular deps
class ChildROSLaunchProcess(Process):
    """
    API for remote roslaunch processes
    """
    def __init__(self, name, args, env):
        super(ChildROSLaunchProcess, self).__init__('roslaunch', name, args, env, False)
        self.uri = None

    def set_uri(self, uri):
        self.uri = uri
    
class ROSLaunchBaseHandler(xmlrpc.XmlRpcHandler):
    """
    Common XML-RPC API for the roslaunch server and child node
    """
    def __init__(self, pm):
        self.pm = pm
        self.logger = logging.getLogger('roslaunch.server')
        if self.pm is None:
            raise RLException("cannot create xmlrpc handler: pm is not initialized")
    
    #TODO: kill process, restart (with optional prefix). list active, list dead. CPU usage

    def list_processes(self):
        """
        @return: code, msg, process list.
        Process list is two lists, where first list of
        active process names along with the number of times that
        process has been spawned. Second list contains dead process
        names and their spawn count.
        @rtype: int, str, [[(str, int),], [(str,int),]]
        """
        return 1, "processes on parent machine", self.pm.get_process_names_with_spawn_count()

    def process_info(self, process_name):
        """
        @return: dictionary of metadata about process. Keys vary by implementation
        @rtype: int, str, dict
        """
        p = self.pm.get_process(process_name)
        if p is None:
            return -1, "no process by that name", {}
        else:
            return 1, "process info", p.get_info()

    def get_pid(self):
        """
        @return: code, msg, pid
        @rtype: int, str, int
        """

        pid = os.getpid()
        return 1, str(pid), pid

    def get_node_names(self):
        """
        @return: code, msg, list of node names
        @rtype: int, str, [str]
        """
        if self.pm is None:
            return 0, "uninitialized", []
        return 1, "node names", self.pm.get_active_names()

    def _shutdown(self, reason):
        """
        xmlrpc.XmlRpcHandler API: inform handler of shutdown
        @param reason: human-readable shutdown reason
        @type  reason: str
        """
        return 1, '', 1


# Uses camel-case network-API naming conventions
class ROSLaunchParentHandler(ROSLaunchBaseHandler):
    """
    XML-RPC API for the roslaunch server node
    """
    
    def __init__(self, pm, child_processes, listeners):
        """
        @param child_processes: Map of remote processes so that server can update processes
        with information as children register. Handler will not modify
        keys.
        @type  child_processes: {name : ChildROSLaunchProcess}.
        @param listeners [ProcessListener]: list of
        listeners to notify when process_died events occur.
        """        
        super(ROSLaunchParentHandler, self).__init__(pm)
        self.child_processes = child_processes
        self.listeners = listeners

    def register(self, client, uri):
        """
        Registration callback from newly launched roslaunch clients
        @param client: name of client
        @type  client: str
        @param uri: XML-RPC URI of client    
        @type  uri: str
        @return: code, msg, ignore
        @rtype:  int, str, int
        """        
        if client not in self.child_processes:
            self.logger.error("Unknown child [%s] registered with server", client)
            return -1, "unknown child [%s]"%client, 0
        else:
            self.logger.info("child [%s] registered with server, uri[%s]", client, uri)
            self.child_processes[client].set_uri(uri)
        return 1, "registered", 1

    def list_children(self):
        """
        List the roslaunch child processes.
        @return int, str, [str]: code, msg, list of the roslaunch children URIS
        """        
        return 1, 'roslaunch children', [v.uri for v in self.child_processes.itervalues() if v.uri is not None]
            
    def process_died(self, process_name, exit_code):
        """
        Inform roslaunch server that a remote process has died
        @param process_name: name of process that died
        @type  process_name: str
        @param exit_code: exit code of remote process
        @type  exit_code: int
        @return: code, msg, ignore
        @rtype: int, str, int
        """        
        for l in self.listeners:
            try:
                l.process_died(process_name, exit_code)
            except:
                self.logger.error(traceback.format_exc())
        return 1, '', 0
        
    def log(self, client, level, message):
        """
        Report a log message to the server
        @param client: name of client
        @type  client: str
        @param level: log level (uses rosgraph_msgs.msg.Log levels)
        @type  level: int
        @param message: message to log
        @type  message: str
        """        
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

class ROSLaunchChildHandler(ROSLaunchBaseHandler):
    """
    XML-RPC API implementation for child roslaunches
    NOTE: the client handler runs a process monitor so that
    it can track processes across requests
    """    
    
    def __init__(self, run_id, name, server_uri, pm):
        """
        @param server_uri: XML-RPC URI of server
        @type  server_uri: str
        @param pm: process monitor to use
        @type  pm: L{ProcessMonitor}
        @raise RLException: If parameters are invalid
        """            
        super(ROSLaunchChildHandler, self).__init__(pm)        
        if server_uri is None:
            raise RLException("server_uri is not initialized")
        self.run_id = run_id
        
        # parse the URI to make sure it's valid
        _, urlport = network.parse_http_host_and_port(server_uri)
        if urlport <= 0:
            raise RLException("ERROR: roslaunch server URI is not a valid XML-RPC URI. Value is [%s]"%m.uri)

        self.name = name
        self.pm = pm
        self.server_uri = server_uri
        self.server = xmlrpclib.ServerProxy(server_uri)

    def _shutdown(self, reason):
        """
        xmlrpc.XmlRpcHandler API: inform handler of shutdown
        @param reason: human-readable shutdown reason
        @type  reason: str
        """
        if self.pm is not None:
            self.pm.shutdown()
            self.pm.join()
            self.pm = None
        
    def shutdown(self):
        """
        @return: code, msg, ignore
        @rtype:  int, str, int
        """
        self._shutdown("external call")
        return 1, "success", 1
    
    def _log(self, level, message):
        """
        log message to log file and roslaunch server
        @param level: log level
        @type  level: int
        @param message: message to log
        @type  message: str
        """
        try:
            if self.logger is not None:
                self.logger.debug(message)
            if self.server is not None:
                self.server.log(str(self.name), level, str(message))
        except:
            self.logger.error(traceback.format_exc())
    
    def launch(self, launch_xml):
        """
        Launch the roslaunch XML file. Because this is a child
        roslaunch, it will not set parameters nor manipulate the
        master. Call blocks until launch is complete
        @param xml: roslaunch XML file to launch
        @type  xml: str
        @return: code, msg, [ [ successful launches], [failed launches] ]
        @rtype:  int, str, [ [str], [str] ] 
        """
        if self.pm is None:
            return 0, "uninitialized", -1

        rosconfig = roslaunch.config.ROSLaunchConfig()
        try:
            roslaunch.xmlloader.XmlLoader().load_string(launch_xml, rosconfig)
        except roslaunch.xmlloader.XmlParseException, e:
            return -1, "ERROR: %s"%e, [[], []]
        
        # won't actually do anything other than local, but still required
        rosconfig.assign_machines()

        try:
            # roslaunch clients try to behave like normal roslaunches as much as possible. It's
            # mainly the responsibility of the roslaunch server to not give us any XML that might
            # cause conflict (e.g. master tags, param tags, etc...).
            self._log(Log.INFO, "launching nodes...")
            runner = ROSLaunchRunner(self.run_id, rosconfig, server_uri=self.server_uri, pmon=self.pm)
            succeeded, failed = runner.launch()
            self._log(Log.INFO, "... done launching nodes")
            # enable the process monitor to exit of all processes die
            self.pm.registrations_complete()
            return 1, "launched", [ succeeded, failed ]
        except Exception as e:
            return 0, "ERROR: %s"%traceback.format_exc(), [[], []]
    
_STARTUP_TIMEOUT = 5.0 #seconds

class ROSLaunchNode(xmlrpc.XmlRpcNode):
    """
    Base XML-RPC server for roslaunch parent/child processes
    """
    
    def __init__(self, handler):
        """
        @param handler: xmlrpc api handler
        @type  handler: L{ROSLaunchBaseHandler}
        """
        super(ROSLaunchNode, self).__init__(0, handler)

    def start(self):
        """
        Startup roslaunch server XML-RPC services
        @raise RLException: if server fails to start
        """
        logger = logging.getLogger('roslaunch.server')
        logger.info("starting roslaunch XML-RPC server")
        super(ROSLaunchNode, self).start()
        
        # wait for node thread to initialize
        timeout_t = time.time() + _STARTUP_TIMEOUT
        logger.info("waiting for roslaunch XML-RPC server to initialize")        
        while not self.uri and time.time() < timeout_t:
            time.sleep(0.01)
        if not self.uri:
            raise RLException("XML-RPC initialization failed")
        
        # Make sure our xmlrpc server is actually up. We've seen very
        # odd cases where remote nodes are unable to contact the
        # server but have been unable to prove this is the cause.
        server_up = False
        while not server_up and time.time() < timeout_t:
            try:
                code, msg, val = xmlrpclib.ServerProxy(self.uri).get_pid()
                if val != os.getpid():
                    raise RLException("Server at [%s] did not respond with correct PID. There appears to be something wrong with the networking configuration"%self.uri)
                server_up = True
            except IOError:
                # presumably this can occur if we call in a small time
                # interval between the server socket port being
                # assigned and the XMLRPC server initializing, but it
                # is highly unlikely and unconfirmed
                time.sleep(0.1)
            except socket.error, (errno, msg):
                if errno == 113:
                    p = urlparse.urlparse(self.uri)
                    raise RLException("Unable to contact the address [%s], which should be local.\nThis is generally caused by:\n * bad local network configuration\n * bad ROS_IP environment variable\n * bad ROS_HOSTNAME environment variable\nCan you ping %s?"%(self.uri, p.hostname))
                else:
                    time.sleep(0.1)                    
        if not server_up:
            p = urlparse.urlparse(self.uri)
            raise RLException("""Unable to contact my own server at [%s].
This usually means that the network is not configured properly.

A common cause is that the machine cannot ping itself.  Please check
for errors by running:

\tping %s

For more tips, please see

\thttp://www.ros.org/wiki/ROS/NetworkSetup
"""%(self.uri, p.hostname))
        printlog_bold("started roslaunch server %s"%(self.uri))

    def run(self):
        """
        run() should not be called by higher-level code. ROSLaunchNode
        overrides underlying xmlrpc.XmlRpcNode implementation in order
        to log errors.
        """        
        try:
            super(ROSLaunchNode, self).run()
        except:
            logging.getLogger("roslaunch.remote").error(traceback.format_exc())
            print >> sys.stderr, "ERROR: failed to launch XML-RPC server for roslaunch"

class ROSLaunchParentNode(ROSLaunchNode):
    """
    XML-RPC server for parent roslaunch.
    """
    
    def __init__(self, rosconfig, pm):
        """
        @param config: ROSConfig launch configuration
        @type  config: L{ROSConfig}
        @param pm: process monitor
        @type  pm: L{ProcessMonitor}
        """
        self.rosconfig = rosconfig
        self.listeners = []
        self.child_processes = {} #{ child-name : ChildROSLaunchProcess}.

        if pm is None:
            raise RLException("cannot create parent node: pm is not initialized")
        handler = ROSLaunchParentHandler(pm, self.child_processes, self.listeners)
        super(ROSLaunchParentNode, self).__init__(handler)
        
    def add_child(self, name, p):
        """
        @param name: child roslaunch's name. NOTE: \a name is not
            the same as the machine config key.
        @type  name: str
        @param p: process handle of child
        @type  p: L{Process}
        """        
        self.child_processes[name] = p

    def add_process_listener(self, l):
        """
        Listen to events about remote processes dying. Not
        threadsafe. Must be called before processes started.
        @param l: Process listener
        @type  l: L{ProcessListener}
        """
        self.listeners.append(l)

class _ProcessListenerForwarder(ProcessListener):
    """
    Simple listener that forwards ProcessListener events to a roslaunch server
    """
    def __init__(self, server):
        self.server = server
    def process_died(self, process_name, exit_code):
        try:
            self.server.process_died(process_name, exit_code)
        except Exception as e:
            logging.getLogger("roslaunch.remote").error(traceback.format_exc())

class ROSLaunchChildNode(ROSLaunchNode):
    """
    XML-RPC server for roslaunch child processes
    """    

    def __init__(self, run_id, name, server_uri, pm):
        """
    ## Startup roslaunch remote client XML-RPC services. Blocks until shutdown
    ## @param name: name of remote client
    ## @type  name: str
    ## @param server_uri: XML-RPC URI of roslaunch server
    ## @type  server_uri: str
    ## @return: XML-RPC URI
    ## @rtype: str
        """        
        self.logger = logging.getLogger("roslaunch.server")
        self.run_id = run_id
        self.name = name
        self.server_uri = server_uri
        self.pm = pm
        
        if self.pm is None:
            raise RLException("cannot create child node: pm is not initialized")
        handler = ROSLaunchChildHandler(self.run_id, self.name, self.server_uri, self.pm)
        super(ROSLaunchChildNode, self).__init__(handler)

    def _register_with_server(self):
        """
        Register child node with server
        """        
        name = self.name
        self.logger.info("attempting to register with roslaunch parent [%s]"%self.server_uri)
        try:
            server = xmlrpclib.ServerProxy(self.server_uri)
            code, msg, _ = server.register(name, self.uri)
            if code != 1:
                raise RLException("unable to register with roslaunch server: %s"%msg)
        except Exception as e:
            self.logger.error("Exception while registering with roslaunch parent [%s]: %s"%(self.server_uri, traceback.format_exc(e)))
            # fail
            raise RLException("Exception while registering with roslaunch parent [%s]: %s"%(self.server_uri, traceback.format_exc(e)))
        
        self.logger.debug("child registered with server")
        
        # register printlog handler so messages are funneled to remote
        def serverlog(msg):
            server.log(name, Log.INFO, msg)
        def servererrlog(msg):
            server.log(name, Log.ERROR, msg)
        add_printlog_handler(serverlog)
        add_printerrlog_handler(servererrlog)            

        # register process listener to forward process death events to main server
        self.pm.add_process_listener(_ProcessListenerForwarder(server))

    def start(self):
        """
        Initialize child. Must be called before run
        """        
        self.logger.info("starting roslaunch child process [%s], server URI is [%s]", self.name, self.server_uri)
        super(ROSLaunchChildNode, self).start()
        self._register_with_server()
