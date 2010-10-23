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
Internal use: rospy initialization.

This is mainly routines for initializing the master or slave based on
the OS environment.
"""

import os
import sys
import logging
import time
import traceback

import roslib.rosenv
import roslib.xmlrpc

from ..names import _set_caller_id
from ..core import is_shutdown, add_log_handler, signal_shutdown, rospyerr

from .tcpros import init_tcpros
from .masterslave import ROSHandler

DEFAULT_NODE_PORT = 0 #bind to any open port
DEFAULT_MASTER_PORT=11311 #default port for master's to bind to
DEFAULT_MASTER_URI = 'http://localhost:%s/'%DEFAULT_MASTER_PORT

###################################################
# rospy module lower-level initialization

def _node_run_error(e):
    """
    If XML-RPC errors out of the run() method, this handler is invoked
    """
    rospyerr(traceback.format_exc())
    signal_shutdown('error in XML-RPC server: %s'%(e))

def start_node(environ, resolved_name, master_uri=None, port=None):
    """
    Load ROS slave node, initialize from environment variables
    @param environ: environment variables
    @type  environ: dict
    @param resolved_name: resolved node name
    @type  resolved_name: str
    @param master_uri: override ROS_MASTER_URI: XMlRPC URI of central ROS server
    @type  master_uri: str
    @param port: override ROS_PORT: port of slave xml-rpc node
    @type  port: int
    @return: node server instance
    @rtype roslib.xmlrpc.XmlRpcNode
    @raise ROSInitException: if node has already been started
    """
    init_tcpros()
    if not master_uri:
        master_uri = roslib.rosenv.get_master_uri()
    if not master_uri:
        master_uri = DEFAULT_MASTER_URI

    # this will go away in future versions of API
    _set_caller_id(resolved_name) 

    handler = ROSHandler(resolved_name, master_uri)
    node = roslib.xmlrpc.XmlRpcNode(port, handler, on_run_error=_node_run_error)
    node.start()
    while not node.uri and not is_shutdown():
        time.sleep(0.00001) #poll for XMLRPC init
    logging.getLogger("rospy.init").info("ROS Slave URI: [%s]", node.uri)

    while not handler._is_registered() and not is_shutdown():
        time.sleep(0.1) #poll for master registration
    logging.getLogger("rospy.init").info("registered with master")
    return node

def _stdout_handler(level):
    def fn(msg):
        sys.stdout.write("[%s] %f: %s\n"%(level,time.time(), str(msg)))
    return fn
def _log_stream_handler(stream, level):
    def fn(msg):
        stream.write("[%s] %f: %s\n"%(level,time.time(), str(msg)))
    return fn

_loggers_initialized = False
def init_log_handlers():
    global _loggers_initialized
    if _loggers_initialized:
        return

    from roslib.msg import Log
    
    # client logger
    clogger = logging.getLogger("rosout")
    add_log_handler(Log.DEBUG, clogger.debug)
    add_log_handler(Log.INFO, clogger.info)
    add_log_handler(Log.ERROR, clogger.error)
    add_log_handler(Log.WARN, clogger.warn)
    add_log_handler(Log.FATAL, clogger.critical)

    # TODO: make this configurable
    # INFO -> stdout
    # ERROR, FATAL -> stderr
    add_log_handler(Log.INFO, _log_stream_handler(sys.stdout, 'INFO'))
    add_log_handler(Log.WARN, _log_stream_handler(sys.stderr, 'WARN'))
    add_log_handler(Log.ERROR, _log_stream_handler(sys.stderr, 'ERROR'))
    add_log_handler(Log.FATAL, _log_stream_handler(sys.stderr, 'FATAL'))

