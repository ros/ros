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
import logging
import time

import roslib.rosenv

import rospy.core 
import rospy.msproxy 
import rospy.names

import rospy.impl.tcpros 
import rospy.impl.msnode 
import rospy.impl.masterslave 

DEFAULT_NODE_PORT = 0 #bind to any open port
DEFAULT_MASTER_PORT=11311 #default port for master's to bind to

_node = None #global var for ros init and easy interpreter access 
def get_node_proxy():
    """
    Retrieve L{NodeProxy} for slave node running on this machine.

    @return: slave node API handle
    @rtype: L{rospy.NodeProxy}
    """
    return _node

###################################################
# rospy module lower-level initialization

def default_master_uri():
    """
    @return: URI of master that will be used if master is not otherwise configured.
    @rtype: str
    """
    return 'http://localhost:%s/'%DEFAULT_MASTER_PORT

def _sub_start_node(environ, resolved_name, master_uri=None, port=DEFAULT_NODE_PORT):
    """
    Subroutine for X{start_node()}
    """
    if not master_uri:
        master_uri = roslib.rosenv.get_master_uri()
    if not master_uri:
        master_uri = default_master_uri()

    handler = rospy.impl.masterslave.ROSHandler(resolved_name, master_uri)
    node = rospy.impl.msnode.ROSNode(resolved_name, port, handler)
    node.start()
    while not node.uri and not rospy.core.is_shutdown():
        time.sleep(0.00001) #poll for XMLRPC init
    logging.getLogger("rospy.init").info("ROS Slave URI: [%s]", node.uri)

    while not handler._is_registered() and not rospy.core.is_shutdown():
        time.sleep(0.1) #poll for master registration
    logging.getLogger("rospy.init").info("registered with master")

    return rospy.msproxy.NodeProxy(node.uri)

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
    @return: node proxy instance
    @rtype rospy.msproxy.NodeProxy
    """
    global _node
    rospy.impl.tcpros.init_tcpros()
    if _node is not None:
        raise Exception("Only one master/slave can be run per instance (multiple calls to start_node)")
    _node = _sub_start_node(environ, resolved_name, master_uri, port)
    return _node
    
