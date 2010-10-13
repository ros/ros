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

"""Internal use: contains the L{ROSNode} wrapper, which provides an unified interface for running an XMLRPC server for the rospy client library."""

import os
import logging
import traceback

import roslib.xmlrpc

import rospy.core

from rospy.names import _set_caller_id, get_namespace, get_caller_id, scoped_name

class ROSNode(roslib.xmlrpc.XmlRpcNode):
    """
    Base ROS node, aka a slave node. Enables basic functionality for
    the connection of topics with other nodes.  ROSNode is initialized
    when the uri field has a value.
    """

    def __init__(self, name, port=0, rpc_handler=None):
        """
        Node constructor
        @param name: name of this node
        @type  name: str
        @param port: port to use for starting XML-RPC API. Set to 0 or omit to bind to any available port.
        @type port: int
        @param rpc_handler: XML-RPC API handler for node. 
        @type  rpc_handler: roslib.xmlrpc.XmlRpcHandler
        """
        super(ROSNode, self).__init__(port, rpc_handler)
        self.name = scoped_name(get_namespace(), name) #use the de-contexted name
        #TODO: move out of ROSNode
        _set_caller_id(name) #update global name context

        logging.getLogger('rospy.msnode').info("Node initialized: callerId[%s] local name[%s] port[%s]", get_caller_id(), self.name, self.port)

    def set_uri(self, uri):
        """
        Override parent hook for URI initialization
        """
        super(ROSNode, self).set_uri(uri)
        rospy.core.set_node_uri(self.uri)
            
    def shutdown(self, reason):
        super(ROSNode, self).shutdown(reason)
        self.name = None

    def run(self):
        try:
            #wrap lower-level run in order to report errors and call ROS shutdown APIs
            rospy.core.add_shutdown_hook(self.shutdown)            
            super(ROSNode, self).run()
        except:
            try:
                logerr("ERROR: error running XML-RPC server: \n"+traceback.format_exc())
            except: pass
            rospy.core.signal_shutdown('error in XML-RPC server')
