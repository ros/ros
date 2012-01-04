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
ROS Master. 

This module integrates the lower-level implementation modules into a
single interface for running and stopping the ROS Master.
"""

import logging
import time

import rosgraph.xmlrpc

import rosmaster.master_api

DEFAULT_MASTER_PORT=11311 #default port for master's to bind to

class Master(object):
    
    def __init__(self, port=DEFAULT_MASTER_PORT):
        self.port = port
        
    def start(self):
        """
        Start the ROS Master.
        """
        self.handler = None
        self.master_node = None
        self.uri = None

        handler = rosmaster.master_api.ROSMasterHandler()
        master_node = rosgraph.xmlrpc.XmlRpcNode(self.port, handler)
        master_node.start()

        # poll for initialization
        while not master_node.uri:
            time.sleep(0.0001) 

        # save fields
        self.handler = handler
        self.master_node = master_node
        self.uri = master_node.uri
        
        logging.getLogger('rosmaster.master').info("Master initialized: port[%s], uri[%s]", self.port, self.uri)

    def ok(self):
        if self.master_node is not None:
            return self.master_node.handler._ok()
        else:
            return False
    
    def stop(self):
        if self.master_node is not None:
            self.master_node.shutdown('Master.stop')
            self.master_node = None
