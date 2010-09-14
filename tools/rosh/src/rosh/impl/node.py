# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

from __future__ import with_statement

import rosnode
import rospy

from rosh.impl.exceptions import ROSHException
from rosh.impl.namespace import Namespace, Concept

class Node(Namespace):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance. 
        @type  config: L{NamespaceConfig}
        """
        super(Node, self).__init__(name, config)
        self._uri = None
        self._init_uri()

    def _list(self):
        """
        Override Namespace._list()
        """
        return rosnode.get_node_names(namespace=self._ns)

    def _kill(self):
        rosnode.kill_nodes([self._name])
    
    def _init_uri(self):
        if self._name and self._uri is None:
            try:
                self._uri = rosnode.get_api_uri(self._config.master.handle, self._name)
            except:
                pass
        
    def _show(self):
        """
        show() handler
        """
        show_graph(self._ns)
    
    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        if self._uri is None:
            self._init_uri()
        if self._uri is None:
            return self._ns
        else:
            return rosnode.get_node_info_description(self._name)+'\n'+rosnode.get_api_uri(self._config.master.handle, self._name)

    def __call__(self):
        """
        Ping the node.
        """
        return rosnode.rosnode_ping(self._name, max_count=1, verbose=False)
    
class Nodes(Concept):

    def __init__(self, ctx, lock):
        super(Nodes, self).__init__(ctx, lock, Node)

    def _show(self):
        show_graph('/')

def show_graph(ns):
    import subprocess
    cmd = ['rxgraph', '--nodens', ns]
    # TODO: check for error return code?
    subprocess.Popen(cmd, stderr=subprocess.PIPE)

