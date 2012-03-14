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
Scripting interface for roslaunch
"""

from roslaunch.core import Node, Master, RLException

import roslaunch.config
import roslaunch.parent
import roslaunch.xmlloader 

class ROSLaunch(object):
    """
    ROSLaunchParent represents the main 'parent' roslaunch process. It
    is responsible for loading the launch files, assigning machines,
    and then starting up any remote processes. The __main__ method
    delegates most of runtime to ROSLaunchParent.

    This must be called from the Python Main thread due to signal registration.    
    """

    def __init__(self):
        """
        @raise RLException: if fails to initialize
        """
        import rosgraph.masterapi
        master = rosgraph.masterapi.Master('/roslaunch_script')
        uuid = master.getParam('/run_id')
        self.parent = roslaunch.parent.ROSLaunchParent(uuid, [], is_core=False)
        self.started = False

    def load(self, f):
        """
        Load roslaunch file
        
        @param f: filename
        @type  f: str
        """
        raise NotImplemented

    def load_str(self, s):
        """
        Load roslaunch string
        
        @param s: string representation of roslaunch config
        @type  s: str
        """
        raise NotImplemented
        
    def launch(self, node):
        """
        Launch a roslaunch node instance
        
        @param node: roslaunch Node instance
        @type  node: roslaunch.Node
        @return: node process
        @rtype: roslaunch.Process
        @raise RLException: if launch fails
        """
        if not self.started:
            raise RLException("please start ROSLaunch first")
        elif not isinstance(node, Node):
            raise ValueError("arg must be of type Node")

        proc, success = self.parent.runner.launch_node(node)
        if not success:
            raise RLException("failed to launch %s/%s"%(node.package, node.type))
        return proc

    def start(self):
        """
        Start roslaunch. This will launch any pre-configured launches and spin up the process monitor thread.
        """
        self.parent.start(auto_terminate=False)
        self.started = True
        
    def spin(self):
        self.parent.spin()

    def spin_once(self):
        self.parent.spin_once()        
        
    def stop(self):
        self.parent.shutdown()

