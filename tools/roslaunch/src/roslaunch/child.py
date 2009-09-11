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

import roslaunch.core 
import roslaunch.pmon
import roslaunch.server

## ROSLaunchChild infrastructure
class ROSLaunchChild(object):

    ## Startup roslaunch remote client XML-RPC services. Blocks until shutdown
    ## @param run_id str: UUID of roslaunch session
    ## @param name str: name of remote client
    ## @param server_uri str: XML-RPC URI of roslaunch server
    ## @return str: XML-RPC URI
    def __init__(self, run_id, name, server_uri):
        roslaunch.core.set_child_mode(True)
        
        self.logger = logging.getLogger("roslaunch.child")
        self.run_id = run_id
        self.name = name
        self.server_uri = server_uri
        self.child_server = None
        self.pm = None

    ## start process monitor for child roslaunch
    ## @param self
    def _start_pm(self):
        # start process monitor
        #  - this test is mainly here so that testing logic can
        #    override process monitor with a mock
        if self.pm is None:
            self.pm = roslaunch.pmon.start_process_monitor()
        if self.pm is None:
            # this should only happen if a shutdown signal is received during startup
            raise roslaunch.core.RLException("cannot startup remote child: unable to start process monitor.")
        self.logger.debug("started process monitor")
        
    ## Run's child. Blocks until child processes exit.
    def run(self):
        try:
            try:
                self.logger.info("starting roslaunch child process [%s], server URI is [%s]", self.name, self.server_uri)
                self._start_pm()
                self.child_server = roslaunch.server.ROSLaunchChildNode(self.run_id, self.name, self.server_uri, self.pm)
                self.logger.info("... creating XMLRPC server for child")
                self.child_server.start()
                self.logger.info("... started XMLRPC server for child")
                # block until process monitor is shutdown
                self.pm.mainthread_spin()
                self.logger.info("... process monitor is done spinning")
            except:
                self.logger.error(traceback.format_exc())
                raise
        finally:
            if self.pm:
                self.pm.shutdown()
                self.pm.join()
            if self.child_server:
                self.child_server.shutdown('roslaunch child complete')

    def shutdown(self):
        if self.pm:
            self.pm.shutdown()
            self.pm.join()
