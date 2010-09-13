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

"""
ROSH library for running processes. 

This library is necessary as ROSH commands, like set_master, can
manipulate the environment in ways that would affect subprocesses.
"""

import os
from subprocess import Popen, PIPE

import roslib.rosenv

def run(config, cmd, stdout=True):
    """
    Run the specified command using the current ROS configuration.
    """
    env = os.environ.copy()
    env[roslib.rosenv.ROS_MASTER_URI] = config.master.master_uri
    
    #TODO:remove
    if 1:
        print "CMD", cmd

    # TODO: added Exec objects to roslaunch that are only accessible via scriptapi
    if stdout:
        p = Popen(cmd, stderr=PIPE, env=env)
    else:
        p = Popen(cmd, stdout=PIPE, stderr=PIPE, env=env)
    
    p.poll()
    return p.returncode in [None, 0]

def _init_ctx(ctx):
    """
    Add shared roslaunch instance to ctx if not already initialized.
    
    @param ctx: ROSH context
    @type  ctx: Context
    """
    import roslaunch
    if not hasattr(ctx, 'roslaunch'):
        ctx.roslaunch = roslaunch.ROSLaunch()
        ctx.roslaunch.start()
    
def launch(ctx, pkg, type_, args, remap={}, stdout=False):
    """
    @param ctx: ROSH context
    @type  ctx: Context
    @return: Node instance
    @rtype: Node
    """
    import roslaunch
    output = 'screen' if stdout else None
    if type(args) in (list, tuple):
        args = ' '.join(args)
    if remap:
        args.append(' '.join(['%s:=%s'%(k, v) for k, v in remap.iteritems()]))
    n = roslaunch.Node(pkg, type_, args=args, output=output, filename='<rosh>')
    _init_ctx(ctx)
    ctx.roslaunch.launch(n)
    return n

def launch_node(ctx, node):
    """
    Launch a Node instance
    
    @param ctx: ROSH context
    @type  ctx: Context
    @param node: node instance. The name attribute may be overwritten.
    @type  node: roslaunch.Node
    @return: Node instance
    @rtype: Node
    """
    _init_ctx(ctx)
    ctx.roslaunch.launch(node)
    return node
    
