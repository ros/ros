# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

import os
import time

import roslaunch.core

## resolve command-line \a args to roslaunch filenames
## @return [str]: resolved filenames
def resolve_launch_arguments(args):
    import roslib.packages
    # user can either specify:
    #  - filename + launch args
    #  - package + relative-filename + launch args
    if not args:
        return args
    resolved_args = None
    top = args[0]
    if os.path.isfile(top):
        resolved_args = [top] + args[1:]
    elif len(args) == 1:
        raise roslaunch.core.RLException("[%s] does not exist. please specify a package and launch file"%(top))
    else:
        try:
            resolved = roslib.packages.find_resource(top, args[1])
            if len(resolved) == 1:
                resolved = resolved[0]
            elif len(resolved) > 1:
                raise roslaunch.core.RLException("multiple files named [%s] in package [%s].\nPlease specify full path instead"%(args[1], top))
        except roslib.packages.InvalidROSPkgException, e:
            raise roslaunch.core.RLException("[%s] is not a package or launch file name"%top)
        if not resolved:
            raise roslaunch.core.RLException("cannot locate [%s] in package [%s]"%(args[1], top))
        else:
            resolved_args = [resolved] + args[2:]
    return resolved_args

## block until master detected
def _wait_for_master():
    import roslaunch.core
    m = roslaunch.core.Master() # get a handle to the default master
    is_running = m.is_running()
    if not is_running:
        roslaunch.core.printlog("roscore/master is not yet running, will wait for it to start")
    while not is_running:
        time.sleep(0.1)
        is_running = m.is_running()
    if is_running:
        roslaunch.core.printlog("master has started, initiating launch")
    else:
        raise RuntimeError("unknown error waiting for master to start")

## use echo (where available) to change the name of the terminal window
def change_terminal_name(args, is_core):
    import platform
    if platform.system() in ['Linux', 'Darwin', 'Unix']:
        try:
            if is_core:
                print '\033]2;roscore\007'
            else:
                print '\033]2;'+','.join(args)+'\007'                
        except: pass

## @param options_runid str: run_id value from command-line or None
## @param options_wait_for_master bool: the wait_for_master command
## option. If this is True, it means that we must retrieve the value
## from the parameter server and need to avoid any race conditions
## with the roscore being initialized.
def get_or_generate_uuid(options_runid, options_wait_for_master):
    import roslib.scriptutil
    # Three possible sources of the run_id:
    #
    #  - if we're a child process, we get it from options_runid
    #  - if there's already a roscore running, read from the param server
    #  - generate one if we're running the roscore
    if options_runid:
        return options_runid

    # #773: Generate a run_id to use if we launch a master
    # process.  If a master is already running, we'll get the
    # run_id from it instead
    param_server = roslib.scriptutil.get_param_server()
    val = None
    while val is None:
        try:
            code, msg, val = param_server.getParam('/roslaunch', '/run_id')
            if code == 1:
                return val
            else:
                raise RuntimeError("unknown error communicating with Parameter Server: %s"%msg)
        except:
            if not options_wait_for_master:
                val = roslaunch.core.generate_run_id()
    return val
    
