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
import itertools
import socket
import stat
import sys
import xmlrpclib

from os.path import isfile, isdir

import roslib.packages
import roslaunch
import roslaunch.netapi

from roswtf.environment import paths, is_executable
from roswtf.rules import warning_rule, error_rule

def bin_roslaunch_check(ctx):
    roslaunch = os.path.join(ctx.ros_root, 'bin', 'roslaunch')
    if not isfile(roslaunch):
        return "%(ros_root)s/bin is missing roslaunch"
    if not is_executable(roslaunch):
        return "%s is lacking executable permissions"%roslaunch

def _find_node(pkg, node_type):
    try:
        dir = roslib.packages.get_pkg_dir(pkg)
    except roslib.packages.InvalidROSPkgException:
        # caught by another rule
        return []
    paths = []
    #UNIXONLY
    node_exe = None
    for p, dirs, files in os.walk(dir):
        if node_type in files:
            test_path = os.path.join(p, node_type)
            s = os.stat(test_path)
            if (s.st_mode & stat.S_IRWXU == stat.S_IRWXU):
                paths.append(test_path)
        if '.svn' in dirs:
            dirs.remove('.svn')
        elif '.git' in dirs:
            dirs.remove('.git')
    return paths

## check if node is cannot be located in package
def roslaunch_missing_node_check(ctx):
    nodes = []
    for filename, rldeps in ctx.launch_file_deps.iteritems():
        nodes.extend(rldeps.nodes)
    errors = []
    for pkg, node_type in nodes:
        paths = _find_node(pkg, node_type)
        if not paths:
            errors.append("node [%s] in package [%s]"%(node_type, pkg))
    return errors
    
## check if two nodes with same name in package
def roslaunch_duplicate_node_check(ctx):
    nodes = []
    for filename, rldeps in ctx.launch_file_deps.iteritems():
        nodes.extend(rldeps.nodes)
    warnings = []
    for pkg, node_type in nodes:
        paths = _find_node(pkg, node_type)
        if len(paths) > 1:
            warnings.append("node [%s] in package [%s]\n"%(node_type, pkg))
    return warnings

def roslaunch_machine_credentials_check(ctx):
    """
    Do basic SSH checks for machine, this does not do the full NxN test
    """
    nodes = []
    for filename, rldeps in ctx.launch_file_deps.iteritems():
        nodes.extend(rldeps.nodes)
    warnings = []
    for pkg, node_type in nodes:
        paths = _find_node(pkg, node_type)
        if len(paths) > 1:
            warnings.append("node [%s] in package [%s]\n"%(node_type, pkg))
    return warnings

def pycrypto_check(ctx):
    try:
        import Crypto
    except ImportError, e:
        return True

def paramiko_check(ctx):
    try:
        import paramiko
    except ImportError, e:
        return True
def paramiko_system_keys(ctx):
    try:
        import paramiko
        ssh = paramiko.SSHClient()
        try:
            ssh.load_system_host_keys() #default location
        except:
            return True
    except: pass

def paramiko_ssh(ctx, address, port, username, password):
    try:
        import paramiko
        ssh = paramiko.SSHClient()

        import roslaunch.remoteprocess
        err_msg = roslaunch.remoteprocess.ssh_check_known_hosts(ssh, address, port, username=username)
        if err_msg:
            return err_msg
        
        if not password: #use SSH agent
            ssh.connect(address, port, username)
        else: #use SSH with login/pass
            ssh.connect(address, port, username, password)

    except paramiko.BadHostKeyException:
        return  "Unable to verify host key for [%s:%s]"%(address, port)
    except paramiko.AuthenticationException:
        return "Authentication to [%s:%s] failed"%(address, port)
    except paramiko.SSHException, e:
        return "[%s:%s]: %s"%(address, port, e)
    except ImportError:
        pass

def _load_roslaunch_config(ctx):
    config = roslaunch.ROSLaunchConfig()
    loader = roslaunch.XmlLoader()
    # TODO load roscore
    for launch_file in ctx.launch_files:
        loader.load(launch_file, config, verbose=False)
    try:
        config.validate()
        config.assign_machines()
    except roslaunch.RLException, e:
        return config, []
    machines = []
    for n in itertools.chain(config.nodes, config.tests):
        if n.machine not in machines:
            machines.append(n.machine)
    return config, machines

def roslaunch_load_check(ctx):
    config = roslaunch.ROSLaunchConfig()
    loader = roslaunch.XmlLoader()
    # TODO load roscore
    for launch_file in ctx.launch_files:
        loader.load(launch_file, config, verbose=False)
    try:
        config.validate()
        config.assign_machines()
    except roslaunch.RLException, e:
        return str(e)
    
def roslaunch_machine_name_check(ctx):
    config, machines = _load_roslaunch_config(ctx)
    bad = []
    for m in machines:
        try:
            socket.gethostbyname(m.address)
        except socket.gaierror:
            bad.append(m.address)
    return ''.join([' * %s\n'%b for b in bad])

def roslaunch_ssh_check(ctx):
    import roslaunch.core
    if not ctx.launch_files:
        return # not relevant
    config, machines = _load_roslaunch_config(ctx)
    err_msgs = []
    for m in machines:
        socket.setdefaulttimeout(3.)
        # only check if the machine requires ssh to connect
        if not roslaunch.core.is_machine_local(m):
            err_msg = paramiko_ssh(ctx, m.address, m.ssh_port, m.user, m.password)
            if err_msg:
                err_msgs.append(err_msg)
    return err_msgs

def roslaunch_missing_pkgs_check(ctx):
    # rospack depends does not return depends that it cannot find, so
    # we have to manually determine this
    config, machines = _load_roslaunch_config(ctx)
    missing = []
    for n in config.nodes:
        pkg = n.package
        try:
            roslib.packages.get_pkg_dir(pkg, required=True)
        except:
            missing.append(pkg)
    return missing

def roslaunch_config_errors(ctx):
    config, machines = _load_roslaunch_config(ctx)
    return config.config_errors

def roslaunch_missing_deps_check(ctx):
    missing = []
    for pkg, miss in ctx.launch_file_missing_deps.iteritems():
        if miss:
            missing.append("%s/manifest.xml: %s"%(pkg, ', '.join(miss)))
    return missing

def roslaunch_respawn_check(ctx):
    respawn = []
    for uri in ctx.roslaunch_uris:
        try:
            r = xmlrpclib.ServerProxy(uri)
            code, msg, val = r.list_processes()
            active, _ = val
            respawn.extend([a for a in active if a[1] > 1])
            #TODO: children processes
            #code, msg, val = r.list_children()
        except:
            pass # error for another rule
    return ["%s (%s)"%(a[0], a[1]) for a in respawn]

def roslaunch_uris_check(ctx):
    # check for any roslaunch processes that cannot be contacted
    bad = []
    # uris only contains the parent launches
    for uri in ctx.roslaunch_uris:
        try:
            r = xmlrpclib.ServerProxy(uri)
            code, msg, val = r.list_children()
            # check the children launches
            if code == 1:
                for child_uri in val:
                    try:
                        r = xmlrpclib.ServerProxy(uri)
                        code, msg, val = r.get_pid()
                    except:
                        bad.append(child_uri)
        except:
            bad.append(uri)
    return bad

def roslaunch_dead_check(ctx):
    dead = []
    for uri in ctx.roslaunch_uris:
        try:
            r = xmlrpclib.ServerProxy(uri)
            code, msg, val = r.list_processes()
            _, dead_list = val
            dead.extend([d[0] for d in dead_list])
            #TODO: children processes
            #code, msg, val = r.list_children()
        except:
            pass # error for another rule
    return dead

online_roslaunch_warnings = [
    (roslaunch_respawn_check,"These nodes have respawned at least once:"),
    (roslaunch_dead_check,"These nodes have died:"),
    # disabling for now as roslaunches don't do cleanup
    #(roslaunch_uris_check,"These roslaunch processes can no longer be contacted and may have exited:"),    
    ]

online_roslaunch_errors = [
    (roslaunch_ssh_check,"SSH failures:"),
    ]

static_roslaunch_warnings = [
    (roslaunch_duplicate_node_check, "Multiple nodes of same name in packages:"),
    (pycrypto_check, "pycrypto is not installed"),
    (paramiko_check, "paramiko is not installed"),
    (paramiko_system_keys, "cannot load SSH host keys -- your known_hosts file may be corrupt") ,
    (roslaunch_config_errors, "Loading your launch files reported the following configuration errors:"),
    ]
static_roslaunch_errors = [
    (bin_roslaunch_check, "roslaunch executable is invalid:"),
    (roslaunch_missing_deps_check, 
     "Package %(pkg)s is missing roslaunch dependencies.\nPlease add the following tags to %(pkg)s/manifest.xml:"),
    (roslaunch_missing_pkgs_check, 
     "Cannot find the following required packages:"),
    (roslaunch_missing_node_check, "Several nodes in your launch file could not be located. These are either typed incorrectly or need to be built:"),
    (roslaunch_machine_name_check,"Cannot resolve the following hostnames:"),
    (roslaunch_load_check, "roslaunch load failed"),
    ]

def wtf_check_static(ctx):
    if not ctx.launch_files:
        return

    #NOTE: roslaunch files are already loaded separately into context

    #TODO: check each machine name
    #TODO: bidirectional ping for each machine

    for r in static_roslaunch_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in static_roslaunch_errors:
        error_rule(r, r[0](ctx), ctx)
        
def _load_online_ctx(ctx):
    ctx.roslaunch_uris = roslaunch.netapi.get_roslaunch_uris()
    
def wtf_check_online(ctx):
    _load_online_ctx(ctx)
    for r in online_roslaunch_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in online_roslaunch_errors:
        error_rule(r, r[0](ctx), ctx)
