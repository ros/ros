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

if __name__ == '__main__':
    import roslib; roslib.load_manifest('roslaunch')
    
import os
import getpass
import logging
import socket
import string
import sys
import urlparse
import xmlrpclib

import roslib.names 
import roslib.network
import roslib.packages
import roslib.scriptutil 
import roslib.substitution_args
import roslib.rosenv

#TODO:temporary until xacro is ported after next ROS stable release
resolve_args = roslib.substitution_args.resolve_args

class RLException(Exception): pass


## Phases allow executables to be assigned to a particular run period
PHASE_SETUP    = 'setup'
PHASE_RUN      = 'run'
PHASE_TEARDOWN = 'teardown'

_child_mode = False
## @return True if roslaunch is running in remote child mode
def is_child_mode():
    return _child_mode
## @param child_mode bool: True if roslaunch is running in remote
## child mode
def set_child_mode(child_mode):
    global _child_mode
    _child_mode = child_mode

## Check to see if machine is local. NOTE: a machine is not local if
## its user credentials do not match the current user.
## @param machine Machine
## @return True if machine is local and doesn't require remote login
def is_machine_local(machine):
    try:
        machine_addr = socket.gethostbyname(machine.address)
    except socket.gaierror:
        raise RLException("cannot resolve host address for machine [%s]"%machine.address)
    local_addresses = ['localhost'] + roslib.network.get_local_addresses()
    # check 127/8 and local addresses
    is_local = machine_addr.startswith('127.') or machine_addr in local_addresses

    #491: override local to be ssh if machine.user != local user
    if is_local and machine.user:
        import getpass
        is_local = machine.user == getpass.getuser()
    return is_local
    
_printlog_handlers = []
_printerrlog_handlers = []
## core utility for printing message to stdout as well as printlog handlers
def printlog(msg):
    for h in _printlog_handlers:
        try: # don't let this bomb out the actual code
            h(msg)
        except:
            pass
    try: # don't let this bomb out the actual code        
        print msg
    except:
        pass

## similar to printlog, but the message printed to screen is bolded for greater clarity
def printlog_bold(msg):
    for h in _printlog_handlers:
        try: # don't let this bomb out the actual code
            h(msg)
        except:
            pass
    try: # don't let this bomb out the actual code        
        print '\033[1m%s\033[0m'%msg
    except:
        pass

## core utility for printing message to stderr as well as printerrlog handlers
def printerrlog(msg):
    for h in _printerrlog_handlers:
        try: # don't let this bomb out the actual code
            h(msg)
        except:
            pass
    # #1003: this has raised IOError (errno 104) in robot use. Better to
    # trap than let a debugging routine fault code.
    try: # don't let this bomb out the actual code
        print >> sys.stderr, '\033[31m%s\033[0m'%msg
    except:
        pass

## register additional handler for printlog()
def add_printlog_handler(h):
    _printlog_handlers.append(h)
## register additional handler for printerrlog()
def add_printerrlog_handler(h):
    _printerrlog_handlers.append(h)

## delete all printlog handlers. required for testing
def clear_printlog_handlers():
    del _printlog_handlers[:]
## delete all printerrlog handlers. required for testing
def clear_printerrlog_handlers():
    del _printerrlog_handlers[:]
    
## Create dictionary of environment variables to set for launched
## process.
## @param machine Machine: machine being launched on
## @param node Node: node that is being launched or None
## @param master_uri str: ROS master URI 
## @return dict: process env dictionary
def setup_env(node, machine, master_uri):
    d = {}
    d[roslib.rosenv.ROS_MASTER_URI] = master_uri
    d[roslib.rosenv.ROS_ROOT] = machine.ros_root or get_ros_root()
    if machine.ros_package_path: #optional
        d[roslib.rosenv.ROS_PACKAGE_PATH] = machine.ros_package_path
    if machine.ros_ip: #optional
        d[roslib.rosenv.ROS_IP] = machine.ros_ip

    # roslib now depends on PYTHONPATH being set.  This environment
    # needs to be setup correctly for roslaunch through ssh to work
    d['PYTHONPATH'] = os.path.join(d[roslib.rosenv.ROS_ROOT],'core','roslib', 'src')

    # load in machine env_args. Node env args have precedence
    for name, value in machine.env_args:
        d[name] = value

    # add node-specific env args last as they have highest precedence
    if node:
        ns = node.namespace 
        if ns[-1] == '/':
            ns = ns[:-1]
        if ns:
            d[roslib.rosenv.ROS_NAMESPACE] = ns 
        for name, value in node.env_args:
            d[name] = value

    return d

## wrap lower-level exceptions in RLException class
## @return fn: function wrapper that throws an RLException if the
## wrapped function throws an Exception
def rle_wrapper(fn):
    def wrapped_fn(*args):
        try:
            return fn(*args)
        except Exception, e:
            # we specifically catch RLExceptions and print their messages differently
            raise RLException("ERROR: %s"%e)
    return wrapped_fn
        
get_ros_root         = rle_wrapper(roslib.rosenv.get_ros_root)
get_master_uri_env   = rle_wrapper(roslib.rosenv.get_master_uri) 
def get_ros_package_path():
    # ROS_PACKAGE_PATH not required to be set
    return roslib.rosenv.get_ros_package_path(False)

## resolve localhost addresses to an IP address so that
## @param uri: XML-RPC URI
## @param force_localhost bool: if True, URI is mapped onto the local machine no matter what
def remap_localhost_uri(uri, force_localhost=False):
    hostname, port = roslib.network.parse_http_host_and_port(uri)
    if force_localhost or hostname == 'localhost':
        return roslib.network.create_local_xmlrpc_uri(port)
    else:
        return uri

## @param pkg: rospack package parameter
## @param tag: rospack export tag parameter
## @return str: result of executing rospack export/tag on pkg
#def rospack_export(pkg, tag):
#    if not '/' in tag:
#        raise RLException("$(export pkg tag/attribute) command must contain an attribute. Value was [%s]"%tag)
#    return roslib.scriptutil.rospackexec(['export/%s'%tag, pkg])

##################################################################
# DATA STRUCTURES

## Data structure for representing and quering state of master 
class Master:
    __slots__ = ['type', 'auto', 'uri', 'log_output']
    ## don't start a master
    AUTO_NO    = 0
    ## start a master if one isn't running
    AUTO_START   = 1
    ## start/restart master (i.e. restart an existing master)
    AUTO_RESTART = 2
    ZENMASTER = 'zenmaster'
    BOTHERDER = 'botherder'    

    ## ctor
    ## @param uri: master URI
    ## @param type_: 'zenmaster' or 'botherder'
    ## @param auto int: AUTO_NO | AUTO_START | AUTO_RESTART. AUTO_NO
    ##   is the default
    def __init__(self, type_=None, uri=None, auto=None):
        if auto is not None and type(auto) != int:
            raise RLException("invalid auto value: %s"%auto)            
        self.type = type_ or Master.ZENMASTER
        self.auto = auto or Master.AUTO_NO
        if self.auto not in [Master.AUTO_NO, Master.AUTO_START, Master.AUTO_RESTART]:
            raise RLException("invalid auto value: %s"%auto)
        self.uri  = remap_localhost_uri(uri or get_master_uri_env())
        # by default, master output goes to screen
        self.log_output = False

    def __eq__(self, m2):
        if not isinstance(m2, Master):
            return False
        else:
            return m2.auto == self.auto and m2.type == self.type and m2.uri == self.uri and m2.log_output == self.log_output

    ## @return ServerProxy: XMLRPC proxy for communicating with master
    def get(self):
        return xmlrpclib.ServerProxy(self.uri)
    ## override port specification of Master. This only has an effect on masters that have not
    ## been launched yet.
    def set_port(self, port):
        host, _ = roslib.network.parse_http_host_and_port(self.uri)
        self.uri = 'http://%s:%s/'%(host, port)
    
    ## @return bool: True if the master is running
    def is_running(self):
        try:
            try:
                to_orig = socket.getdefaulttimeout()
                # enable timeout 
                socket.setdefaulttimeout(5.0)
                logging.getLogger('roslaunch').info('master.is_running[%s]'%self.uri)
                code, status, val = self.get().getPid('/roslaunch')
                if code != 1:
                    raise RLException("ERROR: master failed status check: %s"%msg)
                logging.getLogger('roslaunch.core').debug('master.is_running[%s]: True'%self.uri)
                return True
            finally:
                socket.setdefaulttimeout(to_orig) 
        except:
            logging.getLogger('roslaunch.core').debug('master.is_running[%s]: True'%self.uri)
            return False

## number of seconds that a child machine is allowed to register with
## the parent before being considered failed
_DEFAULT_REGISTER_TIMEOUT = 10.0 

## Data structure for storing information about a machine in the ROS
## system.  Corresponds to the 'machine' tag in the launch
## specification.
class Machine(object):
    __slots__ = ['name', 'ros_root', 'ros_package_path', 'ros_ip',\
                 'address', 'ssh_port', 'user', 'password', 'assignable',\
                 'env_args', 'timeout']
    def __init__(self, name, ros_root, ros_package_path, \
                 address, ros_ip=None, ssh_port=22, user=None, password=None, \
                 assignable=True, env_args=[], timeout=None):
        self.name = name
        self.ros_root = ros_root
        self.ros_package_path = ros_package_path
        self.ros_ip = ros_ip or None
        self.user = user or None
        self.password = password or None
        self.address = address
        self.ssh_port = ssh_port
        self.assignable = assignable
        self.env_args = env_args
        self.timeout = timeout or _DEFAULT_REGISTER_TIMEOUT
        
    def __str__(self):
        return "Machine(name[%s] ros_root[%s] ros_package_path[%s] ros_ip[%s] address[%s] ssh_port[%s] user[%s] assignable[%s] env_args[%s] timeout[%s])"%(self.name, self.ros_root, self.ros_package_path, self.ros_ip, self.address, self.ssh_port, self.user, self.assignable, str(self.env_args), self.timeout)
    def __eq__(self, m2):
        if not isinstance(m2, Machine):
            return False
        return self.name == m2.name and \
               self.assignable == m2.assignable and \
               self.config_equals(m2)
    
    ## Get a key that represents the configuration of the
    ## machine. machines with identical configurations have identical
    ## keys
    ##
    ## @param self
    ## @return str: configuration key
    def config_key(self):
        return "Machine(address[%s] ros_root[%s] ros_package_path[%s] ros_ip[%s] ssh_port[%s] user[%s] password[%s] env_args[%s] timeout[%s])"%(self.address, self.ros_root, self.ros_package_path, self.ros_ip or '', self.ssh_port, self.user or '', self.password or '', str(self.env_args), self.timeout)
    ## @return True if machines have identical configurations
    def config_equals(self, m2):
        if not isinstance(m2, Machine):
            return False
        return self.ros_root  == m2.ros_root and \
               self.ros_package_path == m2.ros_package_path and \
               self.ros_ip == m2.ros_ip and \
               self.address  == m2.address and \
               self.ssh_port  == m2.ssh_port and \
               self.user == m2.user and \
               self.password == m2.password and \
               set(self.env_args) == set(m2.env_args) and \
               self.timeout == m2.timeout

    def __ne__(self, m2):
        return not self.__eq__(m2)

## Data structure for storing information about a desired parameter in
## the ROS system Corresponds to the 'param' tag in the launch
## specification.
class Param(object):
    def __init__(self, key, value):
        self.key = key
        self.value = value
    def __eq__(self, p):
        if not isinstance(p, Param):
            return False
        return p.key == self.key and p.value == self.value
    def __ne__(self, p):
        return not self.__eq__(p)
    def __str__(self):
        return "%s=%s"%(self.key, self.value)
    def __repr__(self):
        return "%s=%s"%(self.key, self.value)

#TODO: lists, maps(?)
## convert a value from a string representation into the specified
## type
## @param type str: int, double, string, bool, or auto
def convert_value(value, type):
    type = type.lower()
    # currently don't support XML-RPC date, dateTime, maps, or list
    # types
    if type == 'auto':
        #attempt numeric conversion
        try:
            if '.' in value:
                return string.atof(value)
            else:
                return string.atoi(value)
        except ValueError, e:
            pass
        #bool
        lval = value.lower()
        if lval == 'true' or lval == 'false':
            return convert_value(value, 'bool')
        #string
        return value
    elif type == 'str' or type == 'string':
        return value
    elif type == 'int':
        return string.atoi(value)
    elif type == 'double':
        return string.atof(value)
    elif type == 'bool' or type == 'boolean':
        value = value.lower()
        if value == 'true' or value == '1':
            return True
        elif value == 'false' or value == '0':
            return False
        raise Exception("%s is not a '%' type"%(value, type))
    else:
        raise Exception("Unknown type '%s'"%type)        
            
_local_m = None
## Get the Machine instance representing the local machine
def local_machine():
    global _local_m
    if _local_m is None:
        _local_m = Machine('', get_ros_root(), \
                           get_ros_package_path(), 'localhost',\
                           ros_ip=roslib.network.get_address_override())
    return _local_m

## Data structure for storing information about a desired node in
## the ROS system Corresponds to the 'node' tag in the launch
## specification.
class Node(object):
    __slots__ = ['package', 'type', 'name', 'namespace', \
                 'machine_name', 'machine', 'args', 'respawn', \
                 'remap_args', 'env_args',\
                 'process_name', 'output', 'cwd',
                 'launch_prefix']

    ## @param package str: node package name
    ## @param node_type str: node type
    ## @param name str: node name    
    ## @param namespace str: namespace for node
    ## @param machine_name str: name of machine to run node on
    ## @param args str: argument string to pass to node executable
    ## @param respawn bool: if True, respawn node if it dies
    ## @param remap_args [(str, str)]: list of [(from, to)] remapping arguments
    ## @param env_args [(str, str)]: list of [(key, value)] of
    ## additional environment vars to set for node
    ## @param output str: where to log output to, either Node, 'screen' or 'log'
    ## @param cwd str: current working directory of node, either 'node' or 'ros-root'
    ## @param launch_prefix str: launch command/arguments to prepend to node executable arguments
    def __init__(self, package, node_type, name=None, namespace='/', \
                 machine_name=None, args='', respawn=False, \
                 remap_args=None,env_args=None, output=None, cwd=None, launch_prefix=None):
        self.package = package
        self.type = node_type
        self.name = name or None
        self.namespace = roslib.names.make_global_ns(namespace or '/')
        self.machine_name = machine_name or None
        
        ## machine is the assigned machine instance. should probably
        ## consider storing this elsewhere as it can be inconsistent
        ## with machine_name and is also a runtime, rather than
        ## configuration property
        self.machine = None
        
        self.respawn = respawn
        self.args = args or ''
        self.remap_args = remap_args or []
        self.env_args = env_args or []        
        self.output = output or 'log'
        self.cwd = cwd or None
        self.launch_prefix = launch_prefix or None
        
        ## slot to store the process name in so that we can query the
        ## associated process state
        self.process_name = None

    def xmltype(self):
        return 'node'
    
    def xmlattrs(self):
        name_str = cwd_str = respawn_str = None
        if self.name:
            name_str = self.name
        if self.cwd:
            cwd_str = self.cwd
        if self.respawn:
            respawn_str = 'true'
        else:
            respawn_str = 'false'

        return [
            ('pkg', self.package),
            ('type', self.type),
            ('machine', self.machine_name),
            ('ns', self.namespace),
            ('args', self.args),
            ('output', self.output),
            ('cwd', cwd_str), 
            ('respawn', respawn_str), #not valid on <test>
            ('name', name_str),
            ('launch-prefix', self.launch_prefix),
            ]

    ## convert representation into remote representation. Remote representation does
    ## not include parameter settings or 'machine' attribute
    def to_remote_xml(self):
        respawn_str = test_name_str = name_str = cwd_str = ''
        t = self.xmltype()
        attrs = [(a, v) for a, v in self.xmlattrs() if v != None and a != 'machine']
        xmlstr = '<%s %s>\n'%(t, ' '.join(['%s="%s"'%(val[0], _xml_escape(val[1])) for val in attrs]))
        xmlstr += ''.join(['  <remap from="%s" to="%s" />\n'%tuple(r) for r in self.remap_args])
        xmlstr += ''.join(['  <env name="%s" value="%s" />\n'%tuple(e) for e in self.env_args])
        xmlstr += "</%s>"%t
        return xmlstr
        
## escape \a s for XML
## @param s str: string to escape
## @return str: string with XML entities (<, >, ", &) escaped.
def _xml_escape(s):
    # gross, but doesn't need to be fast. always replace amp first
    s = str(s)
    s = s.replace('&', '&amp;')
    s = s.replace('"', '&quot;')
    s = s.replace('>', '&gt;')
    s = s.replace('<', '&lt;')
    return s
    
TEST_TIME_LIMIT_DEFAULT = 1 * 60 #seconds


## A Test is a Node with special semantics that it performs a
## unit/integration test.  The data model is the same except the
## option to set the respawn flag is removed.
class Test(Node):
    __slots__ = ['test_name', 'time_limit', 'retry']

    ## Construct a new test node.
    ## @param test_name str: name of test for recording in test results
    ## @param time_limit int/float/long: number of seconds that a test
    ## should run before marked as a failure
    def __init__(self, test_name, package, node_type, name=None, \
                 namespace='/', machine_name=None, args='', \
                 remap_args=None, env_args=None, time_limit=None, cwd=None,
                 launch_prefix=None, retry=None):
        super(Test, self).__init__(package, node_type, name=name, \
                                   namespace=namespace, \
                                   machine_name=machine_name, args=args, \
                                   remap_args=remap_args,
                                   env_args=env_args,
                                   #output always is log
                                   output='log', cwd=cwd,
                                   launch_prefix=launch_prefix)
        self.test_name = test_name

        self.retry = retry or 0
        time_limit = time_limit or TEST_TIME_LIMIT_DEFAULT
        if not type(time_limit) in (float, int, long):
            raise ValueError("'time-limit' must be a number")
        time_limit = float(time_limit) #force to floating point
        if time_limit <= 0:
            raise ValueError("'time-limit' must be a positive number")

        self.time_limit = time_limit

    def xmltype(self):
        return 'test'
    
    ## NOTE: xmlattrs does not necessarily procedure identical XML as
    ## to what it was initialized with, though the properties are the same
    def xmlattrs(self):
        attrs = Node.xmlattrs(self)
        attrs = [(a, v) for (a, v) in attrs if a != 'respawn']
        attrs.append(('test-name', self.test_name))

        if self.retry:
            attrs.append(('retry', str(self.retry)))
        if self.time_limit != TEST_TIME_LIMIT_DEFAULT:
            attrs.append(('time-limit', self.time_limit))
        return attrs

        
## Executable is a generic container for exectuable commands.
class Executable(object):
    
    ## @param self
    ## @param cmd str: name of command to run
    ## @param args (str,): arguments to command
    ## @param phase str:
    ## PHASE_SETUP|PHASE_RUN|PHASE_TEARDOWN. Indicates whether the
    ## command should be run before, during, or after launch.
    def __init__(self, cmd, args, phase=PHASE_RUN):
        self.command = cmd
        self.args = args
        self.phase = phase
    def __repr__(self):
        return "%s %s"%(self.command, ' '.join(self.args))
    def __str__(self):
        return "%s %s"%(self.command, ' '.join(self.args))
        
## RosbinExecutables are exectuables stored in ROS_ROOT/bin. 
class RosbinExecutable(Executable):
    def __init__(self, cmd, args, phase=PHASE_RUN):
        super(RosbinExecutable, self).__init__(cmd, args, phase)
    def __repr__(self):
        return "ros/bin/%s %s"%(self.command, ' '.join(self.args))
    def __str__(self):
        return "ros/bin/%s %s"%(self.command, ' '.join(self.args))

    
## utility routine for generating run IDs (UUIDs)
## @return str: guid
def generate_run_id():
    try:
        import uuid
    except ImportError, e:
        import roslib.uuid as uuid
    return str(uuid.uuid1())

