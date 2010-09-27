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
Core roslaunch model and lower-level utility routines.
"""

import os
import logging
import socket
import sys

import roslib.names 
import roslib.network
import roslib.substitution_args
import roslib.rosenv

#TODO:temporary until xacro is ported after next ROS stable release
resolve_args = roslib.substitution_args.resolve_args

class RLException(Exception):
    """Base roslaunch exception type"""
    pass


## Phases allow executables to be assigned to a particular run period
PHASE_SETUP    = 'setup'
PHASE_RUN      = 'run'
PHASE_TEARDOWN = 'teardown'

_child_mode = False
def is_child_mode():
    """
    @return: True if roslaunch is running in remote child mode
    @rtype: bool
    """
    return _child_mode
def set_child_mode(child_mode):
    """
    @param child_mode: True if roslaunch is running in remote
    child mode
    @type  child_mode: bool
    """
    global _child_mode
    _child_mode = child_mode

def is_machine_local(machine):
    """
    Check to see if machine is local. NOTE: a machine is not local if
    its user credentials do not match the current user.
    @param machine: Machine
    @type  machine: L{Machine}
    @return: True if machine is local and doesn't require remote login
    @rtype: bool
    """    
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
def printlog(msg):
    """
    Core utility for printing message to stdout as well as printlog handlers
    @param msg: message to print
    @type  msg: str
    """
    for h in _printlog_handlers:
        try: # don't let this bomb out the actual code
            h(msg)
        except:
            pass
    try: # don't let this bomb out the actual code        
        print msg
    except:
        pass

def printlog_bold(msg):
    """
    Similar to L{printlog()}, but the message printed to screen is bolded for greater clarity
    @param msg: message to print
    @type  msg: str
    """
    for h in _printlog_handlers:
        try: # don't let this bomb out the actual code
            h(msg)
        except:
            pass
    try: # don't let this bomb out the actual code        
        print '\033[1m%s\033[0m'%msg
    except:
        pass

def printerrlog(msg):
    """
    Core utility for printing message to stderr as well as printerrlog handlers
    @param msg: message to print
    @type  msg: str
    """    
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

def add_printlog_handler(h):
    """
    Register additional handler for printlog()
    """
    _printlog_handlers.append(h)

def add_printerrlog_handler(h):
    """
    Register additional handler for printerrlog()
    """
    _printerrlog_handlers.append(h)

def clear_printlog_handlers():
    """
    Delete all printlog handlers. required for testing
    """
    del _printlog_handlers[:]

def clear_printerrlog_handlers():
    """
    Delete all printerrlog handlers. required for testing
    """
    del _printerrlog_handlers[:]
    
def setup_env(node, machine, master_uri):
    """
    Create dictionary of environment variables to set for launched
    process.

    setup_env() will only set ROS_*, PYTHONPATH, and user-specified
    environment variables.
    
    @param machine Machine: machine being launched on
    @type  machine: L{Machine}
    @param node: node that is being launched or None
    @type  node: L{Node} 
    @param master_uri: ROS master URI
    @type  master_uri: str
    @return: process env dictionary
    @rtype: dict
    """
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

def rle_wrapper(fn):
    """
    Wrap lower-level exceptions in RLException class
    @return: function wrapper that throws an RLException if the
        wrapped function throws an Exception
    @rtype: fn
    """    
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
    """
    @return: ROS_PACKAGE_PATH value
    @rtype: str
    """
    # ROS_PACKAGE_PATH not required to be set
    return roslib.rosenv.get_ros_package_path(required=False)

def remap_localhost_uri(uri, force_localhost=False):
    """
    Resolve localhost addresses to an IP address so that
    @param uri: XML-RPC URI
    @type  uri: str
    @param force_localhost: if True, URI is mapped onto the local machine no matter what
    @type  force_localhost: bool
    """
    hostname, port = roslib.network.parse_http_host_and_port(uri)
    if force_localhost or hostname == 'localhost':
        return roslib.network.create_local_xmlrpc_uri(port)
    else:
        return uri

##################################################################
# DATA STRUCTURES

class Master:
    """
    Data structure for representing and querying state of master 
    """
    __slots__ = ['type', 'auto', 'uri', 'log_output']
    ## don't start a master
    AUTO_NO    = 0
    ## start a master if one isn't running
    AUTO_START   = 1
    ## start/restart master (i.e. restart an existing master)
    AUTO_RESTART = 2
    ROSMASTER = 'rosmaster'
    
    # deprecated
    ZENMASTER = 'zenmaster'        

    def __init__(self, type_=None, uri=None, auto=None):
        """
        Create new Master instance.
        @param uri: master URI
        @type  uri: str
        @param type_: Currently only support 'rosmaster' 
        @type  type_: str
        @param auto: AUTO_NO | AUTO_START | AUTO_RESTART. AUTO_NO
          is the default
        @type  auto: int
        """
        if auto is not None and type(auto) != int:
            raise RLException("invalid auto value: %s"%auto)            
        self.type = type_ or Master.ROSMASTER
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

    def get(self):
        """
        @return: XMLRPC proxy for communicating with master
        @rtype: xmlrpclib.ServerProxy
        """
        import xmlrpclib
        return xmlrpclib.ServerProxy(self.uri)
    
    def get_multi(self):
        """
        @return: multicall XMLRPC proxy for communicating with master
        @rtype: xmlrpclib.MultiCall
        """
        import xmlrpclib
        return xmlrpclib.MultiCall(self.get())

    def set_port(self, port):
        """
        Override port specification of Master. This only has an effect on masters that have not
        been launched yet.
        @param port: port
        @type  port: int
        """
        host, _ = roslib.network.parse_http_host_and_port(self.uri)
        self.uri = 'http://%s:%s/'%(host, port)
    
    def is_running(self):
        """
        Check if master is running. 
        @return: True if the master is running
        @rtype: bool
        """
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

class Machine(object):
    """
    Data structure for storing information about a machine in the ROS
    system.  Corresponds to the 'machine' tag in the launch
    specification.
    """
    __slots__ = ['name', 'ros_root', 'ros_package_path', 'ros_ip',\
                 'address', 'ssh_port', 'user', 'password', 'assignable',\
                 'env_args', 'timeout']
    def __init__(self, name, ros_root, ros_package_path, \
                 address, ros_ip=None, ssh_port=22, user=None, password=None, \
                 assignable=True, env_args=[], timeout=None):
        """
        @param name: machine name
        @type  name: str
        @param ros_root: ROS_ROOT on machine
        @type  ros_root: str
        @param ros_package_path: ROS_PACKAGE_PATH on machine
        @type  ros_package_path: str
        @param address: network address of machine
        @type  address: str
        @param ros_ip: ROS_IP on machine
        @type  ros_ip: str
        @param ssh_port: SSH port number
        @type  ssh_port: int
        @param user: SSH username
        @type  user: str
        @param password: SSH password. Not recommended for use. Use SSH keys instead.
        @type  password: str
        """
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
    
    def config_key(self):
        """
        Get a key that represents the configuration of the
        machine. machines with identical configurations have identical
        keys
    
        @return: configuration key
        @rtype:  str
        """
        return "Machine(address[%s] ros_root[%s] ros_package_path[%s] ros_ip[%s] ssh_port[%s] user[%s] password[%s] env_args[%s] timeout[%s])"%(self.address, self.ros_root, self.ros_package_path, self.ros_ip or '', self.ssh_port, self.user or '', self.password or '', str(self.env_args), self.timeout)

    def config_equals(self, m2):
        """
        @return: True if machines have identical configurations
        @rtype: bool
        """
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

class Param(object):
    """
    Data structure for storing information about a desired parameter in
    the ROS system Corresponds to the 'param' tag in the launch
    specification.
    """
    def __init__(self, key, value):
        self.key = roslib.names.canonicalize_name(key)
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
            
_local_m = None
def local_machine():
    """
    @return: Machine instance representing the local machine
    @rtype: L{Machine}
    """
    global _local_m
    if _local_m is None:
        _local_m = Machine('', get_ros_root(), \
                           get_ros_package_path(), 'localhost',\
                           ros_ip=roslib.network.get_address_override())
    return _local_m

class Node(object):
    """
    Data structure for storing information about a desired node in
    the ROS system Corresponds to the 'node' tag in the launch
    specification.
    """
    __slots__ = ['package', 'type', 'name', 'namespace', \
                 'machine_name', 'machine', 'args', 'respawn', \
                 'remap_args', 'env_args',\
                 'process_name', 'output', 'cwd',
                 'launch_prefix', 'required',
                 'filename']

    def __init__(self, package, node_type, name=None, namespace='/', \
                 machine_name=None, args='', respawn=False, \
                 remap_args=None,env_args=None, output=None, cwd=None, \
                 launch_prefix=None, required=False, filename='<unknown>'):
        """
        @param package: node package name
        @type  package: str
        @param node_type: node type
        @type  node_type: str
        @param name: node name
        @type  name: str
        @param namespace: namespace for node
        @type  namespace: str
        @param machine_name: name of machine to run node on
        @type  machine_name: str
        @param args: argument string to pass to node executable
        @type  args: str
        @param respawn: if True, respawn node if it dies
        @type  respawn: bool
        @param remap_args: list of [(from, to)] remapping arguments
        @type  remap_args: [(str, str)]: 
        @param env_args: list of [(key, value)] of
        additional environment vars to set for node
        @type  env_args: [(str, str)]
        @param output: where to log output to, either Node, 'screen' or 'log'
        @type  output: str
        @param cwd: current working directory of node, either 'node', 'ROS_HOME' or 'ros-root'. Default: ROS_HOME
        @type  cwd: str
        @param launch_prefix: launch command/arguments to prepend to node executable arguments
        @type  launch_prefix: str
        @param required: node is required to stay running (launch fails if node dies)
        @type  required: bool
        @param filename: name of file Node was parsed from
        @type  filename: str

        @raise ValueError: if parameters do not validate
        """        

        self.package = package
        self.type = node_type
        self.name = name or None
        self.namespace = roslib.names.make_global_ns(namespace or '/')
        self.machine_name = machine_name or None
        self.respawn = respawn
        self.args = args or ''
        self.remap_args = remap_args or []
        self.env_args = env_args or []        
        self.output = output
        self.cwd = cwd
        if self.cwd == 'ros_home': # be lenient on case
            self.cwd = 'ROS_HOME'
        elif self.cwd == 'ros-root':
            printerrlog("WARNING: 'ros-root' value for <node> 'cwd' attribute is deprecated.")
            
        self.launch_prefix = launch_prefix or None
        self.required = required
        self.filename = filename

        if self.respawn and self.required:
            raise ValueError("respawn and required cannot both be set to true")
        
        # validation
        if self.name and roslib.names.SEP in self.name: # #1821, namespaces in nodes need to be banned
            raise ValueError("node name cannot contain a namespace")
        if not len(self.package.strip()):
            raise ValueError("package must be non-empty")
        if not len(self.type.strip()):
            raise ValueError("type must be non-empty")
        if not self.output in ['log', 'screen', None]:
            raise ValueError("output must be one of 'log', 'screen'")
        if not self.cwd in ['ROS_HOME', 'ros-root', 'node', None]:
            raise ValueError("cwd must be one of 'ROS_HOME', 'ros-root', 'node'")
        
        # Extra slots for assigning later
        
        # slot to store the process name in so that we can query the
        # associated process state
        self.process_name = None

        # machine is the assigned machine instance. should probably
        # consider storing this elsewhere as it can be inconsistent
        # with machine_name and is also a runtime, rather than
        # configuration property
        self.machine = None

        
        
    def xmltype(self):
        return 'node'
    
    def xmlattrs(self):
        name_str = cwd_str = respawn_str = None
        if self.name:
            name_str = self.name
        if self.cwd:
            cwd_str = self.cwd

        return [
            ('pkg', self.package),
            ('type', self.type),
            ('machine', self.machine_name),
            ('ns', self.namespace),
            ('args', self.args),
            ('output', self.output),
            ('cwd', cwd_str), 
            ('respawn', self.respawn), #not valid on <test>
            ('name', name_str),
            ('launch-prefix', self.launch_prefix),
            ('required', self.required),
            ]

    #TODO: unify with to_remote_xml using a filter_fn
    def to_xml(self):
        """
        convert representation into XML representation. Currently cannot represent private parameters.
        @return: XML representation for remote machine
        @rtype: str
        """
        t = self.xmltype()
        attrs = [(a, v) for a, v in self.xmlattrs() if v != None]
        xmlstr = '<%s %s>\n'%(t, ' '.join(['%s="%s"'%(val[0], _xml_escape(val[1])) for val in attrs]))
        xmlstr += ''.join(['  <remap from="%s" to="%s" />\n'%tuple(r) for r in self.remap_args])
        xmlstr += ''.join(['  <env name="%s" value="%s" />\n'%tuple(e) for e in self.env_args])
        xmlstr += "</%s>"%t
        return xmlstr

    def to_remote_xml(self):
        """
        convert representation into remote representation. Remote representation does
        not include parameter settings or 'machine' attribute
        @return: XML representation for remote machine
        @rtype: str
        """
        t = self.xmltype()
        attrs = [(a, v) for a, v in self.xmlattrs() if v != None and a != 'machine']
        xmlstr = '<%s %s>\n'%(t, ' '.join(['%s="%s"'%(val[0], _xml_escape(val[1])) for val in attrs]))
        xmlstr += ''.join(['  <remap from="%s" to="%s" />\n'%tuple(r) for r in self.remap_args])
        xmlstr += ''.join(['  <env name="%s" value="%s" />\n'%tuple(e) for e in self.env_args])
        xmlstr += "</%s>"%t
        return xmlstr
        
def _xml_escape(s):
    """
    Escape string for XML
    @param s: string to escape
    @type  s: str
    @return: string with XML entities (<, >, ", &) escaped.
    @rtype: str
    """
    # gross, but doesn't need to be fast. always replace amp first
    s = str(s)
    s = s.replace('&', '&amp;')
    s = s.replace('"', '&quot;')
    s = s.replace('>', '&gt;')
    s = s.replace('<', '&lt;')
    return s
    
TEST_TIME_LIMIT_DEFAULT = 1 * 60 #seconds


class Test(Node):
    """
    A Test is a Node with special semantics that it performs a
    unit/integration test.  The data model is the same except the
    option to set the respawn flag is removed.
    """
    __slots__ = ['test_name', 'time_limit', 'retry']

    def __init__(self, test_name, package, node_type, name=None, \
                 namespace='/', machine_name=None, args='', \
                 remap_args=None, env_args=None, time_limit=None, cwd=None,
                 launch_prefix=None, retry=None, filename="<unknown>"):
        """
        Construct a new test node.
        @param test_name: name of test for recording in test results
        @type  test_name: str
        @param time_limit: number of seconds that a test
        should run before marked as a failure
        @type  time_limit: int/float/long
        """
        super(Test, self).__init__(package, node_type, name=name, \
                                   namespace=namespace, \
                                   machine_name=machine_name, args=args, \
                                   remap_args=remap_args,
                                   env_args=env_args,
                                   #output always is log
                                   output='log', cwd=cwd,
                                   launch_prefix=launch_prefix, filename=filename)
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
    
    def xmlattrs(self):
        """
        NOTE: xmlattrs does not necessarily produce identical XML as
        to what it was initialized with, though the properties are the same
        """
        attrs = Node.xmlattrs(self)
        attrs = [(a, v) for (a, v) in attrs if a != 'respawn']
        attrs.append(('test-name', self.test_name))

        if self.retry:
            attrs.append(('retry', str(self.retry)))
        if self.time_limit != TEST_TIME_LIMIT_DEFAULT:
            attrs.append(('time-limit', self.time_limit))
        return attrs

        
class Executable(object):
    """
    Executable is a generic container for exectuable commands.
    """
    
    def __init__(self, cmd, args, phase=PHASE_RUN):
        """
        @param cmd: name of command to run
        @type  cmd: str
        @param args: arguments to command
        @type  args: (str,)
        @param phase: PHASE_SETUP|PHASE_RUN|PHASE_TEARDOWN. Indicates whether the
            command should be run before, during, or after launch.
        @type  phase: str
        """
        self.command = cmd
        self.args = args
        self.phase = phase
    def __repr__(self):
        return "%s %s"%(self.command, ' '.join(self.args))
    def __str__(self):
        return "%s %s"%(self.command, ' '.join(self.args))
        
class RosbinExecutable(Executable):
    """
    RosbinExecutables are exectuables stored in ROS_ROOT/bin. 
    """
    def __init__(self, cmd, args, phase=PHASE_RUN):
        super(RosbinExecutable, self).__init__(cmd, args, phase)
    def __repr__(self):
        return "ros/bin/%s %s"%(self.command, ' '.join(self.args))
    def __str__(self):
        return "ros/bin/%s %s"%(self.command, ' '.join(self.args))

    
def generate_run_id():
    """
    Utility routine for generating run IDs (UUIDs)
    @return: guid
    @rtype: str
    """    
    import uuid
    return str(uuid.uuid1())

