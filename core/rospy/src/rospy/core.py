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

"""rospy internal core implementation library"""

from __future__ import with_statement

import atexit
import logging
import os
import signal
import sys
import threading
import time
import traceback
import types
import urlparse
import xmlrpclib

import roslib.msg
import roslib.rosenv 
import roslib.roslogging

import rospy.exceptions

from rospy.names import *
from rospy.impl.validators import ParameterInvalid

_logger = logging.getLogger("rospy.core")

# number of seconds to wait to join on threads. network issue can
# cause joins to be not terminate gracefully, and it's better to
# teardown dirty than to hang
_TIMEOUT_SHUTDOWN_JOIN = 5.

import warnings
def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emmitted
    when the function is used."""
    def newFunc(*args, **kwargs):
        warnings.warn("Call to deprecated function %s." % func.__name__,
                      category=DeprecationWarning, stacklevel=2)
        return func(*args, **kwargs)
    newFunc.__name__ = func.__name__
    newFunc.__doc__ = func.__doc__
    newFunc.__dict__.update(func.__dict__)
    return newFunc

#########################################################
# ROSRPC

ROSRPC = "rosrpc://"

def parse_rosrpc_uri(uri):
    """
    utility function for parsing ROS-RPC URIs
    @param uri: ROSRPC URI
    @type  uri: str
    @return: address, port
    @rtype: (str, int)
    @raise ParameterInvalid: if uri is not a valid ROSRPC URI
    """
    if uri.startswith(ROSRPC):
        dest_addr = uri[len(ROSRPC):]            
    else:
        raise ParameterInvalid("Invalid protocol for ROS service URL: %s"%uri)
    try:
        if '/' in dest_addr:
            dest_addr = dest_addr[:dest_addr.find('/')]
        dest_addr, dest_port = dest_addr.split(':')
        dest_port = int(dest_port)
    except:
        raise ParameterInvalid("ROS service URL is invalid: %s"%uri)
    return dest_addr, dest_port

#########################################################
        
# rospy logger
_rospy_logger = logging.getLogger("rospy.internal")

_logdebug_handlers = []
_loginfo_handlers = []
_logwarn_handlers = []
_logerr_handlers = []
_logfatal_handlers = []

# we keep a separate, non-rosout log file to contain stack traces and
# other sorts of information that scare users but are essential for
# debugging

def rospydebug(msg, *args):
    """Internal rospy client library debug logging"""
    _rospy_logger.debug(msg, *args)
def rospyerr(msg, *args):
    """Internal rospy client library error logging"""
    _rospy_logger.error(msg, *args)
def rospywarn(msg, *args):
    """Internal rospy client library warn logging"""
    _rospy_logger.warn(msg, *args)
    
def add_log_handler(level, h):
    """
    Add handler for specified level
    @param level: log level (use constants from roslib.msg.Log)
    @type  level: int
    @param h: log message handler
    @type  h: fn
    @raise ROSInternalException: if level is invalid
    """
    if level == roslib.msg.Log.DEBUG:
        _logdebug_handlers.append(h)
    elif level == roslib.msg.Log.INFO:
        _loginfo_handlers.append(h)
    elif level == roslib.msg.Log.WARN:
        _logwarn_handlers.append(h)
    elif level == roslib.msg.Log.ERROR:
        _logerr_handlers.append(h)
    elif level == roslib.msg.Log.FATAL:
        _logfatal_handlers.append(h)
    else:
        raise rospy.exceptions.ROSInternalException("invalid log level: %s"%level)
    
def logdebug(msg, *args):
    """
    Log a debug message to the /rosout topic
    @param msg: message to log, may include formatting arguments
    @type  msg: str
    @param args: format-string arguments, if necessary
    """    
    if args:
        msg = msg%args
    for h in _logdebug_handlers:
        h(msg)

def logwarn(msg, *args):
    """
    Log a warning message to the /rosout topic
    @param msg: message to log, may include formatting arguments
    @type  msg: str
    @param args: format-string arguments, if necessary    
    """    
    if args:
        msg = msg%args
    for h in _logwarn_handlers:
        h(msg)

def loginfo(msg, *args):
    """
    Log an info message to the /rosout topic    
    @param msg: message to log, may include formatting arguments
    @type  msg: str
    @param args: format-string arguments, if necessary
    """    
    if args:
        msg = msg%args
    for h in _loginfo_handlers:
        h(msg)
logout = loginfo # alias deprecated name

def logerr(msg, *args):
    """
    Log an error message to the /rosout topic
    @param msg: message to log, may include formatting arguments
    @type  msg: str
    @param args: format-string arguments, if necessary
    """
    if args:
        msg = msg%args
    for h in _logerr_handlers:
        h(msg)
logerror = logerr # alias logerr

def logfatal(msg, *args):
    """
    Log an error message to the /rosout topic    
    @param msg: message to log, may include formatting arguments
    @type  msg: str
    @param args: format-string arguments, if necessary
    """        
    if args:
        msg = msg%args
    for h in _logfatal_handlers:
        h(msg)

#########################################################
# CONSTANTS

MASTER_NAME = "master" #master is a reserved node name for the central master

def get_ros_root(required=False, env=None):
    """
    Get the value of ROS_ROOT.
    @param env: override environment dictionary
    @type  env: dict
    @param required: if True, fails with ROSException
    @return: Value of ROS_ROOT environment
    @rtype: str
    @raise ROSException: if require is True and ROS_ROOT is not set
    """
    if env is None:
        env = os.environ
    ros_root = env.get(roslib.rosenv.ROS_ROOT, None)
    if required and not ros_root:
        raise rospy.exceptions.ROSException('%s is not set'%roslib.rosenv.ROS_ROOT)
    return ros_root


#########################################################
# API

_uri = None
def get_node_uri():
    """
    Get this Node's URI.
    @return: this Node's XMLRPC URI
    @rtype: str
    """
    return _uri

def set_node_uri(uri):
    """set the URI of the local node.
    This is an internal API method, it does not actually affect the XMLRPC URI of the Node."""
    global _uri
    _uri = uri

#########################################################
# Logging

_log_filename = None
def configure_logging(node_name, level=logging.INFO):
    """
    Setup filesystem logging for this node
    @param node_name: Node's name
    @type  node_name str
    @param level: (optional) Python logging level (INFO, DEBUG, etc...). (Default: logging.INFO)
    @type  level: int
    """
    global _log_filename

    # #988 __log command-line remapping argument
    mappings = get_mappings()
    if '__log' in get_mappings():
        logfilename_remap = mappings['__log']
        filename = os.path.abspath(logfilename_remap)
    else:
        # fix filesystem-unsafe chars
        filename = node_name.replace('/', '_') + '.log'
        if filename[0] == '_':
            filename = filename[1:]
        if not filename:
            raise rospy.exceptions.ROSException('invalid configure_logging parameter: %s'%node_name)
    _log_filename = roslib.roslogging.configure_logging('rospy', level, filename=filename, additional=['roslib', 'rosout', 'xmlrpc'])

class NullHandler(logging.Handler):
    def emit(self, record):
        pass
    
# keep logging happy until we have the node name to configure with
logging.getLogger('rospy').addHandler(NullHandler())    
    

#########################################################
# Init/Shutdown/Exit API and Handlers

_client_ready = False


def is_initialized():
    """
    Get the initialization state of the local node. If True, node has
    been configured.
    @return: True if local node initialized
    @rtype: bool
    """
    return _client_ready
def set_initialized(initialized):
    """
    set the initialization state of the local node
    @param initialized: True if node initialized
    @type  initialized: bool
    """
    global _client_ready
    _client_ready = initialized

_shutdown_lock  = threading.RLock()

# _shutdown_flag flags that rospy is in shutdown mode, in_shutdown
# flags that the shutdown routine has started. These are separate
# because 'pre-shutdown' hooks require rospy to be in a non-shutdown
# mode. These hooks are executed during the shutdown routine.
_shutdown_flag  = False
_in_shutdown = False

# various hooks to call on shutdown. shutdown hooks are called in the
# shutdown state, preshutdown are called just before entering shutdown
# state, and client shutdown is called before both of these.
_shutdown_hooks = []
_preshutdown_hooks = []
_client_shutdown_hooks = []
# threads that must be joined on shutdown
_shutdown_threads = []

_signalChain = {}

def is_shutdown():
    """
    @return: True if shutdown flag has been set
    @rtype: bool
    """
    return _shutdown_flag

def _add_shutdown_hook(h, hooks):
    """
    shared implementation of add_shutdown_hook and add_preshutdown_hook
    """
    if type(h) not in [types.FunctionType, types.MethodType]:
        raise TypeError("shutdown hook [%s] must be a function: %s"%(h, type(h)))
    if _shutdown_flag:
        _logger.warn("add_shutdown_hook called after shutdown")
        h("already shutdown")
        return
    with _shutdown_lock:
        if hooks is None:
            # race condition check, don't log as we are deep into shutdown
            return
        hooks.append(h)

def _add_shutdown_thread(t):
    """
    Register thread that must be joined() on shutdown
    """
    if _shutdown_flag:
        #TODO
        return
    with _shutdown_lock:
        if _shutdown_threads is None:
            # race condition check, don't log as we are deep into shutdown
            return
        # in order to prevent memory leaks, reap dead threads. The
        # last thread may not get reaped until shutdown, but this is
        # relatively minor
        for other in _shutdown_threads[:]:
            if not other.isAlive():
                _shutdown_threads.remove(other)
        _shutdown_threads.append(t)

def add_client_shutdown_hook(h):
    """
    Add client method to invoke when system shuts down. Unlike
    L{add_shutdown_hook} and L{add_preshutdown_hooks}, these methods
    will be called before any rospy internal shutdown code.
    
    @param h: function that takes in a single string argument (shutdown reason)
    @type  h: fn(str)
    """
    _add_shutdown_hook(h, _client_shutdown_hooks)

def add_preshutdown_hook(h):
    """
    Add method to invoke when system shuts down. Unlike
    L{add_shutdown_hook}, these methods will be called before any
    other shutdown hooks.
    
    @param h: function that takes in a single string argument (shutdown reason)
    @type  h: fn(str)
    """
    _add_shutdown_hook(h, _preshutdown_hooks)

def add_shutdown_hook(h):
    """
    Add method to invoke when system shuts down.

    Shutdown hooks are called in the order that they are
    registered. This is an internal API method that is used to
    cleanup. See the client X{on_shutdown()} method if you wish to
    register client hooks.

    @param h: function that takes in a single string argument (shutdown reason)
    @type  h: fn(str)
    """
    _add_shutdown_hook(h, _shutdown_hooks)

def signal_shutdown(reason):
    """
    Initiates shutdown process by signaling objects waiting on _shutdown_lock.
    Shutdown and pre-shutdown hooks are invoked.
    @param reason: human-readable shutdown reason, if applicable
    @type  reason: str
    """
    global _shutdown_flag, _in_shutdown, _shutdown_lock, _shutdown_hooks
    _logger.info("signal_shutdown [%s]"%reason)
    if _shutdown_flag or _in_shutdown:
        return
    with _shutdown_lock:
        if _shutdown_flag or _in_shutdown:
            return
        _in_shutdown = True

        # make copy just in case client re-invokes shutdown
        for h in _client_shutdown_hooks:
            try:
                # client shutdown hooks do not accept a reason arg
                h()
            except:
                traceback.print_exc()
        del _client_shutdown_hooks[:]

        for h in _preshutdown_hooks:
            try:
                h(reason)
            except:
                traceback.print_exc()
        del _preshutdown_hooks[:]

        # now that pre-shutdown hooks have been called, raise shutdown
        # flag. This allows preshutdown hooks to still publish and use
        # service calls properly
        _shutdown_flag = True
        for h in _shutdown_hooks:
            try:
                h(reason)
            except Exception, e:
                print >> sys.stderr, "signal_shutdown hook error[%s]"%e
        del _shutdown_hooks[:]

        threads = _shutdown_threads[:]

    for t in threads:
        if t.isAlive():
            t.join(_TIMEOUT_SHUTDOWN_JOIN)
    del _shutdown_threads[:]
    try:
        time.sleep(0.1) #hack for now until we get rid of all the extra threads
    except KeyboardInterrupt: pass

def _ros_signal(sig, stackframe):
    signal_shutdown("signal-"+str(sig))
    prev_handler = _signalChain.get(sig, None)
    if prev_handler is not None and not type(prev_handler) == int:
        try:
            prev_handler(sig, stackframe)
        except KeyboardInterrupt:
            pass #filter out generic keyboard interrupt handler

def _ros_atexit():
    signal_shutdown('atexit')
atexit.register(_ros_atexit)

# #687
def register_signals():
    """
    register system signal handlers for SIGTERM and SIGINT
    """
    _signalChain[signal.SIGTERM] = signal.signal(signal.SIGTERM, _ros_signal)
    _signalChain[signal.SIGINT]  = signal.signal(signal.SIGINT, _ros_signal)
    
# Validators ######################################

def is_topic(param_name):
    """
    Validator that checks that parameter is a valid ROS topic name
    """    
    def validator(param_value, caller_id):
        v = valid_name_validator_resolved(param_name, param_value, caller_id)
        if param_value == '/':
            raise ParameterInvalid("ERROR: parameter [%s] cannot be the global namespace"%param_name)            
        return v
    return validator

_proxies = {} #cache ServerProxys
def xmlrpcapi(uri):
    """
    @return: instance for calling remote server or None if not a valid URI
    @rtype: xmlrpclib.ServerProxy
    """
    if uri is None:
        return None
    uriValidate = urlparse.urlparse(uri)
    if not uriValidate[0] or not uriValidate[1]:
        return None
    if not _proxies.has_key(uri):
        _proxies[uri] = xmlrpclib.ServerProxy(uri)
    return _proxies[uri]

