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

import os
import sys
import signal
import threading
import traceback
import xmlrpclib
import urlparse
import logging
import atexit
import time

from rospy.names import *
from rospy.rosutil import *

import roslib.msg
from roslib.rosenv import ROS_ROOT, ROS_MASTER_URI, ROS_HOSTNAME, ROS_NAMESPACE, ROS_PACKAGE_PATH, ROS_LOG_DIR

_logger = logging.getLogger("rospy.core")
_mlogger = logging.getLogger("rospy.masterslave")

## Info-level master log statements. These statements may be printed
## to screen so they should be user-readable.
## @param msg str
## @param args arguments for \a msg if \a msg is a format string
def mloginfo(msg, *args):
    #mloginfo is in core so that it is accessible to master and masterdata
    _mlogger.info(msg, *args)
    if args:
        print msg%args
    else:
        print msg

## Warn-level master log statements. These statements may be printed
## to screen so they should be user-readable.
## @param msg str
## @param args arguments for \a msg if \a msg is a format string
def mlogwarn(msg, *args):
    #mloginfo is in core so that it is accessible to master and masterdata
    _mlogger.warn(msg, *args)
    if args:
        print "WARN: "+msg%args
    else:
        print "WARN: "+str(msg)

#########################################################
        
## client logger
_clogger = logging.getLogger("rospy.rosout")

_logdebug_handlers = [_clogger.debug]
_logout_handlers = [_clogger.info]
_logwarn_handlers = [_clogger.warn]
_logerr_handlers = [_clogger.error]
_logfatal_handlers = [_clogger.critical]

def add_log_handler(level, h):
    if level == roslib.msg.Log.DEBUG:
        _logdebug_handlers.append(h)
    elif level == roslib.msg.Log.INFO:
        _logout_handlers.append(h)
    elif level == roslib.msg.Log.WARN:
        _logwarn_handlers.append(h)
    elif level == roslib.msg.Log.ERROR:
        _logerr_handlers.append(h)
    elif level == roslib.msg.Log.FATAL:
        _logfatal_handlers.append(h)
    else:
        assert 1, "invalid level: %s"%level
    
## \ingroup clientapi Client API
## Log a debug message to the /rosout topic
def logdebug(msg):
    for h in _logdebug_handlers:
        h(msg)
## \ingroup clientapi Client API
## Log a warning message to the /rosout topic    
def logwarn(msg):
    for h in _logwarn_handlers:
        h(msg)
## \ingroup clientapi Client API
## Log an info message to the /rosout topic    
def logout(msg):
    for h in _logout_handlers:
        h(msg)
## \ingroup clientapi Client API
## Log an error message to the /rosout topic    
def logerr(msg):
    for h in _logerr_handlers:
        h(msg)
## \ingroup clientapi Client API
## Log an error message to the /rosout topic    
def logfatal(msg):
    for h in _logfatal_handlers:
        h(msg)

#########################################################
# CONSTANTS

MASTER_NAME = "master" #master is a reserved node name for the central master
TCPROS = "TCPROS" #name of our customized TCP protocol for accepting flows over server socket

# Return code slots
STATUS = 0
MSG = 1
VAL = 2

# Receive buffer size for topics/services (in bytes)
DEFAULT_BUFF_SIZE = 65536


#########################################################
# DATA STRUCTURES

# alias 
import roslib.rostime
Time = roslib.rostime.Time
Duration = roslib.rostime.Duration


#########################################################
# EXCEPTIONS

## \ingroup clientapi Client API
## Base exception class for ROS clients
class ROSException(Exception): pass

## Base class for exceptions that are internal to the ROS system
class ROSInternalException(Exception): pass
## Internal exception for errors initializing ROS state
class ROSInitException(ROSInternalException): pass
## Base class for transport-related exceptions
class TransportException(ROSInternalException): pass
## Internal class for representing broken connections
class TransportTerminated(TransportException): pass
## Internal exception for representing exceptions that occur
## establishing transports
class TransportInitError(TransportException): pass

#########################################################
# Environment variables used to configure master/slave

ENV_DOC = {
    ROS_ROOT: "Directory of ROS installation to use",
    ROS_PACKAGE_PATH: "Paths to search for additional ROS packages",    
    ROS_LOG_DIR: "Directory to write log files to",
    ROS_NAMESPACE: "Namespace to place node in", #defined in names.py
    ROS_MASTER_URI: "Default URI of ROS central server",
    ROS_HOSTNAME: "address to bind to",    
    }
ENV_VARS = ENV_DOC.keys()

## \ingroup clientapi
def get_ros_root(env=os.environ, require=False):
    rosRoot = env.get(ROS_ROOT, None)
    if require and not rosRoot:
        raise Exception('%s is not set'%ROS_ROOT)
    return rosRoot
getRosRoot = get_ros_root

#########################################################
# API

_uri = None
## \ingroup clientapi
def get_node_uri():
    return _uri
getNodeUri = get_node_uri 
## set the URI of the local node 
def set_node_uri(uri):
    global _uri
    _uri = uri

#########################################################
# Logging

_log_filename = None
def configure_logging(node_name):
    global _log_filename
    import roslib.scriptutil as scriptutil
    # fix filesystem-unsafe chars
    node_name = node_name.replace('/', '_')
    if node_name[0] == '_':
        node_name = node_name[1:]
    if not node_name:
        raise ROSException('invalid configure_logging parameter: %s'%node_name)
    _log_filename = scriptutil.configure_logging('rospy', logging.DEBUG, filename=node_name)
    #if _log_filename:
    #    print "... logging to ", _log_filename

class NullHandler(logging.Handler):
    def emit(self, record):
        pass
    
# keep logging happy until we have the node name to configure with
logging.getLogger('rospy').addHandler(NullHandler())    
    

#########################################################
# Init/Shutdown/Exit API and Handlers

_client_ready = False


## Get the initialization state of the local node. If True, node has
## been configured.
## @return bool True if local node initialized
def is_initialized():
    return _client_ready
## set the initialization state of the local node
## @param initialized bool: True if node initialized
def set_initialized(initialized):
    global _client_ready
    _client_ready = initialized

_shutdown_lock  = threading.Lock()
_shutdown_flag  = False
_shutdown_hooks = []

_signalChain = {}

## \ingroup clientapi Client API
## @return bool: True if shutdown flag has been set
def is_shutdown():
    return _shutdown_flag
isShutdown = is_shutdown #temp alias 

## Add method to invoke when system shuts down
#  @param h fn: function that takes in a single string argument (shutdown reason)
def add_shutdown_hook(h):
    if _shutdown_flag:
        _logger.warn("add_shutdown_hook called after shutdown")
        h("already shutdown")
        return
    try:
        _shutdown_lock.acquire()
        if _shutdown_hooks is None:
            # race condition check, don't log as we are deep into shutdown
            return
        _shutdown_hooks.append(h)
    finally:
        _shutdown_lock.release()

## \ingroup clientapi
## signal objects waiting on _shutdown_lock. Also invoke shutdown hooks
## @param reason str: human-readable shutdown reason, if applicable
def signal_shutdown(reason):
    global _shutdown_flag, _shutdown_lock, _shutdown_hooks
    if not _shutdown_flag:
        #_logger.debug("signal_shutdown start, reason [%s]"%reason)
        _shutdown_lock.acquire()
        if _shutdown_flag:
            return
        _shutdown_flag = True
        for h in _shutdown_hooks:
            try:
                h(reason)
            except:
                pass
        del _shutdown_hooks[:]
        _shutdown_lock.release()
        time.sleep(0.1) #hack for now until we get rid of all the extra threads

signalShutdown = signal_shutdown # temporary alias

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
## register system signal handlers for SIGTERM and SIGINT
def register_signals():
    _signalChain[signal.SIGTERM] = signal.signal(signal.SIGTERM, _ros_signal)
    _signalChain[signal.SIGINT]  = signal.signal(signal.SIGINT, _ros_signal)
    
# Validators ######################################

## \ingroup validators
## Validator that checks that parameter is a valid API handle
## (i.e. URI). Both http and rosrpc are allowed schemes.
def is_api(paramName):
    "Valid API handle"
    def validator(paramValue, callerId):
        if not paramValue or not type(paramValue) == str:
            raise ParameterInvalid("ERROR: parameter [%s] is not an XMLRPC URI"%paramName)
        if not paramValue.startswith("http://") and not paramValue.startswith(ROSRPC):
            raise ParameterInvalid("ERROR: parameter [%s] is not an RPC URI"%paramName)
        #could do more fancy parsing, but the above catches the major cases well enough
        return paramValue
    return validator

## \ingroup validators    
## Validator that checks that parameter is a valid ROS topic name
def is_topic(param_name):
    def validator(param_value, caller_id):
        v = valid_name_validator_resolved(param_name, param_value, caller_id)
        if param_value == '/':
            raise ParameterInvalid("ERROR: parameter [%s] cannot be the global namespace"%param_name)            
        return param_value
    return validator

## \ingroup validators
## Validator that checks that parameter is a valid ROS service name
def is_service(param_name):
    def validator(param_value, caller_id):
        v = valid_name_validator_resolved(param_name, param_value, caller_id)
        if param_value == '/':
            raise ParameterInvalid("ERROR: parameter [%s] cannot be the global namespace"%param_name)            
        return param_value
    return validator

# XML-RPC stubs ###################################

_proxies = {} #cache ServerProxys
## @return xmlrpclib.ServerProxy: instance for calling remote server
## or None if not a valid URI
def xmlrpcapi(uri):
    if uri is None:
        return None
    uriValidate = urlparse.urlparse(uri)
    if not uriValidate[0] or not uriValidate[1]:
        return None
    if not _proxies.has_key(uri):
        _proxies[uri] = xmlrpclib.ServerProxy(uri)
    return _proxies[uri]

#########################################################
# Parameter Server Cache

## Cache of values on the parameter server. Implementation
## is just a thread-safe dictionary.
class ParamServerCache(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.d = {}
    ## Update the value of the parameter in the cache
    ## @param self
    ## @param key str: parameter key
    ## @param value str: parameter value
    ## @throws KeyError if \a key is not already in the cache.
    def update(self, key, value):
        if not key in self.d:
            raise KeyError(key)
        try:
            self.lock.acquire()
            self.d[key] = value
        finally:
            self.lock.release()
    ## Set the value of the parameter in the cache. This is a
    ## prerequisite of calling update().
    ## @param self
    ## @param key str: parameter key
    ## @param value str: parameter value
    def update(self, key, value):
        try:
            self.lock.acquire()
            self.d[key] = value
        finally:
            self.lock.release()
            
    ## @param self
    ## @param key str: parameter key
    ## @return Current value for parameter
    ## @throws KeyError: if value is not currently cached
    def get(self, key):
        try:
            self.lock.acquire()
            return self.d[key]
        finally:
            self.lock.release()

_param_server_cache = None
## Get a handle on the client-wide parameter server cache
def get_param_server_cache():
    global _param_server_cache
    if _param_server_cache is None:
        _param_server_cache = ParamServerCache()        
    return _param_server_cache
