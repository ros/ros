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

import atexit
import logging
import os
import signal
import string
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
from rospy.validators import ParameterInvalid

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
# ROSRPC

ROSRPC = "rosrpc://"

## utility function for parsing ROS-RPC URIs
## @param uri str: ROSRPC URI
## @return (str, int): address, port
## @throws ParameterInvalid if \a uri is not a valid ROSRPC URI
def parse_rosrpc_uri(uri):
    if uri.startswith(ROSRPC):
        dest_addr = uri[len(ROSRPC):]            
    else:
        raise ParameterInvalid("Invalid protocol for ROS service URL: %s"%uri)
    try:
        if '/' in dest_addr:
            dest_addr = dest_addr[:dest_addr.find('/')]
        dest_addr, dest_port = dest_addr.split(':')
        dest_port = string.atoi(dest_port)
    except:
        raise ParameterInvalid("ROS service URL is invalid: %s"%uri)
    return dest_addr, dest_port

#########################################################
        
def _stdout_handler(msg):
    sys.stdout.write(str(msg)+'\n')
def _stderr_handler(msg):
    sys.stderr.write(str(msg)+'\n')

## client logger
_clogger = logging.getLogger("rospy.rosout")

_logdebug_handlers = [_clogger.debug]
_loginfo_handlers = [_clogger.info, _stdout_handler]
_logwarn_handlers = [_clogger.warn]
_logerr_handlers = [_clogger.error, _stderr_handler]
_logfatal_handlers = [_clogger.critical, _stderr_handler]

## Add handler for specified level
## @param level int: log level (use constants from roslib.msg.Log)
## @param h fn: log message handler
## @raise ROSInternalException if \a level is invalid
def add_log_handler(level, h):
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
    
## \ingroup clientapi Client API
## Log a debug message to the /rosout topic
def logdebug(msg, *args):
    if args:
        msg = msg%args
    for h in _logdebug_handlers:
        h(msg)
## \ingroup clientapi Client API
## Log a warning message to the /rosout topic    
def logwarn(msg, *args):
    if args:
        msg = msg%args
    for h in _logwarn_handlers:
        h(msg)
## \ingroup clientapi Client API
## Log an info message to the /rosout topic    
def loginfo(msg, *args):
    if args:
        msg = msg%args
    for h in _loginfo_handlers:
        h(msg)
logout = loginfo # alias deprecated name
## \ingroup clientapi Client API
## Log an error message to the /rosout topic    
def logerr(msg, *args):
    if args:
        msg = msg%args
    for h in _logerr_handlers:
        h(msg)
logerror = logerr # alias logerr
## \ingroup clientapi Client API
## Log an error message to the /rosout topic    
def logfatal(msg, *args):
    if args:
        msg = msg%args
    for h in _logfatal_handlers:
        h(msg)

#########################################################
# CONSTANTS

MASTER_NAME = "master" #master is a reserved node name for the central master

# Return code slots
STATUS = 0
MSG = 1
VAL = 2

## \ingroup clientapi
def get_ros_root(env=os.environ, require=False):
    rosRoot = env.get(roslib.rosenv.ROS_ROOT, None)
    if require and not rosRoot:
        raise rospy.exceptions.ROSException('%s is not set'%roslib.rosenv.ROS_ROOT)
    return rosRoot


#########################################################
# API

_uri = None
## \ingroup clientapi
def get_node_uri():
    return _uri

## set the URI of the local node 
def set_node_uri(uri):
    global _uri
    _uri = uri

#########################################################
# Logging

_log_filename = None
def configure_logging(node_name):
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
    _log_filename = roslib.roslogging.configure_logging('rospy', logging.DEBUG, filename=filename, additional=['roslib'])

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
_preshutdown_hooks = []

_signalChain = {}

## \ingroup clientapi Client API
## @return bool: True if shutdown flag has been set
def is_shutdown():
    return _shutdown_flag

## @internal
## shared implementation of add_shutdown_hook and add_preshutdown_hook
def _add_shutdown_hook(h, hooks):
    if type(h) not in [types.FunctionType, types.MethodType]:
        raise TypeError("shutdown hook [%s] must be a function: %s"%(h, type(h)))
    if _shutdown_flag:
        _logger.warn("add_shutdown_hook called after shutdown")
        h("already shutdown")
        return
    try:
        _shutdown_lock.acquire()
        if hooks is None:
            # race condition check, don't log as we are deep into shutdown
            return
        hooks.append(h)
    finally:
        _shutdown_lock.release()

## @internal
## Add method to invoke when system shuts down. Unlike add_shutdown_hook, these
## methods will be called before any other shutdown hooks.
## @param h fn: function that takes in a single string argument (shutdown reason)
def add_preshutdown_hook(h):
    _add_shutdown_hook(h, _preshutdown_hooks)

## @internal
## Add method to invoke when system shuts down. Shutdown hooks are called in the order
## that they are registered. This is an internal API 
## method that is used to cleanup. See the client on_shutdown() method if you wish to register a
## shutdown method.
## @param h fn: function that takes in a single string argument (shutdown reason)
def add_shutdown_hook(h):
    _add_shutdown_hook(h, _shutdown_hooks)

## \ingroup clientapi
## signal objects waiting on _shutdown_lock. Also invoke shutdown hooks
## @param reason str: human-readable shutdown reason, if applicable
def signal_shutdown(reason):
    global _shutdown_flag, _shutdown_lock, _shutdown_hooks
    _logger.info("signal_shutdown [%s]"%reason)
    if not _shutdown_flag:
        _shutdown_lock.acquire()
        if _shutdown_flag:
            return
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
        _shutdown_lock.release()
        
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
    def validator(param_value, callerId):
        if not param_value or not isinstance(param_value, basestring):
            raise ParameterInvalid("ERROR: parameter [%s] is not an XMLRPC URI"%paramName)
        if not param_value.startswith("http://") and not param_value.startswith(ROSRPC):
            raise ParameterInvalid("ERROR: parameter [%s] is not an RPC URI"%paramName)
        #could do more fancy parsing, but the above catches the major cases well enough
        return param_value
    return validator

## \ingroup validators    
## Validator that checks that parameter is a valid ROS topic name
def is_topic(param_name):
    def validator(param_value, caller_id):
        v = valid_name_validator_resolved(param_name, param_value, caller_id)
        if param_value == '/':
            raise ParameterInvalid("ERROR: parameter [%s] cannot be the global namespace"%param_name)            
        return v
    return validator

## \ingroup validators
## Validator that checks that parameter is a valid ROS service name
def is_service(param_name):
    def validator(param_value, caller_id):
        v = valid_name_validator_resolved(param_name, param_value, caller_id)
        if param_value == '/':
            raise ParameterInvalid("ERROR: parameter [%s] cannot be the global namespace"%param_name)            
        return v
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

