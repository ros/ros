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

## rospy client API 

import logging
import os
import socket
import struct
import sys
import time
import random

import roslib.msg
import roslib.names
import roslib.network

import rospy.core
from rospy.core import logwarn, loginfo, logerr, logdebug
import rospy.exceptions
import rospy.init
import rospy.names
import rospy.rosout 
import rospy.rostime
import rospy.simtime

TIMEOUT_READY = 15.0 #seconds

# log level constants
DEBUG = roslib.msg.Log.DEBUG
INFO = roslib.msg.Log.INFO
WARN = roslib.msg.Log.WARN
ERROR = roslib.msg.Log.ERROR
FATAL = roslib.msg.Log.FATAL

## \ingroup clientapi Client API
## Register function to be called on shutdown. This function will be
## called before Node begins teardown.
## @param h fn(): Function with zero args to be called on shutdown.
def on_shutdown(h):
    # wrap function to strip \a reason argument that gets passed in for internal use
    def wrapper(reason):
        h()
    rospy.core.add_preshutdown_hook(wrapper)
    
## \ingroup clientapi Client API
## blocks until ROS node is shutdown. Yields activity to other threads.
## @throws ROSInitException if node is not in a properly initialized state
def spin():
    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())        
    try:
        while not rospy.core.is_shutdown():
            time.sleep(0.5)
    except KeyboardInterrupt:
        logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')

## \ingroup clientapi
## @return [str]: copy of sys.argv with ROS remapping arguments removed
def myargv(argv=sys.argv):
    return [a for a in argv if not rospy.names.REMAP in a]

## Uploads private params to the parameter server. Private params are specified
## via command-line remappings. 
def _init_node_params(argv, node_name):
    # #1027: load in param name mappings
    import roslib.params
    params = roslib.params.load_command_line_node_params(argv)
    for param_name, param_value in params.iteritems():
        logdebug("setting param %s to %s"%(param_name, param_value))
        set_param(roslib.names.PRIV_NAME + param_name, param_value)

_init_node_args = None

## \ingroup clientapi
## Register client node with the master under the specified name.
## This should be called after Pub/Sub topics have been declared and
## it MUST be called from the main Python thread unless \a
## disable_signals is set to True. Duplicate calls to init_node are
## only allowed if the arguments are identical as the side-effects of
## this method are not reversible.
##
## @param name
## @param argv Command line arguments to this program. ROS reads
## these arguments to find renaming params. Defaults to sys.argv.
## @param anonymous bool: if True, a name will be auto-generated for
##   the node using \a name as the base.  This is useful when you
##   wish to have multiple instances of the same node and don't care
##   about their actual names (e.g. tools, guis). \a name will be
##   used as the stem of the auto-generated name. NOTE: you cannot
##   remap the name of an anonymous node.
## @param log_level int: log level for sending message to /rosout,
##   which is INFO by default. For convenience, you may use
##   rospy.DEBUG, rospy.INFO, rospy.ERROR, rospy.WARN, rospy.FATAL,
## @param disable_signals bool: If True, rospy will not register its
##   own signal handlers. You must set this flag if (a) you are unable
##   to call init_node from the main thread and/or you are using rospy
##   in an environment where you need to control your own signal
##   handling (e.g. WX).
## @param disable_rostime bool: for rostests only, suppresses
## automatic subscription to rostime
## @throws ROSInitException if initialization/registration fails
def init_node(name, argv=sys.argv, anonymous=False, log_level=INFO, disable_rostime=False, disable_signals=False):
    global _init_node_args

    # #972: allow duplicate init_node args if calls are identical
    # NOTE: we don't bother checking for node name aliases (e.g. 'foo' == '/foo').
    if _init_node_args:
        if _init_node_args != (name, argv, anonymous, log_level, disable_rostime, disable_signals):
            raise rospy.exceptions.ROSException("rospy.init_node() has already been called with different arguments: "+str(_init_node_args))
        else:
            return #already initialized
    _init_node_args = (name, argv, anonymous, log_level, disable_rostime, disable_signals)
        
    if not disable_signals:
        # NOTE: register_signals must be called from main thread
        rospy.core.register_signals() # add handlers for SIGINT/etc...
    else:
        logwarn("signal handlers for rospy disabled")

    # check for name override
    name_remap = rospy.names.resolve_name('__name', '/')
    if name_remap != '/__name':
        # re-resolve, using actual namespace
        name = rospy.names.resolve_name('__name')
        if anonymous:
            logwarn("[%s] WARNING: due to __name setting, anonymous setting is being changed to false"%name)
            anonymous = False
        
    if anonymous:
        # not as good as a uuid/guid, but more readable
        name = "%s-%s-%s-%s"%(name, socket.gethostname(), os.getpid(), int(time.time()*1000))

    node_name = rospy.names.resolve_name(name)
    rospy.core.configure_logging(node_name)
    
    logger = logging.getLogger("rospy.client")
    logger.info("init_node, name[%s], pid[%s]", node_name, os.getpid())
            
    node = rospy.init.start_node(os.environ, name=name) #node initialization blocks until registration with master
    
    timeout_t = time.time() + TIMEOUT_READY
    code = None
    while time.time() < timeout_t and code is None and not rospy.core.is_shutdown():
        try:
            code, msg, master_uri = node.getMasterUri()
        except:
            time.sleep(0.01) #poll for init

    if rospy.core.is_shutdown():
        logger.warn("aborting node initialization as shutdown has been triggered")
        raise rospy.exceptions.ROSInitException("init_node interrupted before it could complete")

    # upload private params (set via command-line) to parameter server
    _init_node_params(argv, name)

    rospy.core.set_initialized(True)
    if code is None:
        logger.error("ROS node initialization failed: unable to connect to local node") 
        raise rospy.exceptions.ROSInitException("ROS node initialization failed: unable to connect to local node")        
    elif code != 1:
        logger.error("ROS node initialization failed: %s, %s, %s", code, msg, master_uri)
        raise rospy.exceptions.ROSInitException("ROS node initialization failed: %s, %s, %s", code, msg, master_uri)

    rospy.rosout.load_rosout_handlers(log_level)
    rospy.rosout.init_rosout()
    logdebug("init_node, name[%s], pid[%s]", node_name, os.getpid())    
    if not disable_rostime:
        if not rospy.simtime.init_simtime():
            raise rospy.exceptions.ROSInitException("Failed to initialize time. Please check logs for additional details")
    else:
        rospy.rostime.set_rostime_initialized(True)

## #503
## @deprecated 
ready = init_node

#_master_proxy is a MasterProxy wrapper
_master_proxy = None

## \ingroup clientapi 
# Get a remote handle to the ROS Master. This method can be called
# independent of running a ROS node, though the ROS_MASTER_URI must be
# declared in the environment.
#
# @return MasterProxy: ROS Master remote object
# @throws Exception if server cannot be located or system cannot be
# initalized
def get_master(env=os.environ):
    global _master_proxy
    if _master_proxy is not None:
        return _master_proxy
    # check against local interpreter plus global env
    import roslib.rosenv
    master_uri = rospy.init.get_local_master_uri() or roslib.rosenv.get_master_uri()
    _master_proxy = rospy.msproxy.MasterProxy(master_uri)
    return _master_proxy

#########################################################
# Topic helpers

## \ingroup clientapi 
def get_published_topics(namespace='/'):
    code, msg, val = get_master().getPublishedTopics(namespace)
    if code != 1:
        raise rospy.exceptions.ROSException("unable to get published topics: %s"%msg)
    return val

    
#########################################################
# Service helpers

## \ingroup clientapi 
## Blocks until service is available. Use this in
## initialization code if your program depends on a
## service already running.
## @param service str: name of service
## @param timeout double: timeout time in seconds
## @throws ROSException if specified \a timeout is exceeded
def wait_for_service(service, timeout=None):
    def contact_service(service, timeout=10.0):
        code, _, uri = master.lookupService(service)
        if False and code == 1:
            return True
        elif True and code == 1:
            # disabling for now as this is causing socket.error 22 "Invalid argument" on OS X
            addr = rospy.core.parse_rosrpc_uri(uri)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)            
            try:
                # we always want to timeout just in case we're connecting
                # to a down service.
                s.settimeout(timeout)
                s.connect(addr)
                h = { 'probe' : '1', 'md5sum' : '*',
                      'callerid' : rospy.names.get_caller_id(),
                      'service': service }
                roslib.network.write_ros_handshake_header(s, h)
                return True
            finally:
                if s is not None:
                    s.close()

    service = rospy.names.resolve_name(service)
    master = get_master()
    first = False
    if timeout:
        timeout_t = time.time() + timeout
        while not rospy.core.is_shutdown() and time.time() < timeout_t:
            try:
                if contact_service(service, timeout_t-time.time()):
                    return
                time.sleep(0.3)
            except: # service not actually up
                if first:
                    first = False
                    rospy.core.logerr("wait_for_service(%s): failed to contact [%s], will keep trying"%(service, uri))
        raise rospy.exceptions.ROSException("timeout exceeded while waiting for service %s"%service)
    else:
        while not rospy.core.is_shutdown():
            try:
                if contact_service(service):
                    return
                time.sleep(0.3)
            except: # service not actually up
                if first:
                    first = False
                    rospy.core.logerr("wait_for_service(%s): failed to contact [%s], will keep trying"%(service, uri))
    
#########################################################
# Param Server Access

_param_server = None
## @internal
## Initialize parameter server singleton
def _init_param_server():
    global _param_server
    if _param_server is None:
        _param_server = get_master() #in the future param server will be a service
        
# class and singleton to distinguish whether or not user has passed us a default value
class _Unspecified(object): pass
_unspecified = _Unspecified()

## \ingroup clientapi 
## Retrieve a parameter from the param server
## @param default: (optional)default value to return if key is not set
## @return XmlRpcLegalValue: parameter value
## @throws ROSException if parameter server reports an error
## @throws KeyError if value not set and default is not given
def get_param(param_name, default=_unspecified):
    try:
        _init_param_server()
        return _param_server[param_name] #MasterProxy does all the magic for us
    except KeyError:
        if default != _unspecified:
            return default
        else:
            raise

## \ingroup clientapi 
## Retrieve list of parameter names
## @return [str]: parameter names
## @throws ROSException if parameter server reports an error
def get_param_names():
    _init_param_server()
    code, msg, val = _param_server.getParamNames() #MasterProxy does all the magic for us
    if code != 1:
        raise rospy.exceptions.ROSException("Unable to retrieve parameter names: %s"%msg)
    else:
        return val

## \ingroup clientapi 
## Set a parameter on the param server
## @param param_name str: parameter name
## @param param_value XmlRpcLegalValue: parameter value
## @throws ROSException if parameter server reports an error
def set_param(param_name, param_value):
    _init_param_server()
    _param_server[param_name] = param_value #MasterProxy does all the magic for us

## \ingroup clientapi 
## Search for a parameter on the param server
## @param param_name str: parameter name
## @throws ROSException if parameter server reports an error
def search_param(param_name):
    _init_param_server()
    return _param_server.search_param(param_name)
    
## \ingroup clientapi 
## Delete a parameter on the param server
## @param param_name str: parameter name
## @throws ROSException if parameter server reports an error
def delete_param(param_name):
    _init_param_server()
    del _param_server[param_name] #MasterProxy does all the magic for us

## \ingroup clientapi 
## Test if parameter exists on the param server
## @param param_name str: parameter name
## @throws ROSException if parameter server reports an error
def has_param(param_name):
    _init_param_server()
    return param_name in _param_server #MasterProxy does all the magic for us

################################################################################
# Time helpers

# these cannot go into rostime due to circular deps

## Convenience class for sleeping in a loop at a specified rate
class Rate(object):
    ## @param hz int: hz rate to determine sleeping
    def __init__(self, hz):
        # #1403
        self.last_time = rospy.rostime.get_rostime()
        self.sleep_dur = rospy.rostime.Duration(0, int(1e9/hz))

    def sleep(self):
        curr_time = rospy.rostime.get_rostime()
        # detect time jumping backwards
        if self.last_time > curr_time:
            self.last_time = curr_time

        # calculate sleep interval
        elapsed = curr_time - self.last_time
        sleep(self.sleep_dur - elapsed)
        self.last_time = self.last_time + self.sleep_dur

        # detect time jumping forwards, as well as loops that are
        # inherently too slow
        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time

# TODO: may want more specific exceptions for sleep
## \ingroup clientapi
## sleep for the specified \a duration in ROS time. If \a duration
## is negative, sleep immediately returns.
## @param duration float or Duration: seconds (or rospy.Duration) to sleep
## @throws ROSInterruptException if ROS time is set backwards or if
## ROS shutdown occurs before sleep completes
def sleep(duration):
    # make a copy of current rostime
    initial_rostime = rospy.rostime.get_rostime()
    if initial_rostime is None:
        if isinstance(duration, roslib.rostime.Duration):
            duration = duration.to_seconds()
        if duration < 0:
            return
        else:
            time.sleep(duration)
    else:
        if not isinstance(duration, roslib.rostime.Duration):
            duration = rospy.rostime.Duration.from_seconds(duration)
        sleep_t = initial_rostime + duration
        
        rostime_cond = rospy.rostime.get_rostime_cond()

        # break loop if sleep_t is reached, time moves backwards, or
        # node is shutdown
        while rospy.rostime.get_rostime() < sleep_t and \
              rospy.rostime.get_rostime() >= initial_rostime and \
                  not rospy.core.is_shutdown():
            try:
                rostime_cond.acquire()
                rostime_cond.wait(0.001)
            finally:
                rostime_cond.release()

        if rospy.rostime.get_rostime() < initial_rostime:
            raise rospy.exceptions.ROSInterruptException("ROS time moved backwards")
        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("ROS shutdown request")

