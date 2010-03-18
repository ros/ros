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
Additional ROS client API methods.
"""

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

# hide rospy.init implementation from users
def get_node_proxy():
    """
    Retrieve L{NodeProxy} for slave node running on this machine.

    @return: slave node API handle
    @rtype: L{rospy.NodeProxy}
    """
    return rospy.init.get_node_proxy()
    
def on_shutdown(h):
    """
    Register function to be called on shutdown. This function will be
    called before Node begins teardown.
    @param h: Function with zero args to be called on shutdown.
    @type  h: fn()
    """
    rospy.core.add_client_shutdown_hook(h)
    
def spin():
    """
    Blocks until ROS node is shutdown. Yields activity to other threads.
    @raise ROSInitException: if node is not in a properly initialized state
    """
    
    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())        
    try:
        while not rospy.core.is_shutdown():
            time.sleep(0.5)
    except KeyboardInterrupt:
        logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')

def myargv(argv=sys.argv):
    """
    Helper function for using sys.argv is ROS client libraries.
    @return: copy of sys.argv with ROS remapping arguments removed
    @rtype: [str]
    """
    import roslib.scriptutil
    return roslib.scriptutil.myargv(argv=argv)

def _init_node_params(argv, node_name):
    """
    Uploads private params to the parameter server. Private params are specified
    via command-line remappings.
    """

    # #1027: load in param name mappings
    import roslib.params
    params = roslib.params.load_command_line_node_params(argv)
    for param_name, param_value in params.iteritems():
        logdebug("setting param %s to %s"%(param_name, param_value))
        set_param(roslib.names.PRIV_NAME + param_name, param_value)

_init_node_args = None

def init_node(name, argv=sys.argv, anonymous=False, log_level=INFO, disable_rostime=False, disable_rosout=False, disable_signals=False):
    """
    Register client node with the master under the specified name.
    This MUST be called from the main Python thread unless
    disable_signals is set to True. Duplicate calls to init_node are
    only allowed if the arguments are identical as the side-effects of
    this method are not reversible.

    @param name: Node's name. This parameter must be a base name,
        meaning that it cannot contain namespaces (i.e. '/')
    @type  name: string
    
    @param argv: Command line arguments to this program. ROS reads
        these arguments to find renaming params. Defaults to sys.argv.
    @type  argv: [str]

    @param anonymous: if True, a name will be auto-generated for the
        node using name as the base.  This is useful when you wish to
        have multiple instances of the same node and don't care about
        their actual names (e.g. tools, guis). name will be used as
        the stem of the auto-generated name. NOTE: you cannot remap
        the name of an anonymous node.  @type anonymous: bool

    @param log_level: log level for sending message to /rosout, which
        is INFO by default. For convenience, you may use rospy.DEBUG,
        rospy.INFO, rospy.ERROR, rospy.WARN, rospy.FATAL,
    @type  log_level: int
    
    @param disable_signals: If True, rospy will not register its own
        signal handlers. You must set this flag if (a) you are unable
        to call init_node from the main thread and/or you are using
        rospy in an environment where you need to control your own
        signal handling (e.g. WX). If you set this to True, you should
        call rospy.signal_shutdown(reason) to initiate clean shutdown.
    @type  disable_signals: bool
    
    @param disable_rostime: for rostests only, suppresses
        automatic subscription to rostime
    @type  disable_rostime: bool

    @param disable_rosout: suppress auto-publication of rosout
    @type  disable_rostime: bool

    @raise ROSInitException: if initialization/registration fails
    @raise ValueError: if parameters are invalid (e.g. name contains a namespace or is otherwise illegal)
    """
    # this test can be eliminated once we change from warning to error in the next check
    if roslib.names.SEP in name:
        raise ValueError("namespaces are not allowed in node names")
    if not roslib.names.is_legal_base_name(name):
        import warnings
        warnings.warn("'%s' is not a legal ROS base name. This may cause problems with other ROS tools"%name, stacklevel=2)
    
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
        logdebug("signal handlers for rospy disabled")

    # check for name override
    mappings = rospy.names.get_mappings()
    if '__name' in mappings:
        # use roslib version of resolve_name to avoid remapping
        name = roslib.names.resolve_name(mappings['__name'], rospy.core.get_caller_id())
        if anonymous:
            logdebug("[%s] WARNING: due to __name setting, anonymous setting is being changed to false"%name)
            anonymous = False
        
    if anonymous:
        # not as good as a uuid/guid, but more readable. can't include
        # hostname as that is not guaranteed to be a legal ROS name
        name = "%s_%s_%s"%(name, os.getpid(), int(time.time()*1000))

    resolved_node_name = rospy.names.resolve_name(name)
    rospy.core.configure_logging(resolved_node_name)
    # #1810
    rospy.names.initialize_mappings(resolved_node_name)
    
    logger = logging.getLogger("rospy.client")
    logger.info("init_node, name[%s], pid[%s]", resolved_node_name, os.getpid())
            
    node = rospy.init.start_node(os.environ, resolved_node_name) #node initialization blocks until registration with master
    
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
    if not disable_rosout:
        rospy.rosout.init_rosout()
    logdebug("init_node, name[%s], pid[%s]", resolved_node_name, os.getpid())    
    if not disable_rostime:
        if not rospy.simtime.init_simtime():
            raise rospy.exceptions.ROSInitException("Failed to initialize time. Please check logs for additional details")
    else:
        rospy.rostime.set_rostime_initialized(True)

#_master_proxy is a MasterProxy wrapper
_master_proxy = None

def get_master(env=os.environ):
    """
    Get a remote handle to the ROS Master.
    This method can be called independent of running a ROS node,
    though the ROS_MASTER_URI must be declared in the environment.

    @return: ROS Master remote object
    @rtype: L{rospy.MasterProxy}
    @raise Exception: if server cannot be located or system cannot be
    initialized
    """
    global _master_proxy
    if _master_proxy is not None:
        return _master_proxy
    import roslib.rosenv
    _master_proxy = rospy.msproxy.MasterProxy(roslib.rosenv.get_master_uri())
    return _master_proxy

#########################################################
# Topic helpers

def get_published_topics(namespace='/'):
    """
    Retrieve list of topics that the master is reporting as being published.

    @return: List of topic names and types: [[topic1, type1]...[topicN, typeN]]
    @rtype: [[str, str]]
    """
    code, msg, val = get_master().getPublishedTopics(namespace)
    if code != 1:
        raise rospy.exceptions.ROSException("unable to get published topics: %s"%msg)
    return val

class _WFM(object):
    def __init__(self):
        self.msg = None
    def cb(self, msg):
        if self.msg is None:
            self.msg = msg
            
def wait_for_message(topic, topic_type, timeout=None):
    """
    Receive one message from topic.
    
    This will create a new subscription to the topic, receive one message, then unsubscribe.

    @param topic: name of topic
    @type  topic: str
    @param topic_type: topic type
    @type  topic_type: L{rospy.Message} class
    @param timeout: timeout time in seconds
    @type  timeout: double
    @return: Message
    @rtype: L{rospy.Message}
    @raise ROSException: if specified timeout is exceeded
    @raise ROSInterruptException: if shutdown interrupts wait
    """
    wfm = _WFM()
    s = None
    try:
        s = rospy.topics.Subscriber(topic, topic_type, wfm.cb)
        if timeout is not None:
            timeout_t = time.time() + timeout
            while not rospy.core.is_shutdown() and wfm.msg is None:
                time.sleep(0.01)
                if time.time() >= timeout_t:
                    raise rospy.exceptions.ROSException("timeout exceeded while waiting for message on topic %s"%topic)

        else:
            while not rospy.core.is_shutdown() and wfm.msg is None:
                time.sleep(0.01)            
    finally:
        if s is not None:
            s.unregister()
    if rospy.core.is_shutdown():
        raise rospy.exceptions.ROSInterruptException("rospy shutdown")
    return wfm.msg


#########################################################
# Service helpers

def wait_for_service(service, timeout=None):
    """
    Blocks until service is available. Use this in
    initialization code if your program depends on a
    service already running.
    @param service: name of service
    @type  service: str
    @param timeout: timeout time in seconds, None for no
    timeout. NOTE: timeout=0 is invalid as wait_for_service actually
    contacts the service, so non-blocking behavior is not
    possible. For timeout=0 uses cases, just call the service without
    waiting.
    @type  timeout: double
    @raise ROSException: if specified timeout is exceeded
    @raise ROSInterruptException: if shutdown interrupts wait
    """
    def contact_service(resolved_name, timeout=10.0):
        code, _, uri = master.lookupService(resolved_name)
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
                      'callerid' : rospy.core.get_caller_id(),
                      'service': resolved_name }
                roslib.network.write_ros_handshake_header(s, h)
                return True
            finally:
                if s is not None:
                    s.close()
    if timeout == 0.:
        raise ValueError("timeout must be non-zero")
    resolved_name = rospy.names.resolve_name(service)
    master = get_master()
    first = False
    if timeout:
        timeout_t = time.time() + timeout
        while not rospy.core.is_shutdown() and time.time() < timeout_t:
            try:
                if contact_service(resolved_name, timeout_t-time.time()):
                    return
                time.sleep(0.3)
            except KeyboardInterrupt:
                # re-raise
                rospy.core.logdebug("wait_for_service: received keyboard interrupt, assuming signals disabled and re-raising")
                raise 
            except: # service not actually up
                if first:
                    first = False
                    rospy.core.logerr("wait_for_service(%s): failed to contact [%s], will keep trying"%(resolved_name, uri))
        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("rospy shutdown")
        else:
            raise rospy.exceptions.ROSException("timeout exceeded while waiting for service %s"%resolved_name)
    else:
        while not rospy.core.is_shutdown():
            try:
                if contact_service(resolved_name):
                    return
                time.sleep(0.3)
            except KeyboardInterrupt:
                # re-raise
                rospy.core.logdebug("wait_for_service: received keyboard interrupt, assuming signals disabled and re-raising")
                raise 
            except: # service not actually up
                if first:
                    first = False
                    rospy.core.logerr("wait_for_service(%s): failed to contact [%s], will keep trying"%(resolved_name, uri))
        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("rospy shutdown")
    
    
#########################################################
# Param Server Access

_param_server = None
def _init_param_server():
    """
    Initialize parameter server singleton
    """
    global _param_server
    if _param_server is None:
        _param_server = get_master() #in the future param server will be a service
        
# class and singleton to distinguish whether or not user has passed us a default value
class _Unspecified(object): pass
_unspecified = _Unspecified()

def get_param(param_name, default=_unspecified):
    """
    Retrieve a parameter from the param server
    @param default: (optional) default value to return if key is not set
    @type  default: any
    @return: parameter value
    @rtype: XmlRpcLegalValue
    @raise ROSException: if parameter server reports an error
    @raise KeyError: if value not set and default is not given
    """
    try:
        _init_param_server()
        return _param_server[param_name] #MasterProxy does all the magic for us
    except KeyError:
        if default != _unspecified:
            return default
        else:
            raise

def get_param_names():
    """
    Retrieve list of parameter names.
    @return: parameter names
    @rtype: [str]
    @raise ROSException: if parameter server reports an error
    """
    _init_param_server()
    code, msg, val = _param_server.getParamNames() #MasterProxy does all the magic for us
    if code != 1:
        raise rospy.exceptions.ROSException("Unable to retrieve parameter names: %s"%msg)
    else:
        return val

def set_param(param_name, param_value):
    """
    Set a parameter on the param server
    @param param_name: parameter name
    @type  param_name: str
    @param param_value: parameter value
    @type  param_value: XmlRpcLegalValue
    @raise ROSException: if parameter server reports an error
    """
    # #2202
    if not roslib.names.is_legal_name(param_name):
        import warnings
        warnings.warn("'%s' is not a legal ROS graph resource name. This may cause problems with other ROS tools"%param_name, stacklevel=2)

    _init_param_server()
    _param_server[param_name] = param_value #MasterProxy does all the magic for us

def search_param(param_name):
    """
    Search for a parameter on the param server
    @param param_name: parameter name
    @type  param_name: str
    @return: key of matching parameter or None if no matching parameter. 
    @rtype: str
    @raise ROSException: if parameter server reports an error
    """
    _init_param_server()
    return _param_server.search_param(param_name)
    
def delete_param(param_name):
    """
    Delete a parameter on the param server
    @param param_name: parameter name
    @type  param_name: str
    @raise KeyError: if parameter is not set    
    @raise ROSException: if parameter server reports an error
    """    
    _init_param_server()
    del _param_server[param_name] #MasterProxy does all the magic for us

def has_param(param_name):
    """
    Test if parameter exists on the param server
    @param param_name: parameter name
    @type  param_name: str
    @raise ROSException: if parameter server reports an error
    """
    _init_param_server()
    return param_name in _param_server #MasterProxy does all the magic for us

################################################################################
# Time helpers

# these cannot go into rostime due to circular deps


class Rate(object):
    """
    Convenience class for sleeping in a loop at a specified rate
    """
    
    def __init__(self, hz):
        """
        Constructor.
        @param hz: hz rate to determine sleeping
        @type  hz: int
        """
        # #1403
        self.last_time = rospy.rostime.get_rostime()
        self.sleep_dur = rospy.rostime.Duration(0, int(1e9/hz))

    def sleep(self):
        """
        Attempt sleep at the specified rate. sleep() takes into
        account the time elapsed since the last successful
        sleep().
        
        @raise ROSInterruptException: if ROS time is set backwards or if
        ROS shutdown occurs before sleep completes
        """
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
def sleep(duration):
    """
    sleep for the specified duration in ROS time. If duration
    is negative, sleep immediately returns.
    
    @param duration: seconds (or rospy.Duration) to sleep
    @type  duration: float or Duration
    @raise ROSInterruptException: if ROS time is set backwards or if
    ROS shutdown occurs before sleep completes
    """
    if rospy.rostime.is_wallclock():
        if isinstance(duration, roslib.rostime.Duration):
            duration = duration.to_sec()
        if duration < 0:
            return
        else:
            time.sleep(duration)
    else:
        initial_rostime = rospy.rostime.get_rostime()
        if not isinstance(duration, roslib.rostime.Duration):
            duration = rospy.rostime.Duration.from_sec(duration)
        sleep_t = initial_rostime + duration

        rostime_cond = rospy.rostime.get_rostime_cond()

        # break loop if sleep_t is reached, time moves backwards, or
        # node is shutdown
        while rospy.rostime.get_rostime() < sleep_t and \
              rospy.rostime.get_rostime() >= initial_rostime and \
                  not rospy.core.is_shutdown():
            try:
                rostime_cond.acquire()
                rostime_cond.wait(0.5)
            finally:
                rostime_cond.release()

        if rospy.rostime.get_rostime() < initial_rostime:
            raise rospy.exceptions.ROSInterruptException("ROS time moved backwards")
        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("ROS shutdown request")

