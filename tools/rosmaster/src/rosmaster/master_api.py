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
ROS Master API. 

L{ROSMasterHandler} provides the API implementation of the
Master. Python allows an API to be introspected from a Python class,
so the handler has a 1-to-1 mapping with the actual XMLRPC API.

API return convention: (statusCode, statusMessage, returnValue)

 - statusCode: an integer indicating the completion condition of the method. 
 - statusMessage: a human-readable string message for debugging
 - returnValue: the return value of the method; method-specific.

Current status codes: 

 - -1: ERROR: Error on the part of the caller, e.g. an invalid parameter
 - 0: FAILURE: Method was attempted but failed to complete correctly.
 - 1: SUCCESS: Method completed successfully.

Individual methods may assign additional meaning/semantics to statusCode.
"""

import os
import sys
import logging
import threading
import time
import traceback

from rosgraph.xmlrpc import XmlRpcHandler

import rosgraph.names
from rosgraph.names import resolve_name
import rosmaster.paramserver
import rosmaster.threadpool

from rosmaster.util import xmlrpcapi
from rosmaster.registrations import RegistrationManager
from rosmaster.validators import non_empty, non_empty_str, not_none, is_api, is_topic, is_service, valid_type_name, valid_name, empty_or_valid_name, ParameterInvalid

NUM_WORKERS = 3 #number of threads we use to send publisher_update notifications

# Return code slots
STATUS = 0
MSG = 1
VAL = 2

_logger = logging.getLogger("rosmaster.master")

LOG_API = False

def mloginfo(msg, *args):
    """
    Info-level master log statements. These statements may be printed
    to screen so they should be user-readable.
    @param msg: Message string
    @type  msg: str
    @param args: arguments for msg if msg is a format string
    """
    #mloginfo is in core so that it is accessible to master and masterdata
    _logger.info(msg, *args)

def mlogwarn(msg, *args):
    """
    Warn-level master log statements. These statements may be printed
    to screen so they should be user-readable.
    @param msg: Message string
    @type  msg: str    
    @param args: arguments for msg if msg is a format string
    """
    #mloginfo is in core so that it is accessible to master and masterdata
    _logger.warn(msg, *args)
    if args:
        print "WARN: "+msg%args
    else:
        print "WARN: "+str(msg)


def apivalidate(error_return_value, validators=()):
    """
    ROS master/slave arg-checking decorator. Applies the specified
    validator to the corresponding argument and also remaps each
    argument to be the value returned by the validator.  Thus,
    arguments can be simultaneously validated and canonicalized prior
    to actual function call.
    @param error_return_value: API value to return if call unexpectedly fails
    @param validators: sequence of validators to apply to each
      arg. None means no validation for the parameter is required. As all
      api methods take caller_id as the first parameter, the validators
      start with the second param.
    @type  validators: sequence
    """
    def check_validates(f):
        assert len(validators) == f.func_code.co_argcount - 2, "%s failed arg check"%f #ignore self and caller_id
        def validated_f(*args, **kwds):
            if LOG_API:
                _logger.debug("%s%s", f.func_name, str(args[1:]))
                #print "%s%s"%(f.func_name, str(args[1:]))
            if len(args) == 1:
                _logger.error("%s invoked without caller_id paramter"%f.func_name)
                return -1, "missing required caller_id parameter", error_return_value
            elif len(args) != f.func_code.co_argcount:
                return -1, "Error: bad call arity", error_return_value

            instance = args[0]
            caller_id = args[1]
            if not isinstance(caller_id, basestring):
                _logger.error("%s: invalid caller_id param type", f.func_name)
                return -1, "caller_id must be a string", error_return_value
            
            newArgs = [instance, caller_id] #canonicalized args
            try:
                for (v, a) in zip(validators, args[2:]):
                    if v:
                        try:
                            newArgs.append(v(a, caller_id)) 
                        except ParameterInvalid, e:
                            _logger.error("%s: invalid parameter: %s", f.func_name, str(e) or 'error')
                            return -1, str(e) or 'error', error_return_value
                    else:
                        newArgs.append(a)

                if LOG_API:
                    retval = f(*newArgs, **kwds)
                    _logger.debug("%s%s returns %s", f.func_name, args[1:], retval)
                    return retval
                else:
                    code, msg, val = f(*newArgs, **kwds)
                    if val is None:
                        return -1, "Internal error (None value returned)", error_return_value
                    return code, msg, val
            except TypeError, te: #most likely wrong arg number
                _logger.error(traceback.format_exc())
                return -1, "Error: invalid arguments: %s"%te, error_return_value
            except Exception, e: #internal failure
                _logger.error(traceback.format_exc())
                return 0, "Internal failure: %s"%e, error_return_value
        validated_f.func_name = f.func_name
        validated_f.__doc__ = f.__doc__ #preserve doc
        return validated_f
    return check_validates

def publisher_update_task(api, topic, pub_uris):
    """
    Contact api.publisherUpdate with specified parameters
    @param api: XML-RPC URI of node to contact
    @type  api: str
    @param topic: Topic name to send to node
    @type  topic: str
    @param pub_uris: list of publisher APIs to send to node
    @type  pub_uris: [str]
    """
    
    mloginfo("publisherUpdate[%s] -> %s", topic, api)
    #TODO: check return value for errors so we can unsubscribe if stale
    xmlrpcapi(api).publisherUpdate('/master', topic, pub_uris)

def service_update_task(api, service, uri):
    """
    Contact api.serviceUpdate with specified parameters
    @param api: XML-RPC URI of node to contact
    @type  api: str
    @param service: Service name to send to node
    @type  service: str
    @param uri: URI to send to node        
    @type  uri: str
    """
    mloginfo("serviceUpdate[%s, %s] -> %s",service, uri, api)
    xmlrpcapi(api).serviceUpdate('/master', service, uri)

###################################################
# Master Implementation

class ROSMasterHandler(object):
    """
    XML-RPC handler for ROS master APIs.
    API routines for the ROS Master Node. The Master Node is a
    superset of the Slave Node and contains additional API methods for
    creating and monitoring a graph of slave nodes.

    By convention, ROS nodes take in caller_id as the first parameter
    of any API call.  The setting of this parameter is rarely done by
    client code as ros::msproxy::MasterProxy automatically inserts
    this parameter (see ros::client::getMaster()).
    """
    
    def __init__(self):
        """ctor."""

        self.uri = None
        self.done = False

        self.thread_pool = rosmaster.threadpool.MarkedThreadPool(NUM_WORKERS)
        # pub/sub/providers: dict { topicName : [publishers/subscribers names] }
        self.ps_lock = threading.Condition(threading.Lock())

        self.reg_manager = RegistrationManager(self.thread_pool)

        # maintain refs to reg_manager fields
        self.publishers  = self.reg_manager.publishers
        self.subscribers = self.reg_manager.subscribers
        self.services = self.reg_manager.services
        self.param_subscribers = self.reg_manager.param_subscribers
        
        self.topics_types = {} #dict { topicName : type }

        # parameter server dictionary
        self.param_server = rosmaster.paramserver.ParamDictionary(self.reg_manager)

    def _shutdown(self, reason=''):
        if self.thread_pool is not None:
            self.thread_pool.join_all(wait_for_tasks=False, wait_for_threads=False)
            self.thread_pool = None
        self.done = True
        
    def _ready(self, uri):
        """
        Initialize the handler with the XMLRPC URI. This is a standard callback from the XmlRpcNode API.

        @param uri: XML-RPC URI
        @type  uri: str
        """
        self.uri = uri

    def _ok(self):
        return not self.done
    
    ###############################################################################
    # EXTERNAL API

    @apivalidate(0, (None, ))
    def shutdown(self, caller_id, msg=''):
        """
        Stop this server
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param msg: a message describing why the node is being shutdown.
        @type  msg: str
        @return: [code, msg, 0]
        @rtype: [int, str, int]
        """
        if msg:
            print >> sys.stdout, "shutdown request: %s"%msg
        else:
            print >> sys.stdout, "shutdown requst"
        self._shutdown('external shutdown request from [%s]: %s'%(caller_id, msg))
        return 1, "shutdown", 0
        
    @apivalidate('')
    def getUri(self, caller_id):
        """
        Get the XML-RPC URI of this server.
        @param caller_id str: ROS caller id    
        @return [int, str, str]: [1, "", xmlRpcUri]
        """
        return 1, "", self.uri

        
    @apivalidate(-1)
    def getPid(self, caller_id):
        """
        Get the PID of this server
        @param caller_id: ROS caller id
        @type  caller_id: str
        @return: [1, "", serverProcessPID]
        @rtype: [int, str, int]
        """
        return 1, "", os.getpid()

    
    ################################################################
    # PARAMETER SERVER ROUTINES
    
    @apivalidate(0, (non_empty_str('key'),))
    def deleteParam(self, caller_id, key):
        """
        Parameter Server: delete parameter
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param key: parameter name
        @type  key: str
        @return: [code, msg, 0]
        @rtype: [int, str, int]
        """
        try:
            key = resolve_name(key, caller_id)
            self.param_server.delete_param(key, self._notify_param_subscribers)
            mloginfo("-PARAM [%s] by %s",key, caller_id)            
            return  1, "parameter %s deleted"%key, 0                
        except KeyError, e:
            return -1, "parameter [%s] is not set"%key, 0
        
    @apivalidate(0, (non_empty_str('key'), not_none('value')))
    def setParam(self, caller_id, key, value):
        """
        Parameter Server: set parameter.  NOTE: if value is a
        dictionary it will be treated as a parameter tree, where key
        is the parameter namespace. For example:::
          {'x':1,'y':2,'sub':{'z':3}}

        will set key/x=1, key/y=2, and key/sub/z=3. Furthermore, it
        will replace all existing parameters in the key parameter
        namespace with the parameters in value. You must set
        parameters individually if you wish to perform a union update.
        
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param key: parameter name
        @type  key: str
        @param value: parameter value.
        @type  value: XMLRPCLegalValue
        @return: [code, msg, 0]
        @rtype: [int, str, int]
        """
        key = resolve_name(key, caller_id)
        self.param_server.set_param(key, value, self._notify_param_subscribers)
        mloginfo("+PARAM [%s] by %s",key, caller_id)
        return 1, "parameter %s set"%key, 0

    @apivalidate(0, (non_empty_str('key'),))
    def getParam(self, caller_id, key):
        """
        Retrieve parameter value from server.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param key: parameter to lookup. If key is a namespace,
        getParam() will return a parameter tree.
        @type  key: str
        getParam() will return a parameter tree.

        @return: [code, statusMessage, parameterValue]. If code is not
            1, parameterValue should be ignored. If key is a namespace,
            the return value will be a dictionary, where each key is a
            parameter in that namespace. Sub-namespaces are also
            represented as dictionaries.
        @rtype: [int, str, XMLRPCLegalValue]
        """
        try:
            key = resolve_name(key, caller_id)
            return 1, "Parameter [%s]"%key, self.param_server.get_param(key)
        except KeyError, e: 
            return -1, "Parameter [%s] is not set"%key, 0

    @apivalidate(0, (non_empty_str('key'),))
    def searchParam(self, caller_id, key):
        """
        Search for parameter key on parameter server. Search starts in caller's namespace and proceeds
        upwards through parent namespaces until Parameter Server finds a matching key.

        searchParam's behavior is to search for the first partial match.
        For example, imagine that there are two 'robot_description' parameters::
          
           /robot_description
             /robot_description/arm
             /robot_description/base
           /pr2/robot_description
             /pr2/robot_description/base

        If I start in the namespace /pr2/foo and search for
        'robot_description', searchParam will match
        /pr2/robot_description. If I search for 'robot_description/arm'
        it will return /pr2/robot_description/arm, even though that
        parameter does not exist (yet).

        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param key: parameter key to search for.
        @type  key: str
        @return: [code, statusMessage, foundKey]. If code is not 1, foundKey should be
            ignored. 
        @rtype: [int, str, str]
        """
        search_key = self.param_server.search_param(caller_id, key)
        if search_key:
            return 1, "Found [%s]"%search_key, search_key
        else:
            return -1, "Cannot find parameter [%s] in an upwards search"%key, ''

    @apivalidate(0, (is_api('caller_api'), non_empty_str('key'),))
    def subscribeParam(self, caller_id, caller_api, key):
        """
        Retrieve parameter value from server and subscribe to updates to that param. See
        paramUpdate() in the Node API. 
        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param key: parameter to lookup.
        @type  key: str
        @param caller_api: API URI for paramUpdate callbacks.
        @type  caller_api: str
        @return: [code, statusMessage, parameterValue]. If code is not
           1, parameterValue should be ignored. parameterValue is an empty dictionary if the parameter
           has not been set yet.
        @rtype: [int, str, XMLRPCLegalValue]
        """
        key = resolve_name(key, caller_id)        
        try:
            # ps_lock has precedence and is required due to
            # potential self.reg_manager modification
            self.ps_lock.acquire()
            val = self.param_server.subscribe_param(key, (caller_id, caller_api))
        finally:
            self.ps_lock.release()
        return 1, "Subscribed to parameter [%s]"%key, val

    @apivalidate(0, (is_api('caller_api'), non_empty_str('key'),))
    def unsubscribeParam(self, caller_id, caller_api, key):
        """
        Retrieve parameter value from server and subscribe to updates to that param. See
        paramUpdate() in the Node API. 
        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param key: parameter to lookup.
        @type  key: str
        @param caller_api: API URI for paramUpdate callbacks.
        @type  caller_api: str
        @return: [code, statusMessage, numUnsubscribed]. 
           If numUnsubscribed is zero it means that the caller was not subscribed to the parameter.
        @rtype: [int, str, int]
        """        
        key = resolve_name(key, caller_id)        
        try:
            # ps_lock is required due to potential self.reg_manager modification
            self.ps_lock.acquire()
            retval = self.param_server.unsubscribe_param(key, (caller_id, caller_api))
        finally:
            self.ps_lock.release()
        return 1, "Unsubscribe to parameter [%s]"%key, 1


    @apivalidate(False, (non_empty_str('key'),))
    def hasParam(self, caller_id, key):
        """
        Check if parameter is stored on server. 
        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param key: parameter to check
        @type  key: str
        @return: [code, statusMessage, hasParam]
        @rtype: [int, str, bool]
        """
        key = resolve_name(key, caller_id)
        if self.param_server.has_param(key):
            return 1, key, True
        else:
            return 1, key, False            

    @apivalidate([])
    def getParamNames(self, caller_id):
        """
        Get list of all parameter names stored on this server.
        This does not adjust parameter names for caller's scope.
        
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: [code, statusMessage, parameterNameList]
        @rtype: [int, str, [str]]
        """
        return 1, "Parameter names", self.param_server.get_param_names()
            
    ##################################################################################
    # NOTIFICATION ROUTINES

    def _notify(self, registrations, task, key, value, node_apis):
        """
        Generic implementation of callback notification
        @param registrations: Registrations
        @type  registrations: L{Registrations}
        @param task: task to queue
        @type  task: fn
        @param key: registration key
        @type  key: str
        @param value: value to pass to task
        @type  value: Any
        """
        # cache thread_pool for thread safety
        thread_pool = self.thread_pool
        if not thread_pool:
            return
        
        try:            
            for node_api in node_apis:
                # use the api as a marker so that we limit one thread per subscriber
                thread_pool.queue_task(node_api, task, (node_api, key, value))
        except KeyError:
            _logger.warn('subscriber data stale (key [%s], listener [%s]): node API unknown'%(key, s))
        
    def _notify_param_subscribers(self, updates):
        """
        Notify parameter subscribers of new parameter value
        @param updates [([str], str, any)*]: [(subscribers, param_key, param_value)*]
        @param param_value str: parameter value
        """
        # cache thread_pool for thread safety
        thread_pool = self.thread_pool
        if not thread_pool:
            return

        for subscribers, key, value in updates:
            # use the api as a marker so that we limit one thread per subscriber
            for caller_id, caller_api in subscribers:
                self.thread_pool.queue_task(caller_api, self.param_update_task, (caller_id, caller_api, key, value))

    def param_update_task(self, caller_id, caller_api, param_key, param_value):
        """
        Contact api.paramUpdate with specified parameters
        @param caller_id: caller ID
        @type  caller_id: str
        @param caller_api: XML-RPC URI of node to contact
        @type  caller_api: str
        @param param_key: parameter key to pass to node
        @type  param_key: str
        @param param_value: parameter value to pass to node
        @type  param_value: str
        """
        mloginfo("paramUpdate[%s]", param_key)
        code, _, _ = xmlrpcapi(caller_api).paramUpdate('/master', param_key, param_value)
        if code == -1:
            try:
                # ps_lock is required due to potential self.reg_manager modification
                self.ps_lock.acquire()
                # reverse lookup to figure out who we just called
                matches = self.reg_manager.reverse_lookup(caller_api)
                for m in matches:
                    retval = self.param_server.unsubscribe_param(param_key, (m.id, caller_api))
            finally:
                self.ps_lock.release()

    def _notify_topic_subscribers(self, topic, pub_uris, sub_uris):
        """
        Notify subscribers with new publisher list
        @param topic: name of topic
        @type  topic: str
        @param pub_uris: list of URIs of publishers.
        @type  pub_uris: [str]
        """
        self._notify(self.subscribers, publisher_update_task, topic, pub_uris, sub_uris)

    ##################################################################################
    # SERVICE PROVIDER

    @apivalidate(0, ( is_service('service'), is_api('service_api'), is_api('caller_api')))
    def registerService(self, caller_id, service, service_api, caller_api):
        """
        Register the caller as a provider of the specified service.
        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param service: Fully-qualified name of service 
        @type  service: str
        @param service_api: Service URI 
        @type  service_api: str
        @param caller_api: XML-RPC URI of caller node 
        @type  caller_api: str
        @return: (code, message, ignore)
        @rtype: (int, str, int)
        """        
        try:
            self.ps_lock.acquire()
            self.reg_manager.register_service(service, caller_id, caller_api, service_api)
            mloginfo("+SERVICE [%s] %s %s", service, caller_id, caller_api)
        finally:
            self.ps_lock.release()
        return 1, "Registered [%s] as provider of [%s]"%(caller_id, service), 1

    @apivalidate(0, (is_service('service'),))
    def lookupService(self, caller_id, service):
        """
        Lookup all provider of a particular service.
        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param service: fully-qualified name of service to lookup.
        @type: service: str
        @return: (code, message, serviceUrl). service URL is provider's
           ROSRPC URI with address and port.  Fails if there is no provider.
        @rtype: (int, str, str)
        """
        try:
            self.ps_lock.acquire()
            service_url = self.services.get_service_api(service)
        finally:
            self.ps_lock.release()
        if service_url:
            return 1, "rosrpc URI: [%s]"%service_url, service_url
        else:
            return -1, "no provider", ''

    @apivalidate(0, ( is_service('service'), is_api('service_api')))
    def unregisterService(self, caller_id, service, service_api):
        """
        Unregister the caller as a provider of the specified service.
        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param service: Fully-qualified name of service
        @type  service: str
        @param service_api: API URI of service to unregister. Unregistration will only occur if current
           registration matches.
        @type  service_api: str
        @return: (code, message, numUnregistered). Number of unregistrations (either 0 or 1).
           If this is zero it means that the caller was not registered as a service provider.
           The call still succeeds as the intended final state is reached.
        @rtype: (int, str, int)
        """
        try:
            self.ps_lock.acquire()
            retval = self.reg_manager.unregister_service(service, caller_id, service_api)
            mloginfo("-SERVICE [%s] %s %s", service, caller_id, service_api)
            return retval
        finally:
            self.ps_lock.release()

    ##################################################################################
    # PUBLISH/SUBSCRIBE

    @apivalidate(0, ( is_topic('topic'), valid_type_name('topic_type'), is_api('caller_api')))
    def registerSubscriber(self, caller_id, topic, topic_type, caller_api):
        """
        Subscribe the caller to the specified topic. In addition to receiving
        a list of current publishers, the subscriber will also receive notifications
        of new publishers via the publisherUpdate API.        
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param topic str: Fully-qualified name of topic to subscribe to. 
        @param topic_type: Datatype for topic. Must be a package-resource name, i.e. the .msg name.
        @type  topic_type: str
        @param caller_api: XML-RPC URI of caller node for new publisher notifications
        @type  caller_api: str
        @return: (code, message, publishers). Publishers is a list of XMLRPC API URIs
           for nodes currently publishing the specified topic.
        @rtype: (int, str, [str])
        """
        #NOTE: subscribers do not get to set topic type
        try:
            self.ps_lock.acquire()
            self.reg_manager.register_subscriber(topic, caller_id, caller_api)

            # ROS 1.1: subscriber can now set type if it is not already set
            #  - don't let '*' type squash valid typing
            if not topic in self.topics_types and topic_type != rosgraph.names.ANYTYPE:
                self.topics_types[topic] = topic_type

            mloginfo("+SUB [%s] %s %s",topic, caller_id, caller_api)
            pub_uris = self.publishers.get_apis(topic)
        finally:
            self.ps_lock.release()
        return 1, "Subscribed to [%s]"%topic, pub_uris

    @apivalidate(0, (is_topic('topic'), is_api('caller_api')))
    def unregisterSubscriber(self, caller_id, topic, caller_api):
        """
        Unregister the caller as a publisher of the topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param topic: Fully-qualified name of topic to unregister.
        @type  topic: str
        @param caller_api: API URI of service to unregister. Unregistration will only occur if current
           registration matches.    
        @type  caller_api: str
        @return: (code, statusMessage, numUnsubscribed). 
          If numUnsubscribed is zero it means that the caller was not registered as a subscriber.
          The call still succeeds as the intended final state is reached.
        @rtype: (int, str, int)
        """
        try:
            self.ps_lock.acquire()
            retval = self.reg_manager.unregister_subscriber(topic, caller_id, caller_api)
            mloginfo("-SUB [%s] %s %s",topic, caller_id, caller_api)
            return retval
        finally:
            self.ps_lock.release()

    @apivalidate(0, ( is_topic('topic'), valid_type_name('topic_type'), is_api('caller_api')))
    def registerPublisher(self, caller_id, topic, topic_type, caller_api):
        """
        Register the caller as a publisher the topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param topic: Fully-qualified name of topic to register.
        @type  topic: str
        @param topic_type: Datatype for topic. Must be a
        package-resource name, i.e. the .msg name.
        @type  topic_type: str
        @param caller_api str: ROS caller XML-RPC API URI
        @type  caller_api: str
        @return: (code, statusMessage, subscriberApis).
        List of current subscribers of topic in the form of XMLRPC URIs.
        @rtype: (int, str, [str])
        """
        #NOTE: we need topic_type for getPublishedTopics.
        try:
            self.ps_lock.acquire()
            self.reg_manager.register_publisher(topic, caller_id, caller_api)
            # don't let '*' type squash valid typing
            if topic_type != rosgraph.names.ANYTYPE or not topic in self.topics_types:
                self.topics_types[topic] = topic_type
            pub_uris = self.publishers.get_apis(topic)
            sub_uris = self.subscribers.get_apis(topic)
            self._notify_topic_subscribers(topic, pub_uris, sub_uris)
            mloginfo("+PUB [%s] %s %s",topic, caller_id, caller_api)
            sub_uris = self.subscribers.get_apis(topic)            
        finally:
            self.ps_lock.release()
        return 1, "Registered [%s] as publisher of [%s]"%(caller_id, topic), sub_uris


    @apivalidate(0, (is_topic('topic'), is_api('caller_api')))
    def unregisterPublisher(self, caller_id, topic, caller_api):
        """
        Unregister the caller as a publisher of the topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param topic: Fully-qualified name of topic to unregister.
        @type  topic: str
        @param caller_api str: API URI of service to
           unregister. Unregistration will only occur if current
           registration matches.
        @type  caller_api: str
        @return: (code, statusMessage, numUnregistered). 
           If numUnregistered is zero it means that the caller was not registered as a publisher.
           The call still succeeds as the intended final state is reached.
        @rtype: (int, str, int)
        """            
        try:
            self.ps_lock.acquire()
            retval = self.reg_manager.unregister_publisher(topic, caller_id, caller_api)
            if retval[VAL]:
                self._notify_topic_subscribers(topic, self.publishers.get_apis(topic), self.subscribers.get_apis(topic))
            mloginfo("-PUB [%s] %s %s",topic, caller_id, caller_api)
        finally:
            self.ps_lock.release()
        return retval

    ##################################################################################
    # GRAPH STATE APIS

    @apivalidate('', (valid_name('node'),))
    def lookupNode(self, caller_id, node_name):
        """
        Get the XML-RPC URI of the node with the associated
        name/caller_id.  This API is for looking information about
        publishers and subscribers. Use lookupService instead to lookup
        ROS-RPC URIs.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param node: name of node to lookup
        @type  node: str
        @return: (code, msg, URI)
        @rtype: (int, str, str)
        """
        try:
            self.ps_lock.acquire()
            node = self.reg_manager.get_node(node_name)
            if node is not None:
                retval = 1, "node api", node.api
            else:
                retval = -1, "unknown node [%s]"%node_name, ''
        finally:
            self.ps_lock.release()
        return retval
        
    @apivalidate(0, (empty_or_valid_name('subgraph'),))
    def getPublishedTopics(self, caller_id, subgraph):
        """
        Get list of topics that can be subscribed to. This does not return topics that have no publishers.
        See L{getSystemState()} to get more comprehensive list.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param subgraph: Restrict topic names to match within the specified subgraph. Subgraph namespace
           is resolved relative to the caller's namespace. Use '' to specify all names.
        @type  subgraph: str
        @return: (code, msg, [[topic1, type1]...[topicN, typeN]])
        @rtype: (int, str, [[str, str],])
        """
        try:
            self.ps_lock.acquire()
            # force subgraph to be a namespace with trailing slash
            if subgraph and subgraph[-1] != rosgraph.names.SEP:
                subgraph = subgraph + rosgraph.names.SEP
            #we don't bother with subscribers as subscribers don't report topic types. also, the intended
            #use case is for subscribe-by-topic-type
            retval = [[t, self.topics_types[t]] for t in self.publishers.iterkeys() if t.startswith(subgraph)]
        finally:
            self.ps_lock.release()
        return 1, "current topics", retval
    
    @apivalidate([])
    def getTopicTypes(self, caller_id): 
        """
        Retrieve list topic names and their types.
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @rtype: (int, str, [[str,str]] )
        @return: (code, statusMessage, topicTypes). topicTypes is a list of [topicName, topicType] pairs.
        """
        try: 
            self.ps_lock.acquire()
            retval = self.topics_types.items()
        finally:
            self.ps_lock.release()
        return 1, "current system state", retval

    @apivalidate([[],[], []])
    def getSystemState(self, caller_id): 
        """
        Retrieve list representation of system state (i.e. publishers, subscribers, and services).
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @rtype: (int, str, [[str,[str]], [str,[str]], [str,[str]]])
        @return: (code, statusMessage, systemState).

           System state is in list representation::
             [publishers, subscribers, services].
        
           publishers is of the form::
             [ [topic1, [topic1Publisher1...topic1PublisherN]] ... ]
        
           subscribers is of the form::
             [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
        
           services is of the form::
             [ [service1, [service1Provider1...service1ProviderN]] ... ]
        """
        edges = []
        try: 
            self.ps_lock.acquire()
            retval = [r.get_state() for r in (self.publishers, self.subscribers, self.services)]
        finally:
            self.ps_lock.release()
        return 1, "current system state", retval
