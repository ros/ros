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
Internal use: ROS Master and Slave API. 

The APIs are implemented by X{ROSHandler} and its subclass
X{ROSMasterHandler}. With this implementation, a Master also includes
the entire Slave API.

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
import string
import logging
import random
import socket
import thread
import threading
import time
import urlparse

from roslib.xmlrpc import XmlRpcHandler

import rospy.names
import rospy.paramserver
import rospy.rostime
import rospy.tcpros
import rospy.threadpool

from rospy.core import *
from rospy.masterdata import RegistrationManager
from rospy.registration import RegManager, get_topic_manager
from rospy.validators import non_empty, non_empty_str, not_none, ParameterInvalid

NUM_WORKERS = 3 #number of threads we use to send publisher_update notifications


# pseudo-validators ###############################
# these validators actually return tuples instead of a function and it is up to a custom
# validator on the class itself to perform the validation
def is_publishers_list(paramName):
    return ('is_publishers_list', paramName)

_logger = logging.getLogger("rospy.masterslave")

LOG_API = True

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
                            #simultaneously validate + canonicalized args
                            if type(v) == list or type(v) == tuple:
                                newArgs.append(instance._custom_validate(v[0], v[1], a, caller_id))
                            else:
                                newArgs.append(v(a, caller_id)) 
                        except ParameterInvalid, e:
                            _logger.error("%s: invalid parameter: %s", f.func_name, e.message or 'error')
                            return -1, e.message or 'error', error_return_value
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


class ROSHandler(XmlRpcHandler):
    """
    Base handler for both slave and master nodes. API methods
    generally provide the capability for establishing point-to-point
    connections with other nodes.
    
    Instance methods are XML-RPC API methods, so care must be taken as
    to what is added here. 
    """
    
    def __init__(self, name, master_uri, is_slave=True):
        """
        Base constructor for ROS nodes/masters
        @param name: ROS name of this node
        @type  name: str
        @param master_uri: URI of master node, or None if this node is the master
        @type  master_uri: str
        @param is_slave: True if this is a slave node, False if master
        @type  is_slave: bool
        """
        super(ROSHandler, self).__init__()
        self.masterUri = master_uri
        self.name = name
        self.uri = None
        self.done = False

        # initialize protocol handlers. The master will not have any.
        self.protocol_handlers = []
        handler = rospy.tcpros.get_tcpros_handler()
        if handler is not None:
            self.protocol_handlers.append(handler)
            
        if is_slave:
            self.reg_man = RegManager(self)
        else:
            self.reg_man = None

    ###############################################################################
    # INTERNAL 

    def _is_registered(self):
        """
        @return: True if slave API is registered with master.
        @rtype: bool
        """
        if self.reg_man is None:
            return False
        else:
            return self.reg_man.is_registered()
        

    def _ready(self, uri):
        """
        @param uri: XML-RPC URI
        @type  uri: str
        callback from ROSNode to inform handler of correct i/o information
        """
        _logger.info("_ready: %s", uri)
        self.uri = uri
        #connect up topics in separate thread
        if self.reg_man:
            import threading
            t = threading.Thread(target=self.reg_man.start, args=(uri, self.masterUri))
            rospy.core._add_shutdown_thread(t)
            t.start()

    def _custom_validate(self, validation, param_name, param_value, caller_id):
        """
        Implements validation rules that require access to internal ROSHandler state.
        @param validation: name of validation rule to use
        @type  validation: str
        @param param_name: name of parameter being validated
        @type  param_name: str
        @param param_value str: value of parameter
        @type  param_value: str
        @param caller_id: value of caller_id parameter to API method
        @type  caller_id: str
        @raise ParameterInvalid: if the parameter does not meet validation
        @return: new value for parameter, after validation
        """
        if validation == 'is_publishers_list':
            if not type(param_value) == list:
                raise ParameterInvalid("ERROR: param [%s] must be a list"%param_name)
            for v in param_value:
                if not isinstance(v, basestring):
                    raise ParameterInvalid("ERROR: param [%s] must be a list of strings"%param_name)
                parsed = urlparse.urlparse(v)
                if not parsed[0] or not parsed[1]: #protocol and host
                    raise ParameterInvalid("ERROR: param [%s] does not contain valid URLs [%s]"%(param_name, v))
            return param_value
        else:
            raise ParameterInvalid("ERROR: param [%s] has an unknown validation type [%s]"%(param_name, validation))

    ## static map for tracking which arguments to a function should be remapped
    #  { methodName : [ arg indices ]
    _remap_table = { } 

    @classmethod
    def remappings(cls, methodName):
        """
        @internal
        @param cls: class to register remappings on
        @type  cls: Class: class to register remappings on    
        @return: parameters (by pos) that should be remapped because they are names
        @rtype: list
        """
        if methodName in cls._remap_table:
            return cls._remap_table[methodName]
        else:
            return []
    
    ###############################################################################
    # UNOFFICIAL/PYTHON-ONLY API

    @apivalidate('')
    ## (Python-Only API) Get the XML-RPC URI of this server
    ## @param self
    ## @param caller_id str: ROS caller id    
    ## @return [int, str, str]: [1, "", xmlRpcUri]
    def getUri(self, caller_id):
        return 1, "", self.uri

    @apivalidate('')
    ## (Python-Only API) Get the ROS node name of this server
    ## @param self
    ## @param caller_id str: ROS caller id    
    ## @return [int, str, str]: [1, "", ROS node name]
    def getName(self, caller_id):
        return 1, "", self.name


    ###############################################################################
    # EXTERNAL API

    @apivalidate([])
    def getBusStats(self, caller_id):
        """
        Retrieve transport/topic statistics
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: [publishStats, subscribeStats, serviceStats]::
           publishStats: [[topicName, messageDataSent, pubConnectionData]...[topicNameN, messageDataSentN, pubConnectionDataN]]
               pubConnectionData: [connectionId, bytesSent, numSent, connected]* . 
           subscribeStats: [[topicName, subConnectionData]...[topicNameN, subConnectionDataN]]
               subConnectionData: [connectionId, bytesReceived, dropEstimate, connected]* . dropEstimate is -1 if no estimate. 
           serviceStats: not sure yet, probably akin to [numRequests, bytesReceived, bytesSent] 
        """
        pub_stats, sub_stats = get_topic_manager().get_pub_sub_stats()
        #TODO: serviceStats
        return 1, '', [pub_stats, sub_stats, []]

    @apivalidate([])
    def getBusInfo(self, caller_id):
        """
        Retrieve transport/topic connection information
        @param caller_id: ROS caller id    
        @type  caller_id: str
        """
        return 1, "bus info", get_topic_manager().get_pub_sub_info()
    
    @apivalidate('')
    def getMasterUri(self, caller_id):
        """
        Get the URI of the master node.
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: [code, msg, masterUri]
        @rtype: [int, str, str]
        """
        if self.masterUri:
            return 1, self.masterUri, self.masterUri
        else:
            return 0, "master URI not set", ""

    def _shutdown(self, reason=''):
        """
        @param reason: human-readable debug string
        @type  reason: str
        """
        if not self.done:
            self.done = True
            if reason:
                _logger.info(reason)
            if self.protocol_handlers:
                for handler in self.protocol_handlers:
                    handler.shutdown()
                del self.protocol_handlers[:]
                self.protocol_handlers = None
            return True
        
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
        if self._shutdown('external shutdown request from [%s]: %s'%(caller_id, msg)):
            signal_shutdown('external shutdown request from [%s]: [%s]'%(caller_id, msg))
        return 1, "shutdown", 0

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

    ###############################################################################
    # PUB/SUB APIS

    @apivalidate([])
    def getSubscriptions(self, caller_id):
        """Retrieve a list of topics that this node subscribes to
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return [int, str, [ [topic1, topicType1]...[topicN, topicTypeN]]]: list of topics this node subscribes to
        """
        return 1, "subscriptions", get_topic_manager().get_subscriptions()

    @apivalidate([])
    def getPublications(self, caller_id):
        """
        Retrieve a list of topics that this node publishes.
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: list of topics published by this node
        @rtype: [int, str, [ [topic1, topicType1]...[topicN, topicTypeN]]]
        """
        return 1, "publications", get_topic_manager().get_publications()
    
    def _connect_topic(self, topic, pub_uri): 
        """
        Connect subscriber to topic
        @param topic: topic name to connect 
        @type  topic: str
        @param pub_uri: API URI of topic publisher
        @type  pub_uri: str
        @return: [code, msg, numConnects]. numConnects is the number
           of subscribers connected to the topic
        @rtype: [int, str, int]
        """
        caller_id = rospy.names.get_caller_id()
        sub = get_topic_manager().get_subscriber_impl(topic)
        if not sub:
            return -1, "No subscriber for topic [%s]"%topic, 0
        elif sub.has_connection(pub_uri):
            return 1, "_connect_topic[%s]: subscriber already connected to publisher [%s]"%(topic, pub_uri), 0
        
        #Negotiate with source for connection
        # - collect supported protocols
        protocols = []
        for h in self.protocol_handlers: #currently only TCPROS
            protocols.extend(h.get_supported())
        if not protocols:
            return 0, "ERROR: no available protocol handlers", 0

        _logger.debug("connect[%s]: calling requestTopic(%s, %s, %s)", topic, caller_id, topic, str(protocols))
        # 1) have to preserve original (unresolved) params as this may
        #    go outside our graph
        # 2) xmlrpclib doesn't give us any way of affecting the
        #    timeout other than affecting the global timeout. We need
        #    to set a timeout to prevent infinite hangs. 60 seconds is
        #    a *very* long time. All of the rospy code right now sets
        #    individual socket timeouts, but this could potentially
        #    affect user code.
        socket.setdefaulttimeout(60.)
        try:
            code, msg, result = \
                xmlrpcapi(pub_uri).requestTopic(caller_id, topic, protocols)
        except Exception, e:
            return 0, "unable to requestTopic: %s"%str(e), 0

        #Create the connection (if possible)
        if code <= 0:
            _logger.debug("connect[%s]: requestTopic did not succeed %s, %s", pub_uri, code, msg)
            return code, msg, 0
        elif not result or type(protocols) != list:
            return 0, "ERROR: publisher returned invalid protocol choice: %s"%(str(result)), 0
        _logger.debug("connect[%s]: requestTopic returned protocol list %s", topic, result)
        protocol = result[0]
        for h in self.protocol_handlers:
            if h.supports(protocol):
                return h.create_transport(topic, pub_uri, result)
        return 0, "ERROR: publisher returned unsupported protocol choice: %s"%result, 0

    @apivalidate(-1, (global_name('parameter_key'), None))
    def paramUpdate(self, caller_id, parameter_key, parameter_value):
        """
        Callback from master of current publisher list for specified topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param parameter_key str: parameter name, globally resolved
        @type  parameter_key: str
        @param parameter_value New parameter value
        @type  parameter_value: XMLRPC-legal value
        @return: [code, status, ignore]. If code is -1 ERROR, the node
        is not subscribed to parameter_key
        @rtype: [int, str, int]
        """
        try:
            rospy.paramserver.get_param_server_cache().update(parameter_key, parameter_value)
            return 1, '', 0
        except KeyError:
            return -1, 'not subscribed', 0

    @apivalidate(-1, (is_topic('topic'), is_publishers_list('publishers')))
    def publisherUpdate(self, caller_id, topic, publishers):
        """
        Callback from master of current publisher list for specified topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param topic str: topic name
        @type  topic: str
        @param publishers: list of current publishers for topic in the form of XMLRPC URIs
        @type  publishers: [str]
        @return: [code, status, ignore]
        @rtype: [int, str, int]
        """
        if self.reg_man:
            for uri in publishers:
                self.reg_man.publisher_update(topic, publishers)
        return 1, "", 0
    
    _remap_table['requestTopic'] = [0] # remap topic 
    @apivalidate([], (is_topic('topic'), non_empty('protocols')))
    def requestTopic(self, caller_id, topic, protocols):
        """
        Publisher node API method called by a subscriber node.
   
        Request that source allocate a channel for communication. Subscriber provides
        a list of desired protocols for communication. Publisher returns the
        selected protocol along with any additional params required for
        establishing connection. For example, for a TCP/IP-based connection,
        the source node may return a port number of TCP/IP server. 
        @param caller_id str: ROS caller id    
        @type  caller_id: str
        @param topic: topic name
        @type  topic: str
        @param protocols: list of desired
         protocols for communication in order of preference. Each
         protocol is a list of the form [ProtocolName,
         ProtocolParam1, ProtocolParam2...N]
        @type  protocols: [[str, XmlRpcLegalValue*]]
        @return: [code, msg, protocolParams]. protocolParams may be an
        empty list if there are no compatible protocols.
        @rtype: [int, str, [str, XmlRpcLegalValue*]]
        """
        if not get_topic_manager().has_publication(topic):
            return -1, "Not a publisher of [%s]"%topic, []
        for protocol in protocols: #simple for now: select first implementation 
            protocol_id = protocol[0]
            for h in self.protocol_handlers:
                if h.supports(protocol_id):
                    _logger.debug("requestTopic[%s]: choosing protocol %s", topic, protocol_id)
                    return h.init_publisher(topic, protocol)
        return 0, "no supported protocol implementations", []

## Contact api.publisherUpdate with specified parameters
## @param api str: XML-RPC URI of node to contact
## @param topic str: Topic name to send to node
## @param pub_uris [str]: list of publisher APIs to send to node
def publisher_update_task(api, topic, pub_uris):
    mloginfo("publisherUpdate[%s] -> %s", topic, api)
    #TODO: check return value for errors so we can unsubscribe if stale
    xmlrpcapi(api).publisherUpdate('/master', topic, pub_uris)

## Contact api.serviceUpdate with specified parameters
## @param api str: XML-RPC URI of node to contact
## @param service str: Service name to send to node
## @param uri str: URI to send to node        
def service_update_task(api, service, uri):
    mloginfo("serviceUpdate[%s, %s] -> %s",service, uri, api)
    xmlrpcapi(api).serviceUpdate('/master', service, uri)

###################################################
# Master Implementation

class ROSMasterHandler(ROSHandler):
    """
    XML-RPC handler for ROS master APIs.
    API routines for the ROS Master Node. The Master Node is a
    superset of the Slave Node and contains additional API methods for
    creating and monitoring a graph of slave nodes.
##
    By convention, ROS nodes take in caller_id as the first parameter
    of any API call.  The setting of this parameter is rarely done by
    client code as ros::msproxy::MasterProxy automatically inserts
    this parameter (see ros::client::getMaster()).
    """
    
    def __init__(self):
        """ctor."""
        super(ROSMasterHandler, self).__init__(MASTER_NAME, None, False)
        self.thread_pool = rospy.threadpool.MarkedThreadPool(NUM_WORKERS)
        # pub/sub/providers: dict { topicName : [publishers/subscribers names] }
        self.ps_lock = threading.Condition(threading.Lock())

        self.reg_manager = RegistrationManager(self.thread_pool)

        # maintain refs to reg_manager fields
        self.publishers  = self.reg_manager.publishers
        self.subscribers = self.reg_manager.subscribers
        self.services = self.reg_manager.services
        self.param_subscribers = self.reg_manager.param_subscribers
        
        self.topics_types = {} #dict { topicName : type }

        ## parameter server dictionary
        self.param_server = rospy.paramserver.ParamDictionary(self.reg_manager)

    def _shutdown(self, reason=''):
        if self.thread_pool is not None:
            self.thread_pool.join_all(wait_for_tasks=False, wait_for_threads=False)
            self.thread_pool = None
        return super(ROSMasterHandler, self)._shutdown(reason)
        
    @apivalidate('')
    def getMasterUri(self, caller_id): #override super behavior
        """
        Get the URI of the master(this) node.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @return: [code, msg, masterUri]
        @rtype: [int, str, str]
        """
        return self.getUri(caller_id)

    ## static map for tracking which arguments to a function should be remapped
    ## { methodName : [ arg indices ]
    _mremap_table = { } 

    @classmethod
    def remappings(cls, method_name):
        """
        @internal
        @return list: parameters (by pos) that should be remapped because they are names
        """
        if method_name in cls._mremap_table:
            return cls._mremap_table[method_name]
        else:
            return ROSHandler.remappings(method_name)

    ################################################################
    # PARAMETER SERVER ROUTINES
    
    _mremap_table['deleteParam'] = [0] # remap key
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
        
    _mremap_table['setParam'] = [0] # remap key
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

    _mremap_table['getParam'] = [0] # remap key
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

    _mremap_table['searchParam'] = [0] # remap key
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

    _mremap_table['subscribeParam'] = [0] # remap key
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

    _mremap_table['unsubscribeParam'] = [0] # remap key
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


    _mremap_table['hasParam'] = [0] # remap key
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

    def _notify(self, registrations, task, key, value):
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
        
        if registrations.has_key(key):
            try:            
                for node_api in registrations.get_apis(key):
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


    def _notify_topic_subscribers(self, topic, pub_uris):
        """
        Notify subscribers with new publisher list
        @param topic: name of topic
        @type  topic: str
        @param pub_uris: list of URIs of publishers.
        @type  pub_uris: [str]
        """
        self._notify(self.subscribers, publisher_update_task, topic, pub_uris)

    def _notify_service_update(self, service, service_api):
        """
        Notify clients of new service provider
        @param service: name of service
        @type  service: str
        @param service_api: new service URI
        @type  service_api: str
        """
        ###TODO:XXX:stub code, this callback doesnot exist yet
        self._notify(self.service_clients, service_update_task, service, service_api)
        
    ##################################################################################
    # SERVICE PROVIDER

    _mremap_table['registerService'] = [0] # remap service
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
            if 0: #TODO
                self._notify_service_update(service, service_api)
        finally:
            self.ps_lock.release()
        return 1, "Registered [%s] as provider of [%s]"%(caller_id, service), 1

    _mremap_table['lookupService'] = [0] # remap service
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

    _mremap_table['unregisterService'] = [0] # remap service
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
            if 0: #TODO
                self._notify_service_update(service, service_api)
            mloginfo("-SERVICE [%s] %s %s", service, caller_id, service_api)
            return retval
        finally:
            self.ps_lock.release()

    ##################################################################################
    # PUBLISH/SUBSCRIBE

    _mremap_table['registerSubscriber'] = [0] # remap topic
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
            mloginfo("+SUB [%s] %s %s",topic, caller_id, caller_api)
            pub_uris = self.publishers.get_apis(topic)
        finally:
            self.ps_lock.release()
        return 1, "Subscribed to [%s]"%topic, pub_uris

    _mremap_table['unregisterSubscriber'] = [0] # remap topic    
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

    _mremap_table['registerPublisher'] = [0] # remap topic   
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
            if topic_type != rospy.names.TOPIC_ANYTYPE or not topic in self.topics_types:
                self.topics_types[topic] = topic_type
            pub_uris = self.publishers.get_apis(topic)
            self._notify_topic_subscribers(topic, pub_uris)
            mloginfo("+PUB [%s] %s %s",topic, caller_id, caller_api)
            sub_uris = self.subscribers.get_apis(topic)            
        finally:
            self.ps_lock.release()
        return 1, "Registered [%s] as publisher of [%s]"%(caller_id, topic), sub_uris


    _mremap_table['unregisterPublisher'] = [0] # remap topic   
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
                self._notify_topic_subscribers(topic, self.publishers.get_apis(topic))
            mloginfo("-PUB [%s] %s %s",topic, caller_id, caller_api)
        finally:
            self.ps_lock.release()
        return retval

    ##################################################################################
    # GRAPH STATE APIS

    _mremap_table['lookupNode'] = [0] # remap node
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
        
    _mremap_table['getPublishedTopics'] = [0] # remap subgraph
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
            if subgraph and subgraph[-1] != SEP:
                subgraph = subgraph + SEP
            #we don't bother with subscribers as subscribers don't report topic types. also, the intended
            #use case is for subscribe-by-topic-type
            retval = [[t, self.topics_types[t]] for t in self.publishers.iterkeys() if t.startswith(subgraph)]
        finally:
            self.ps_lock.release()
        return 1, "current topics", retval
    
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
