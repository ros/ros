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
"""
Python adapter for calling ROS Master API. While it is trivial to call the 
Master directly using XML-RPC, this API provides a safer abstraction in the event
the Master API is changed.
"""

import os
import sys
import xmlrpclib

import roslib.exceptions

class ROSMasterException(roslib.exceptions.ROSLibException):
    """
    Base class of ROS-master related errors.
    """
    pass

class Failure(ROSMasterException):
    """
    Call to Master failed. This generally indicates an internal error
    in the Master and that the Master may be in an inconsistent state.
    """
    pass

class Error(ROSMasterException):
    """
    Master returned an error code, which indicates an error in the
    arguments passed to the Master.
    """
    pass

class Master(object):
    """
    API for interacting with the ROS master. Although the Master is
    relatively simple to interact with using the XMLRPC API, this
    abstraction layer provides protection against future updates. It
    also provides a streamlined API with builtin return code checking
    and caller_id passing.
    """
    
    def __init__(self, caller_id, master_uri=None):
        if master_uri is None:
            master_uri = roslib.rosenv.get_master_uri()
        self.master_uri = master_uri
        self.handle = xmlrpclib.ServerProxy(self.master_uri)
        self.caller_id = caller_id
        
    def _succeed(self, args):
        """
        Check master return code and return the value field.
        
        @param args: master return value
        @type  args: (int, str, XMLRPCLegalValue)
        @return: value field of args (master return value)
        @rtype: XMLRPCLegalValue
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        code, msg, val = args
        if code == 1:
            return val
        elif code == -1:
            raise Error(msg)
        else:
            raise Failure(msg)            

    def getPid(self):
        """
        Get the PID of this server
        @param caller_id: ROS caller id
        @type  caller_id: str
        @return: serverProcessPID
        @rtype: int
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.getPid(self.caller_id))
    
    def registerService(self, service, service_api, caller_api):
        """
        Register the caller as a provider of the specified service.
        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param service str: Fully-qualified name of service 
        @param service_api str: Service URI 
        @param caller_api str: XML-RPC URI of caller node 
        @return: ignore
        @rtype: int
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """        
        return self._succeed(self.handle.registerService(self.caller_id, service, service_api, caller_api))
    
    def lookupService(self, service):
        """
        Lookup all provider of a particular service.
        @param caller_id str: ROS caller id
        @type  caller_id: str
        @param service: fully-qualified name of service to lookup.
        @type: service: str
        @return (int, str, str): (code, message, serviceUrl). service URL is provides
           and address and port of the service.  Fails if there is no provider.
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.lookupService(self.caller_id, service))
    

    def unregisterService(self, service, service_api):
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
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.unregisterService(self.caller_id, service, service_api))
    

    def registerSubscriber(self, topic, topic_type, caller_api):
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
        @rtype: (int, str, list(str))
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.registerSubscriber(self.caller_id, topic, topic_type, caller_api))
    

    def unregisterSubscriber(self, topic, caller_api):
        """
        Unregister the caller as a publisher of the topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param topic: Fully-qualified name of topic to unregister.
        @type  topic: str
        @param caller_api: API URI of service to unregister. Unregistration will only occur if current
        @type  caller_api: str
           registration matches.    
        @return: (code, statusMessage, numUnsubscribed). 
          If numUnsubscribed is zero it means that the caller was not registered as a subscriber.
          The call still succeeds as the intended final state is reached.
        @rtype: (int, str, int)
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.unregisterSubscriber(self.caller_id, topic, caller_api))
    
    def registerPublisher(self, topic, topic_type, caller_api):
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
        @return: subscriberApis.
        List of current subscribers of topic in the form of XMLRPC URIs.
        @rtype: [str]
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.registerPublisher(self.caller_id, topic, topic_type, caller_api))
    
    def unregisterPublisher(self, topic, caller_api):
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
        @return: numUnregistered. 
           If numUnregistered is zero it means that the caller was not registered as a publisher.
           The call still succeeds as the intended final state is reached.
        @rtype: int
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """            
        return self._succeed(self.handle.unregisterPublisher(self.caller_id, topic, caller_api))        

    def lookupNode(self, node_name):
        """
        Get the XML-RPC URI of the node with the associated
        name/caller_id.  This API is for looking information about
        publishers and subscribers. Use lookupService instead to lookup
        ROS-RPC URIs.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param node: name of node to lookup
        @type  node: str
        @return: URI
        @rtype: str
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.lookupNode(self.caller_id, node_name))        
        
    def getPublishedTopics(self, subgraph):
        """
        Get list of topics that can be subscribed to. This does not return topics that have no publishers.
        See L{getSystemState()} to get more comprehensive list.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param subgraph: Restrict topic names to match within the specified subgraph. Subgraph namespace
           is resolved relative to the caller's namespace. Use '' to specify all names.
        @type  subgraph: str
        @return: [[topic1, type1]...[topicN, typeN]]
        @rtype: [[str, str],]
        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.getPublishedTopics(self.caller_id, subgraph))        
    
    def getSystemState(self): 
        """
        Retrieve list representation of system state (i.e. publishers, subscribers, and services).
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @rtype: [[str,[str]], [str,[str]], [str,[str]]]
        @return: systemState

           System state is in list representation::
             [publishers, subscribers, services].
        
           publishers is of the form::
             [ [topic1, [topic1Publisher1...topic1PublisherN]] ... ]
        
           subscribers is of the form::
             [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
        
           services is of the form::
             [ [service1, [service1Provider1...service1ProviderN]] ... ]

        @raise roslib.masterapi.Error: if Master returns ERROR.
        @raise roslib.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.getSystemState(self.caller_id))
