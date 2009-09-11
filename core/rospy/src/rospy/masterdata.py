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
# Revision $Id: masterslave.py 2364 2008-10-09 00:06:22Z sfkwc $

from rospy.core import xmlrpcapi, mloginfo
from rospy.names import get_caller_id
import rospy.exceptions

## Data structures for the Master

## Container for node registration information. Used in master's
## self.nodes data structure.  This is effectively a reference
## counter for the node registration information: when the
## subscriptions and publications are empty the node registration can
## be deleted.
class NodeRef(object):

    ## ctor
    ## @param self
    ## @param api str: node XML-RPC API
    def __init__(self, api):
        self.api = api
        self.param_subscriptions = []
        self.topic_subscriptions = []
        self.topic_publications  = []
        self.services  = []        
        
    ## @param self
    ## @return True if node has no active registrations
    def is_empty(self):
        return sum((len(x) for x in
                    [self.param_subscriptions, 
                     self.topic_subscriptions,
                     self.topic_publications,
                     self.services,])) == 0
    
    def add_param_subscription(self, param_key):
        if not param_key in self.param_subscriptions:
            self.param_subscriptions.append(param_key)
    def add_topic_subscription(self, topic):
        if not topic in self.topic_subscriptions:
            self.topic_subscriptions.append(topic)
    def add_topic_publication(self, topic):
        if not topic in self.topic_publications:
            self.topic_publications.append(topic)
    def add_service(self, service):
        if not service in self.services:
            self.services.append(service)
            
    def remove_param_subscription(self, param_key):
        if param_key in self.param_subscriptions:
            self.param_subscriptions.remove(param_key)
    def remove_topic_subscription(self, topic):
        if topic in self.topic_subscriptions:        
            self.topic_subscriptions.remove(topic)
    def remove_topic_publication(self, topic):
        if topic in self.topic_publications:
            self.topic_publications.remove(topic)
    def remove_service(self, service):
        if service in self.services:
            self.services.remove(service)

# NOTE: I'm not terribly happy that this task has leaked into the data model. need
# to refactor to get this back into masterslave.

## Method to shutdown another ROS node. Generally invoked within a
## separate thread as this is used to cleanup hung nodes.
## @param api str: XML-RPC API of node to shutdown
## @param caller_id str: name of node being shutdown
## @param reason str: human-readable reason why node is being shutdown
def shutdown_node_task(api, caller_id, reason):
    import logging
    try:
        mloginfo("SHUTDOWN [%s %s]: %s"%(caller_id, api, reason))
        xmlrpcapi(api).shutdown(get_caller_id(), reason)
    except:
        pass #expected in many common cases
    
## All calls may result in access/modifications to node registrations
## dictionary, so be careful to guarantee appropriate thread-safeness.
##
## Data structure for storing a set of registrations (e.g. publications, services).
## The underlying data storage is the same except for services, which have the
## constraint that only one registration may be active for a given key. 
class Registrations(object):

    TOPIC_SUBSCRIPTIONS = 1
    TOPIC_PUBLICATIONS = 2
    SERVICE = 3
    PARAM_SUBSCRIPTIONS = 4
    
    ## @param self
    ## @param type_ int: one of [ TOPIC_SUBSCRIPTIONS,
    ## TOPIC_PUBLICATIONS, SERVICE, PARAM_SUBSCRIPTIONS ]
    ## @param node_registrations dict: node registrations dictionary {
    ## caller_id: NodeRef }.  method calls may modify
    ## node_registrations and should be protected with Locks
    ## @param thread_pool ThreadPool thread pool for queueing tasks
    def __init__(self, type_, node_registrations, thread_pool):
        if not type_ in [
            Registrations.TOPIC_SUBSCRIPTIONS,
            Registrations.TOPIC_PUBLICATIONS,
            Registrations.SERVICE,
            Registrations.PARAM_SUBSCRIPTIONS ]:
            raise rospy.exceptions.ROSInternalException("invalid registration type: %s"%type_)
        self.type = type_
        self.node_registrations = node_registrations
        self.thread_pool = thread_pool
        ## { key: [caller_id] }
        self.map = {} 
        self.service_api_map = None

    ## @return True if there are no registrations
    def __nonzero__(self):
        return len(self.map) != 0

    ## Iterate over registration keys
    ## @param self
    ## @return iterator for registration keys
    def iterkeys(self):
        return self.map.iterkeys()

    ## Lookup service API URI. NOTE: this should only be valid if type==SERVICE as
    ## service Registrations instances are the only ones that track service API URIs.
    ## @param self
    ## @param service str: service name
    ## @return str: service_api for registered key or None if
    ## registration is no longer valid. 
    def get_service_api(self, service):
        if self.service_api_map:
            return self.service_api_map.get(service, None)
        else:
            return None
    
    ## Only valid if self.type != SERVICE.
    ## @param self
    ## @param key str: registration key (e.g. topic/service/param name)
    ## @return [str]: caller_apis for registered key, empty list if registration is not valid
    def get_apis(self, key):
        ids = self.map.get(key, [])
        return [self.node_registrations[id].api for id in ids if id in self.node_registrations]
        

    ## Emulate mapping type for has_key()
    def __contains__(self, key):
        return key in self.map
    
    ## @param self
    ## @param key str: registration key (e.g. topic/service/param name)
    ## @return [(str, str),]: (caller_id, caller_api) for registered
    ## key, empty list if registration is not valid
    def __getitem__(self, key):
        # unlike get_apis, returns the caller_id to prevent any race
        # conditions that can occur if caller_id/caller_apis change
        # due to a new node.
        ids = self.map.get(key, [])
        return [(id, self.node_registrations[id].api) for id in ids if id in self.node_registrations]

    ## @param self
    ## @param key str: registration key (e.g. topic/service/param name)
    ## @return bool True if \a key is registered
    def has_key(self, key):
        return key in self.map
    
    ## @param self
    ## @return [str, [str]...]: state in getSystemState()-friendly format [ [key, [callerId1...callerIdN]] ... ]
    def get_state(self):
        return [ [k, v] for k, v in self.map.iteritems()]

    ## Must be implemented by subclasses. Update NodeRef to record
    ## registration
    ## @param self
    ## @param key str: registration key
    ## @param node_ref NodeRef: node registration reference
    def _register_ref(self, node_ref, key):
        # there's the opportunity to get much cuter with Python here
        # and eliminate the runtime checks. I would like to get ride
        # of type entirely
        if self.type == self.TOPIC_SUBSCRIPTIONS:
            node_ref.add_topic_subscription(key)
        elif self.type == self.TOPIC_PUBLICATIONS:
            node_ref.add_topic_publication(key)
        elif self.type == self.SERVICE:
            node_ref.add_service(key)            
        elif self.type == self.PARAM_SUBSCRIPTIONS:
            node_ref.add_param_subscription(key)
        else:
            raise rospy.exceptions.ROSInternalException("internal bug")

    ## Must be implemented by subclasses. Update NodeRef to record
    ## unregistration
    ## @param self
    ## @param node_ref NodeRef: node registration reference
    ## @param key str: registration key
    ## @param caller_id str: caller ID of Node
    def _unregister_ref(self, node_ref, key, caller_id):
        # there's the opportunity to get much cuter with Python here
        # and eliminate the runtime checks.
        if self.type == self.TOPIC_SUBSCRIPTIONS:
            node_ref.remove_topic_subscription(key)
        elif self.type == self.TOPIC_PUBLICATIONS:
            node_ref.remove_topic_publication(key)
        elif self.type == self.SERVICE:
            node_ref.remove_service(key)            
        elif self.type == self.PARAM_SUBSCRIPTIONS:
            node_ref.remove_param_subscription(key)
        else:
            raise rospy.exceptions.ROSInternalException("internal bug")
        if node_ref.is_empty():
            del self.node_registrations[caller_id]
        
    ## Subroutine for managing self.node_registrations dictionary
    ## @param self
    ## @param caller_id str: caller_id of provider
    ## @param caller_api str: caller_api of provider            
    ## @return NodeRef node registration information
    def register_node_api(self, caller_id, caller_api):
        node_ref = self.node_registrations.get(caller_id, None)

        if node_ref is not None:
            if node_ref.api == caller_api:
                return node_ref
            else:
                bumped_api = node_ref.api
                self.thread_pool.queue_task(bumped_api, shutdown_node_task,
                                            (bumped_api, caller_id, "new node registered with same name"))

        node_ref = NodeRef(caller_api)
        self.node_registrations[caller_id] = node_ref
        return node_ref

    ## Add caller_id into the map as a provider of the specified service (key).
    ## Subroutine for managing provider map data structure (essentially a multimap).
    ## @param self
    ## @param key str: registration key (e.g. topic/service/param name)
    ## @param caller_id str: caller_id of provider
    ## @param caller_api str: API URI of provider 
    ## @param service_api str: (keyword) ROS service API URI if registering a service
    ## @return ([str], str): Updated provider list (list of caller_ids)
    ## as well as URI of previous provider (or None).
    def register(self, key, caller_id, caller_api, service_api=None):
        node_ref = self.register_node_api(caller_id, caller_api)

        self._register_ref(node_ref, key)

        map = self.map
        if key in map:
            providers = map[key]
            if not caller_id in providers:
                providers.append(caller_id)
        else:
            map[key] = providers = [caller_id]

        if service_api:
            if self.service_api_map is None:
               self.service_api_map = {}
            self.service_api_map[key] = service_api
                   
        return providers

    ## Remove caller_id from the map as a provider of the specified service (key).
    ## Subroutine for managing provider map data structure, essentially a multimap
    ## @param self
    ## @param key str: registration key (e.g. topic/service/param name)
    ## @param caller_id str: caller_id of provider
    ## @param caller_api str: API URI of provider            
    ## @param service_api str: (keyword) ROS service API URI if registering a service
    ## @return code, msg, val: for ease of master integration, directly returns unregister value for
    ## higher-level XMLRPC API. val is the number of APIs unregistered (0 or 1)
    def unregister(self, key, caller_id, caller_api, service_api=None):
        node_ref = self.node_registrations.get(caller_id, None)

        if node_ref is None:
            return 1, "unknown provider [%s]. This probably means the master was restarted"%caller_id, 0

        # if we are unregistering a service, validate against the caller_api
        if not service_api:
            if node_ref.api != caller_api:
                return 1, "[%s] is no longer the current api handle for [%s]"%(caller_api, key), 0
        else:
            # validate against the service_api instead
            if self.service_api_map is None:
                return 1, "[%s] is not a known provider of [%s]"%(caller_id, key), 0                
            if self.service_api_map.get(key, None) != service_api:
                return 1, "[%s] is no longer the current service api handle for [%s]"%(service_api, key), 0
            else:
                del self.service_api_map[key]
        
        self._unregister_ref(node_ref, key, caller_id)
        
        providers = self.map.get(key, [])
        if caller_id in providers:
            providers.remove(caller_id)
            if not providers:
                del self.map[key]
            return 1, "Unregistered [%s] as provider of [%s]"%(caller_id, key), 1
        return 1, "[%s] is not a known provider of [%s]"%(caller_id, key), 0
