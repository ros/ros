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

"""Internal-use: data structures for the Master"""

## Container for node registration information. Used in master's
## self.nodes data structure.  This is effectively a reference
## counter for the node registration information: when the
## subscriptions and publications are empty the node registration can
## be deleted.
class NodeRef(object):

    ## ctor
    ## @param self
    ## @param api str: node XML-RPC API
    def __init__(self, id, api):
        self.id = id
        self.api = api
        self.param_subscriptions = []
        self.topic_subscriptions = []
        self.topic_publications  = []
        self.services  = []        
        
    ## Delete all state from this NodeRef except for the api location
    def clear(self):
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
    
    def add(self, type_, key):
        if type_ == Registrations.TOPIC_SUBSCRIPTIONS:
            if not key in self.topic_subscriptions:
                self.topic_subscriptions.append(key)
        elif type_ == Registrations.TOPIC_PUBLICATIONS:
            if not key in self.topic_publications:
                self.topic_publications.append(key)
        elif type_ == Registrations.SERVICE:
            if not key in self.services:
                self.services.append(key)
        elif type_ == Registrations.PARAM_SUBSCRIPTIONS:
            if not key in self.param_subscriptions:
                self.param_subscriptions.append(key)
        else:
            raise rospy.exceptions.ROSInternalException("internal bug")

    def remove(self, type_, key):
        if type_ == Registrations.TOPIC_SUBSCRIPTIONS:
            if key in self.topic_subscriptions:        
                self.topic_subscriptions.remove(key)
        elif type_ == Registrations.TOPIC_PUBLICATIONS:
            if key in self.topic_publications:
                self.topic_publications.remove(key)
        elif type_ == Registrations.SERVICE:
            if key in self.services:
                self.services.remove(key)
        elif type_ == Registrations.PARAM_SUBSCRIPTIONS:
            if key in self.param_subscriptions:
                self.param_subscriptions.remove(key)
        else:
            raise rospy.exceptions.ROSInternalException("internal bug")

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
    def __init__(self, type_):
        if not type_ in [
            Registrations.TOPIC_SUBSCRIPTIONS,
            Registrations.TOPIC_PUBLICATIONS,
            Registrations.SERVICE,
            Registrations.PARAM_SUBSCRIPTIONS ]:
            raise rospy.exceptions.ROSInternalException("invalid registration type: %s"%type_)
        self.type = type_
        ## { key: [(caller_id, caller_api)] }
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
        if self.service_api_map and service in self.service_api_map:
            caller_id, service_api = self.service_api_map[service]
            return service_api
        return None
    
    ## Only valid if self.type != SERVICE.
    ## @param self
    ## @param key str: registration key (e.g. topic/service/param name)
    ## @return [str]: caller_apis for registered key, empty list if registration is not valid
    def get_apis(self, key):
        return [api for _, api in self.map.get(key, [])]

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
        return self.map.get(key, [])

    ## @param self
    ## @param key str: registration key (e.g. topic/service/param name)
    ## @return bool True if \a key is registered
    def has_key(self, key):
        return key in self.map
    
    ## @param self
    ## @return [str, [str]...]: state in getSystemState()-friendly format [ [key, [callerId1...callerIdN]] ... ]
    def get_state(self):
        retval = []
        for k in self.map.iterkeys():
            retval.append([k, [id for id, _ in self.map[k]]])
        return retval

    ## Add \a caller_id into the map as a provider of the specified
    ## service (key).  \a caller_id must not have been previously
    ## registered with a different \a caller_api.
    ## Subroutine for managing provider map data structure (essentially a multimap).
    ## @param self
    ## @param key str: registration key (e.g. topic/service/param name)
    ## @param caller_id str: caller_id of provider
    ## @param caller_api str: API URI of provider 
    ## @param service_api str: (keyword) ROS service API URI if registering a service
    def register(self, key, caller_id, caller_api, service_api=None):
        map = self.map
        if key in map and not service_api:
            providers = map[key]
            if not (caller_id, caller_api) in providers:
                providers.append((caller_id, caller_api))
        else:
            map[key] = providers = [(caller_id, caller_api)]

        if service_api:
            if self.service_api_map is None:
               self.service_api_map = {}
            self.service_api_map[key] = (caller_id, service_api)
        elif self.type == Registrations.SERVICE:
            raise rospy.exceptions.ROSInternalException("service_api must be specified for Registrations.SERVICE")            
                   
    ## Remove all registrations associated with \a caller_id
    ## @param caller_id str: caller_id of provider
    def unregister_all(self, caller_id):
        map = self.map
        # fairly expensive
        dead_keys = []
        for key in map:
            providers = map[key]
            # find all matching entries
            to_remove = [(id, api) for id, api in providers if id == caller_id]
            # purge them
            for r in to_remove:
                providers.remove(r)
            if not providers:
                dead_keys.append(key)
        for k in dead_keys:
            del self.map[k]
        if self.type == Registrations.SERVICE and self.service_api_map:
            del dead_keys[:]
            for key, val in self.service_api_map.iteritems():
                if val[0] == caller_id:
                    dead_keys.append(key)
            for k in dead_keys:
                del self.service_api_map[k]
    
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
        # if we are unregistering a topic, validate against the caller_api
        if service_api:
            # validate against the service_api 
            if self.service_api_map is None:
                return 1, "[%s] is not a provider of [%s]"%(caller_id, key), 0 
            if self.service_api_map.get(key, None) != (caller_id, service_api):
                return 1, "[%s] is no longer the current service api handle for [%s]"%(service_api, key), 0
            else:
                del self.service_api_map[key]
                del self.map[key]
            # caller_api is None for unregister service, so we can't validate as well
            return 1, "Unregistered [%s] as provider of [%s]"%(caller_id, key), 1
        elif self.type == Registrations.SERVICE:
            raise rospy.exceptions.ROSInternalException("service_api must be specified for Registrations.SERVICE")
        else:
            providers = self.map.get(key, [])
            if (caller_id, caller_api) in providers:
                providers.remove((caller_id, caller_api))
                if not providers:
                    del self.map[key]
                return 1, "Unregistered [%s] as provider of [%s]"%(caller_id, key), 1
            else:
                return 1, "[%s] is not a known provider of [%s]"%(caller_id, key), 0

## RegistrationManager is not threadsafe, so access must be externally locked as appropriate
class RegistrationManager(object):

    ## @param self
    ## @param thread_pool ThreadPool thread pool for queueing tasks
    def __init__(self, thread_pool):
        self.nodes = {}
        self.thread_pool = thread_pool

        self.publishers  = Registrations(Registrations.TOPIC_PUBLICATIONS)
        self.subscribers = Registrations(Registrations.TOPIC_SUBSCRIPTIONS)
        self.services = Registrations(Registrations.SERVICE)
        self.param_subscribers = Registrations(Registrations.PARAM_SUBSCRIPTIONS)        

    
    ## Get a NodeRef by \a caller_api
    ## @param caller_api str
    ## @return [NodeRef]: nodes that declare \a caller_api as their
    ## API. 99.9% of the time this should only be one node, but we
    ## allow for multiple matches as the master API does not restrict
    ## this.
    def reverse_lookup(self, caller_api):
        matches = [n for n in self.nodes.iteritems() if n.api == caller_api]
        if matches:
            return matches
        
    def get_node(self, caller_id):
        return self.nodes.get(caller_id, None)

    def _register(self, r, key, caller_id, caller_api, service_api=None):
        # update node information
        node_ref, changed = self._register_node_api(caller_id, caller_api)
        node_ref.add(r.type, key)
        # update pub/sub/service indicies
        if changed:
            self.publishers.unregister_all(caller_id)
            self.subscribers.unregister_all(caller_id)
            self.services.unregister_all(caller_id)
            self.param_subscribers.unregister_all(caller_id)
        r.register(key, caller_id, caller_api, service_api)
        
    def _unregister(self, r, key, caller_id, caller_api, service_api=None):
        node_ref = self.nodes.get(caller_id, None)
        if node_ref != None:
            retval = r.unregister(key, caller_id, caller_api, service_api)
            # check num removed field, if 1, unregister is valid
            if retval[2] == 1:
                node_ref.remove(r.type, key)
            if node_ref.is_empty():
                del self.nodes[caller_id]
        else:
            retval = 1, "[%s] is not a registered node"%caller_id, 0
        return retval
    
    ## Register service provider
    ## @return None
    def register_service(self, service, caller_id, caller_api, service_api):
        self._register(self.services, service, caller_id, caller_api, service_api)
    ## Register topic publisher
    ## @return None
    def register_publisher(self, topic, caller_id, caller_api):
        self._register(self.publishers, topic, caller_id, caller_api)
    ## Register topic subscriber
    ## @return None
    def register_subscriber(self, topic, caller_id, caller_api):
        self._register(self.subscribers, topic, caller_id, caller_api)
    ## Register param subscriber
    ## @return None
    def register_param_subscriber(self, param, caller_id, caller_api):
        self._register(self.param_subscribers, param, caller_id, caller_api)

    def unregister_service(self, service, caller_id, service_api):
        caller_api = None
        return self._unregister(self.services, service, caller_id, caller_api, service_api)
        
    def unregister_subscriber(self, topic, caller_id, caller_api):
        return self._unregister(self.subscribers, topic, caller_id, caller_api)
    def unregister_publisher(self, topic, caller_id, caller_api):
        return self._unregister(self.publishers, topic, caller_id, caller_api)
    def unregister_param_subscriber(self, param, caller_id, caller_api):
        return self._unregister(self.param_subscribers, param, caller_id, caller_api)
        
    ## @param self
    ## @param caller_id str: caller_id of provider
    ## @param caller_api str: caller_api of provider
    ## @return (NodeRef, bool) (registration_information,
    ## changed_registration). changed_registration is true if \a
    ## caller_api is differet than the one registered with \a
    ## caller_id
    def _register_node_api(self, caller_id, caller_api):
        node_ref = self.nodes.get(caller_id, None)

        bumped_api = None
        if node_ref is not None:
            if node_ref.api == caller_api:
                return node_ref, False
            else:
                bumped_api = node_ref.api
                self.thread_pool.queue_task(bumped_api, shutdown_node_task,
                                            (bumped_api, caller_id, "new node registered with same name"))

        node_ref = NodeRef(caller_id, caller_api)
        self.nodes[caller_id] = node_ref
        return (node_ref, bumped_api != None)


