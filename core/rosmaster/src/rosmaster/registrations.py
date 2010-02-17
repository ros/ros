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

from rosmaster.util import xmlrpcapi
import rosmaster.exceptions

"""Data structures for representing registration data in the Master"""

class NodeRef(object):
    """
    Container for node registration information. Used in master's
    self.nodes data structure.  This is effectively a reference
    counter for the node registration information: when the
    subscriptions and publications are empty the node registration can
    be deleted.
    """
    def __init__(self, id, api):
        """
        ctor
        @param api str: node XML-RPC API
        """
        self.id = id
        self.api = api
        self.param_subscriptions = []
        self.topic_subscriptions = []
        self.topic_publications  = []
        self.services  = []        
        
    def clear(self):
        """
        Delete all state from this NodeRef except for the api location
        """
        self.param_subscriptions = []
        self.topic_subscriptions = []
        self.topic_publications  = []
        self.services  = []        

    def is_empty(self):
        """
        @return: True if node has no active registrations
        """
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
            raise rosmaster.exceptions.InternalException("internal bug")

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
            raise rosmaster.exceptions.InternalException("internal bug")

# NOTE: I'm not terribly happy that this task has leaked into the data model. need
# to refactor to get this back into masterslave.

def shutdown_node_task(api, caller_id, reason):
    """
    Method to shutdown another ROS node. Generally invoked within a
    separate thread as this is used to cleanup hung nodes.
    
    @param api: XML-RPC API of node to shutdown
    @type  api: str
    @param caller_id: name of node being shutdown
    @type  caller_id: str
    @param reason: human-readable reason why node is being shutdown
    @type  reason: str
    """
    try:
        xmlrpcapi(api).shutdown('/master', reason)
    except:
        pass #expected in many common cases
    
class Registrations(object):
    """
    All calls may result in access/modifications to node registrations
    dictionary, so be careful to guarantee appropriate thread-safeness.

    Data structure for storing a set of registrations (e.g. publications, services).
    The underlying data storage is the same except for services, which have the
    constraint that only one registration may be active for a given key. 
    """

    TOPIC_SUBSCRIPTIONS = 1
    TOPIC_PUBLICATIONS = 2
    SERVICE = 3
    PARAM_SUBSCRIPTIONS = 4
    
    def __init__(self, type_):
        """
        ctor.
        @param type_: one of [ TOPIC_SUBSCRIPTIONS,
        TOPIC_PUBLICATIONS, SERVICE, PARAM_SUBSCRIPTIONS ]
        @type  type_: int
        """
        if not type_ in [
            Registrations.TOPIC_SUBSCRIPTIONS,
            Registrations.TOPIC_PUBLICATIONS,
            Registrations.SERVICE,
            Registrations.PARAM_SUBSCRIPTIONS ]:
            raise rosmaster.exceptions.InternalException("invalid registration type: %s"%type_)
        self.type = type_
        ## { key: [(caller_id, caller_api)] }
        self.map = {} 
        self.service_api_map = None

    def __nonzero__(self):
        """
        @return: True if there are no registrations
        """
        return len(self.map) != 0

    def iterkeys(self):
        """
        Iterate over registration keys
        @return: iterator for registration keys
        """
        return self.map.iterkeys()

    def get_service_api(self, service):
        """
        Lookup service API URI. NOTE: this should only be valid if type==SERVICE as
        service Registrations instances are the only ones that track service API URIs.
        @param service: service name
        @type  service: str
        @return str: service_api for registered key or None if
        registration is no longer valid. 
        @type: str
        """
        if self.service_api_map and service in self.service_api_map:
            caller_id, service_api = self.service_api_map[service]
            return service_api
        return None
    
    def get_apis(self, key):
        """
        Only valid if self.type != SERVICE.
        @param key: registration key (e.g. topic/service/param name)
        @type  key: str
        @return: caller_apis for registered key, empty list if registration is not valid
        @rtype: [str]
        """
        return [api for _, api in self.map.get(key, [])]

    def __contains__(self, key):
        """
        Emulate mapping type for has_key()
        """
        return key in self.map
    
    def __getitem__(self, key):
        """
        @param key: registration key (e.g. topic/service/param name)
        @type  key: str
        @return: (caller_id, caller_api) for registered
        key, empty list if registration is not valid
        @rtype: [(str, str),]
        """
        # unlike get_apis, returns the caller_id to prevent any race
        # conditions that can occur if caller_id/caller_apis change
        # due to a new node.
        return self.map.get(key, [])

    def has_key(self, key):
        """
        @param key: registration key (e.g. topic/service/param name)
        @type  key: str
        @return: True if key is registered
        @rtype: bool
        """
        return key in self.map
    
    def get_state(self):
        """
        @return: state in getSystemState()-friendly format [ [key, [callerId1...callerIdN]] ... ]
        @rtype: [str, [str]...]
        """
        retval = []
        for k in self.map.iterkeys():
            retval.append([k, [id for id, _ in self.map[k]]])
        return retval

    def register(self, key, caller_id, caller_api, service_api=None):
        """
        Add caller_id into the map as a provider of the specified
        service (key).  caller_id must not have been previously
        registered with a different caller_api.
    
        Subroutine for managing provider map data structure (essentially a multimap).
        @param key: registration key (e.g. topic/service/param name)
        @type  key: str
        @param caller_id: caller_id of provider
        @type  caller_id: str
        @param caller_api: API URI of provider 
        @type  caller_api: str
        @param service_api: (keyword) ROS service API URI if registering a service
        @type  service_api: str
        """
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
            raise rosmaster.exceptions.InternalException("service_api must be specified for Registrations.SERVICE")            
                   
    def unregister_all(self, caller_id):
        """
        Remove all registrations associated with caller_id
        @param caller_id: caller_id of provider
        @type  caller_id: str
        """
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
    
    def unregister(self, key, caller_id, caller_api, service_api=None):
        """
        Remove caller_id from the map as a provider of the specified service (key).
        Subroutine for managing provider map data structure, essentially a multimap
        @param key: registration key (e.g. topic/service/param name)
        @type  key: str
        @param caller_id: caller_id of provider
        @type  caller_id: str
        @param caller_api: API URI of provider            
        @type  caller_api: str
        @param service_api: (keyword) ROS service API URI if registering a service
        @type  service_api: str
        @return: for ease of master integration, directly returns unregister value for
        higher-level XMLRPC API. val is the number of APIs unregistered (0 or 1)
        @rtype: code, msg, val
        """
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
            raise rosmaster.exceptions.InternalException("service_api must be specified for Registrations.SERVICE")
        else:
            providers = self.map.get(key, [])
            if (caller_id, caller_api) in providers:
                providers.remove((caller_id, caller_api))
                if not providers:
                    del self.map[key]
                return 1, "Unregistered [%s] as provider of [%s]"%(caller_id, key), 1
            else:
                return 1, "[%s] is not a known provider of [%s]"%(caller_id, key), 0

class RegistrationManager(object):
    """
    Stores registrations for Master.
    
    RegistrationManager is not threadsafe, so access must be externally locked as appropriate
    """

    def __init__(self, thread_pool):
        """
        ctor.
        @param thread_pool: thread pool for queueing tasks
        @type  thread_pool: ThreadPool
        """
        self.nodes = {}
        self.thread_pool = thread_pool

        self.publishers  = Registrations(Registrations.TOPIC_PUBLICATIONS)
        self.subscribers = Registrations(Registrations.TOPIC_SUBSCRIPTIONS)
        self.services = Registrations(Registrations.SERVICE)
        self.param_subscribers = Registrations(Registrations.PARAM_SUBSCRIPTIONS)        

    
    def reverse_lookup(self, caller_api):
        """
        Get a NodeRef by caller_api
        @param caller_api: caller XML RPC URI
        @type  caller_api: str
        @return: nodes that declare caller_api as their
        API. 99.9% of the time this should only be one node, but we
        allow for multiple matches as the master API does not restrict
        this.
        @rtype: [NodeRef]
        """
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
    
    def register_service(self, service, caller_id, caller_api, service_api):
        """
        Register service provider
        @return: None
        """
        self._register(self.services, service, caller_id, caller_api, service_api)
    def register_publisher(self, topic, caller_id, caller_api):
        """
        Register topic publisher
        @return: None
        """
        self._register(self.publishers, topic, caller_id, caller_api)
    def register_subscriber(self, topic, caller_id, caller_api):
        """
        Register topic subscriber
        @return: None
        """
        self._register(self.subscribers, topic, caller_id, caller_api)
    def register_param_subscriber(self, param, caller_id, caller_api):
        """
        Register param subscriber
        @return: None
        """
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
        
    def _register_node_api(self, caller_id, caller_api):
        """
        @param caller_id: caller_id of provider
        @type  caller_id: str
        @param caller_api: caller_api of provider
        @type  caller_api: str
        @return: (registration_information, changed_registration). changed_registration is true if 
        caller_api is differet than the one registered with caller_id
        @rtype: (NodeRef, bool)
        """
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


