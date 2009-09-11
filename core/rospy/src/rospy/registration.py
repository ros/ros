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

import socket
import sys
import logging
import thread
import threading
import time
import traceback

import rospy.core
from rospy.core import is_shutdown, xmlrpcapi, logfatal, logwarn, loginfo, logerr, logdebug
from rospy.names import get_caller_id, get_namespace, resolve_name

## Module handles maintaining registrations with master via internal listener APIs

# topic manager and service manager singletons

_topic_manager = None
def set_topic_manager(tm):
    global _topic_manager
    _topic_manager = tm
def get_topic_manager():
    return _topic_manager

_service_manager = None
def set_service_manager(sm):
    global _service_manager
    _service_manager = sm
def get_service_manager():
    return _service_manager

## Registration types
class Registration(object):
    PUB = 'pub'
    SUB = 'sub'
    SRV = 'srv'
    
## Listener API for subscribing to changes in Publisher/Subscriber/Service declarations
class RegistrationListener(object):
    """Listener API for subscribing to topic decl changes in the topic manager"""

    ## New pub/sub/service declared.
    ## @param self
    ## @param name: topic/service name
    ## @param data_type_or_uri str: topic type or service uri
    ## @param reg_type {Registration.PUB | Registration.SUB | Registration.SRV}
    def reg_added(self, name, data_type_or_uri, reg_type): 
        pass
    
    ## New pub/sub/service removed.
    ## @param self
    ## @param name: topic/service name
    ## @param data_type_or_uri str: topic type or service uri
    ## @param reg_type {Registration.PUB | Registration.SUB | Registration.SRV}
    def reg_removed(self, name, data_type_or_uri, reg_type): 
        pass

class _RegistrationListeners(object):
    
    ## ctor
    ## @param self
    def __init__(self):
        self.listeners = []
        self.lock = threading.Lock()

    ## Subscribe to notifications of pub/sub/service registration
    ## changes. This is an internal API used to notify higher level
    ## routines when to communicate with the master.
    ##
    ## @param self
    ## @param l TopicListener: listener to subscribe
    def add_listener(self, l):
        assert isinstance(l, RegistrationListener)
        try:
            self.lock.acquire()
            self.listeners.append(l)
        finally:
            self.lock.release()

    ## @param self
    ## @param name str: topic/service name
    ## @param data_type_or_uri str: topic type or service uri
    ## @param reg_type str: {Registration.PUB | Registration.SUB | Registration.SRV}
    def notify_removed(self, name, data_type_or_uri, reg_type):
        try:
            self.lock.acquire()
            for l in self.listeners:
                try:
                    l.reg_removed(name, data_type_or_uri, reg_type)
                except Exception, e:
                    logerr("error notifying listener of removal: %s"%traceback.format_exc(e))
        finally:
            self.lock.release()
            
    ## @param self
    ## @param name str; topic/service name
    ## @param data_type str: topic/service type
    ## @param reg_type str: {Registration.PUB | Registration.SUB | Registration.SRV}
    def notify_added(self, name, data_type, reg_type):
        try:
            self.lock.acquire()
            for l in self.listeners:
                try:
                    l.reg_added(name, data_type, reg_type)
                except Exception, e:
                    logerr(traceback.format_exc(e))
        finally:
            self.lock.release()
            
_registration_listeners = _RegistrationListeners()
def get_registration_listeners():
    return _registration_listeners

# RegManager's main purpose is to collect all client->master communication in one place

## Registration manager. Instantiated by slave nodes. Communicates
## with master to maintain topic registration information. Also
## responds to publisher updates to create topic connections
class RegManager(RegistrationListener):

    ## @param self
    ## @param handler node API handler
    def __init__(self, handler):
        self.logger = logging.getLogger("rospy.registration")
        self.handler = handler
        self.uri = self.master_uri = None
        self.updates = []
        self.cond = threading.Condition() #for locking/notifying updates
        self.registered = False        
        rospy.core.add_shutdown_hook(self.cleanup)
        
    ## Start the RegManager. This should be passed in as an argument to a thread
    ## starter as the RegManager is designed to spin in its own thread
    ## @param self
    ## @param uri str: URI of local node
    ## @param master_uri str: Master URI
    def start(self, uri, master_uri):
        self.registered = False 
        self.master_uri = master_uri
        self.uri = uri
        first = True
        tm = get_topic_manager()
        sm = get_service_manager()
        ns = get_namespace()
        caller_id = get_caller_id()
        if not master_uri or master_uri == uri:
            registered = True
            master = None
        else:
            registered = False
            master = xmlrpcapi(master_uri)
            self.logger.info("Registering with master node %s", master_uri)

        while not registered and not is_shutdown():
            try:
                try:
                    # prevent TopicManager and ServiceManager from accepting registrations until we are done
                    tm.lock.acquire()
                    sm.lock.acquire()                    

                    pub, sub, srv = tm.get_publications(), tm.get_subscriptions(), sm.get_services()
                    for name, data_type in pub:
                        name = resolve_name(name, caller_id)
                        self.logger.info("Registering publisher topic [%s] type [%s] with master", name, data_type)
                        code, msg, val = master.registerPublisher(caller_id, name, data_type, uri)
                        if code != 1:
                            logfatal("cannot register publication topic [%s] with master: %s"%(name, msg))
                            rospy.core.signal_shutdown("master/node incompatibility with register publisher")
                    for name, data_type in sub:
                        name = resolve_name(name, caller_id)
                        self.logger.info("registering subscriber topic [%s] type [%s] with master", name, data_type)
                        code, msg, val = master.registerSubscriber(caller_id, name, data_type, uri)
                        if code != 1:
                            logfatal("cannot register subscription topic [%s] with master: %s"%(name, msg))
                            rospy.core.signal_shutdown("master/node incompatibility with register subscriber")                        
                        else:
                            self.publisher_update(name, val)
                    for name, service_uri in srv:
                        name = resolve_name(name, caller_id)
                        self.logger.info("registering service [%s] uri [%s] with master", name, service_uri)
                        code, msg, val = master.registerService(caller_id, name, service_uri, uri)
                        if code != 1:
                            logfatal("cannot register service [%s] with master: %s"%(name, msg))
                            rospy.core.signal_shutdown("master/node incompatibility with register service")                        
 
                    registered = True
                    
                    # Subscribe to updates to our state
                    get_registration_listeners().add_listener(self)
                finally:
                    sm.lock.release()                    
                    tm.lock.release()
                
                if pub or sub:
                    logdebug("Registered [%s] with master node %s", caller_id, master_uri)
                else:
                    logdebug("No topics to register with master node %s", master_uri)
                    
            except Exception, e:
                if first:
                    # this use to print to console always, arguable whether or not this should be subjected to same configuration options as logging
                    logerr("Unable to immediately register with master node [%s]: master may not be running yet. Will keep trying."%master_uri)
                    first = False
                time.sleep(0.2)
        self.registered = True
        self.run()
        
    ## @param self
    ## @return bool: True if registration has occurred with master
    def is_registered(self):
        return self.registered 

    ## Main RegManager thread loop. Periodically checks the update
    ## queue and generates topic connections
    ## @param self
    def run(self):
        "Main RegManager thread loop."
        #Connect the topics
        while not self.handler.done and not is_shutdown():
            cond = self.cond
            try:
                cond.acquire()
                if not self.updates:
                    cond.wait(0.5)
                if self.updates:
                    #work from the end as these are the most up-to-date
                    topic, uris = self.updates.pop()
                    #filter out older updates for same topic
                    self.updates = [x for x in self.updates if x[0] != topic]
                else:
                    topic = uris = None
            finally:
                if cond is not None:
                    cond.release()

            #call _connect_topic on all URIs as it can check to see whether
            #or not a connection exists.
            if uris and not self.handler.done:
                for uri in uris:
                    # #1141: have to multithread this to prevent a bad publisher from hanging us
                    thread.start_new_thread(self._connect_topic_thread, (topic, uri))

    def _connect_topic_thread(self, topic, uri):
        try:
            code, msg, _ = self.handler._connect_topic(topic, uri)
            if code != 1:
                logerr("Unable to connect subscriber to publisher [%s] for topic [%s]: %s", uri, topic, msg)
        except Exception, e:
            if not is_shutdown():
                logerr("Unable to connect to publisher [%s] for topic [%s]: %s"%(uri, topic, traceback.format_exc()))
        
    ## Cleans up registrations with master and releases topic and service resources
    ## @param self
    ## @param reason str: human-reasonable debug string
    def cleanup(self, reason):
        try:
            self.cond.acquire()
            self.cond.notifyAll()
        finally:
            self.cond.release()        

        # we never successfully initialized master_uri
        if not self.master_uri:
            return
        
        master = xmlrpcapi(self.master_uri)
        # we never successfully initialized master
        if master is None:
            return
        
        caller_id = get_caller_id()

        tm = get_topic_manager()
        sm = get_service_manager()
        try:
            if tm is not None:
                for name, _ in tm.get_subscriptions():
                    master.unregisterSubscriber(caller_id, name, self.uri)
                for name, _ in tm.get_publications():
                    master.unregisterPublisher(caller_id, name, self.uri)

            if sm is not None:
                for name, service_uri in sm.get_services():
                    master.unregisterService(caller_id, name, service_uri)
        except socket.error, (errno, msg):
            if errno == 111 or errno == 61: #can't talk to master, nothing we can do about it
                self.logger.warn("cannot unregister with master due to network issues")
            else:
                self.logger.warn("unclean shutdown\n%s"%traceback.format_exc())
        except:
            self.logger.warn("unclean shutdown\n%s"%traceback.format_exc())

        #TODO: cleanup() should actually be orchestrated by a separate cleanup routine that calls the reg manager/sm/tm
        if tm is not None:
            tm.remove_all()
        if sm is not None:
            sm.unregister_all()

    ## RegistrationListener callback
    ## @param self
    ## @param name str: name of topic or service
    ## @param data_type_or_uri str: either the data type (for topic regs) or the service URI (for service regs).
    ## @param reg_type Registration.PUB | Registration.SUB | Registration.SRV
    def reg_removed(self, name, data_type_or_uri, reg_type):
        master_uri = self.master_uri
        if not master_uri:
            self.logger.error("Registrar: master_uri is not set yet, cannot inform master of deregistration")
        else:
            master = xmlrpcapi(master_uri)
            if reg_type == Registration.PUB:
                self.logger.debug("unregisterPublisher(%s, %s)", name, self.uri)
                master.unregisterPublisher(get_caller_id(), name, self.uri)
            elif reg_type == Registration.SUB:            
                self.logger.debug("unregisterSubscriber(%s, %s)", name, data_type_or_uri)
                master.unregisterSubscriber(get_caller_id(), name, self.uri)
            elif reg_type == Registration.SRV:
                self.logger.debug("unregisterService(%s, %s)", name, data_type_or_uri)
                master.unregisterService(get_caller_id(), name, data_type_or_uri)
    
    ## RegistrationListener callback
    ## @param self
    ## @param name str: name of topic or service
    ## @param data_type_or_uri str: either the data type (for topic regs) or the service URI (for service regs).
    ## @param reg_type Registration.PUB | Registration.SUB | Registration.SRV
    def reg_added(self, name, data_type_or_uri, reg_type):
        #TODO: this needs to be made robust to master outages
        master_uri = self.master_uri
        if not master_uri:
            self.logger.error("Registrar: master_uri is not set yet, cannot inform master of registration")
        else:
            master = xmlrpcapi(master_uri)
            args = (get_caller_id(), name, data_type_or_uri, self.uri)
            registered = False
            first = True
            while not registered and not is_shutdown():
                try:
                    if reg_type == Registration.PUB:
                        self.logger.debug("master.registerPublisher(%s, %s, %s, %s)"%args)
                        code, msg, val = master.registerPublisher(*args)
                        if code != 1:
                            logfatal("unable to register publication [%s] with master: %s"%(name, msg))
                    elif reg_type == Registration.SUB:
                        self.logger.debug("master.registerSubscriber(%s, %s, %s, %s)"%args)
                        code, msg, val = master.registerSubscriber(*args)
                        if code == 1:
                            self.publisher_update(name, val)
                        else:
                            # this is potentially worth exiting over. in the future may want to add a retry
                            # timer
                            logfatal("unable to register subscription [%s] with master: %s"%(name, msg))
                    elif reg_type == Registration.SRV:
                        self.logger.debug("master.registerService(%s, %s, %s, %s)"%args)
                        code, msg, val = master.registerService(*args)
                        if code != 1:
                            logfatal("unable to register service [%s] with master: %s"%(name, msg))
                        
                    registered = True
                except Exception, e:
                    if first:
                        msg = "Unable to register with master node [%s]: master may not be running yet. Will keep trying."%master_uri
                        self.logger.error(str(e)+"\n"+msg)
                        print msg
                        first = False
                    time.sleep(0.2)

    ## Inform psmanager of latest publisher list for a topic.  This
    #  will cause RegManager to create a topic connection for all new
    #  publishers (in a separate thread).
    ## @param self
    #  @param topic Topic name
    #  @param uris list of all publishers uris for topic
    def publisher_update(self, topic, uris):
        "Inform psmanager of latest publisher list for a topic."
        try:
            self.cond.acquire()
            self.updates.append((topic, uris))
            self.cond.notifyAll()              
        finally:
            self.cond.release()
