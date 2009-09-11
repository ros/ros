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

import logging
import traceback

import roslib.scriptutil

from rospy.core import *
from rospy.registration import set_service_manager, Registration, get_registration_listeners
from rospy.transport import *

logger = logging.getLogger('rospy.service')

## alias of roslib.message.ServiceDefinition
import roslib.message
ServiceDefinition = roslib.message.ServiceDefinition

## \ingroup clientapi
## Exception class for service errors
class ServiceException(Exception): pass

## superclass for storing service information
class _Service(object):
    def __init__(self, name, service_class):
        self.name = resolve_name(name) #services remap as well
        self.service_class = service_class
        self.request_class = service_class._request_class
        self.response_class = service_class._response_class
        self.uri = None #initialize attr

## Keeps track of currently registered services in the ROS system
class ServiceManager(object):
    def __init__(self):
        self.map = {} # {name : Service}
        self.lock = threading.RLock()

    ## @param self
    ## @return [(str, str)]: List of (service_name, service_uri)  for all registered services.
    def get_services(self):
        try:
            self.lock.acquire()
            ret_val = []
            for name, service in self.map.iteritems():
                ret_val.append((name, service.uri))
            services = self.map.values()
        finally:
            self.lock.release()
        return ret_val
    
    ## Unregister all registered services
    ## @param self 
    def unregister_all(self):
        self.map.clear()
    
    ## Register service with ServiceManager and ROS master
    ## @param self
    ## @param service_name str: name of service (resolved)
    ## @param service Service        
    def register(self, service_name, service):
        err = None
        try:
            self.lock.acquire()
            if service_name in self.map:
                err = "service [%s] already registered"%service_name
            else:
                self.map[service_name] = service
                
            # NOTE: this call can potentially take a long time under lock and thus needs to be reimplmented
            get_registration_listeners().notify_added(service_name, service.uri, Registration.SRV)
        finally:
            self.lock.release()

        if err:
            raise ServiceException(err)
        
    ## Unregister service with ServiceManager and ROS Master
    ## @param self
    ## @param service_name str: name of service
    ## @param service Service: service implementation
    def unregister(self, service_name, service):
        try:
            self.lock.acquire()
            curr = self.map.get(service_name, None)
            if curr == service:
                del self.map[service_name]
                
            # NOTE: this call can potentially take a long time under lock
            get_registration_listeners().notify_removed(service_name, service.uri, Registration.SRV)                
        finally:
            self.lock.release()

    ## @param self
    ## @param service_name str: name of service
    ## @return Service: service implementation
    def get_service(self, service_name):
        return self.map.get(service_name, None)

set_service_manager(ServiceManager())
