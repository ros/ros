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
# Revision $Id: topics.py 5218 2009-07-15 23:46:40Z sfkwc $

import roslib.names
import rospy

# Experimental support for Sessions, which are used by openrave

class Session(object):
    
    ## @param session_name str: Name of session
    ## @param session_type ServiceMessage: Service type of session (e.g. openraveros.srv.openrave_session)
    def __init__(self, session_name, session_type):
        self.session_name = rospy.resolve_name(session_name)
        # names for other services are composed using namespace of session
        self.session_ns = roslib.names.namespace(session_name)
        self.session_type = session_type
        self.handles = {}
        self.headers = {}
        self.sessionid = None
        
    ## @return Response: session create response from session server
    ## @throws rospy.ServiceException if Service is not available or
    ## if session response is not recognized (i.e. does not contain a
    ## sessionid field)
    def create(self, session_req):
        handle = rospy.ServiceProxy(self.session_name, self.session_type)
        #TODO: lookup service and introspect other handles
        resp = handle(session_req)
        rospy.loginfo("creating session service for [%s]"%self.session_name)
        if not hasattr(resp, 'sessionid'):
            raise rospy.ServiceException
        self.sessionid = resp.sessionid
        self.headers = { self.session_name : self.sessionid }
        rospy.loginfo("session service [%s] returned sessionid [%s]"%(self.session_name, resp.sessionid))
        return resp

    ## @return str: sessionid of service
    def get_session_id(self):
        return self.sessionid
    
    ## @param service_name str: Name of session service to call
    ## @throws rospy.ServiceException if Service is not available
    ## @throws rospy.SessionException if Session is not initialized
    def call(self, service_name, service_type, *args):
        if not self.headers:
            raise SessionException("Please call create() first")            
        if self.handles is None:
            raise SessionException("Session has been closed")
        if service_name in self.handles:
            return self.handles[service_name](*args)

        # TODO: enable after ROS 0.7 release
        if 0:
            full_service_name = roslib.names.ns_join(self.session_ns, rospy.remap_name(service_name))
        else:
            full_service_name = roslib.names.ns_join(self.session_ns, service_name)
        full_service_name = rospy.resolve_name(full_service_name)

        handle = rospy.ServiceProxy(full_service_name, service_type, persistent=True, headers=self.headers)
        self.handles[service_name] = handle
        return handle(*args)
    
    ## Close all handles and session. NOTE: not thread-safe
    def close(self):
        if self.handles is None:
            return
        for h in self.handles.iteritems():
            h.close()
        del self.handles[:]
        self.handles = None
