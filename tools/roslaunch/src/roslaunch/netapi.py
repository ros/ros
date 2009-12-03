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
Convience methods for manipulating XML-RPC APIs
"""

import socket
import xmlrpclib

import roslib.network
import roslib.scriptutil

_ID = '/roslaunch-netapi'
def get_roslaunch_uris():
    """
    @return: list of roslaunch XML-RPC URIs for roscore that's in
    the current environment, or None if roscore cannot be contacted.
    @rtype: [str]
    """
    try:
        param_server = roslib.scriptutil.get_param_server()
        code, msg, vals = param_server.getParam(_ID, '/roslaunch/uris')
        if code == 1 and vals:
            return vals.values()
    except socket.error: pass
    return None

class NetProcess(object):
    def __init__(self, name, respawn_count, is_alive, roslaunch_uri):
        self.is_alive = is_alive
        self.respawn_count = respawn_count
        self.name = name
        
        self.roslaunch_uri = roslaunch_uri
        self.machine, _ = roslib.network.parse_http_host_and_port(roslaunch_uri)

def list_processes(roslaunch_uris=None):
    """
    @param roslaunch_uris: (optional) list of XML-RPCS. If none
        are provided, will look up URIs dynamically
    @type  roslaunch_uris: [str]
    @return: list of roslaunch processes
    @rtype: [L{NetProcess}]
    """
    if not roslaunch_uris:
        roslaunch_uris = get_roslaunch_uris()
        if not roslaunch_uris:
            return []

    procs = []
    for uri in roslaunch_uris:
        try:
            r = xmlrpclib.ServerProxy(uri)
            code, msg, val = r.list_processes()
            if code == 1:
                active, dead = val
                procs.extend([NetProcess(a[0], a[1], True, uri) for a in active])
                procs.extend([NetProcess(d[0], d[1], False, uri) for d in dead])
        except:
            #import traceback
            #traceback.print_exc()
            # don't have a mechanism for reporting these errors upwards yet
            pass 
    return procs

if __name__ == "__main__":
    import roslib; roslib.load_manifest('roslaunch')
    print list_processes()
    print "done"
    
