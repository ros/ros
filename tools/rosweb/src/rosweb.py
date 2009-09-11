#!/usr/bin/env python
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

import roslib; roslib.load_manifest('rosweb')

import cStringIO
import os
import string
import sys
import threading
import time
import traceback
import BaseHTTPServer

import roslib.scriptutil
import rospy

CALLER_ID = '/rosweb'

class ROSWebException(Exception): pass

## Factory for ROSWebTopic instances
class RWTFactory(object):
    def __init__(self):
        self.map = {}
        self.lock = threading.Lock()

    def get(self, topic):
        try:
            self.lock.acquire()
            if topic in self.map:
                return self.map[topic]
            self.map[topic] = t = ROSWebTopic(topic)
        finally:
            self.lock.release()
        return t

class ROSWebTopic(object):
    def __init__(self, topic):
        self.topic = topic
        self.cond = threading.Condition()
        self.initialized = False
        self.sub = None

    def init(self):
        try:
            self.cond.acquire()
            if self.initialized:
                return
            m = roslib.scriptutil.get_master()
            code, _, topics = m.getPublishedTopics(CALLER_ID, '/')
            if code != 1:
                raise ROSWebException("unable to communicate with master")
            
            for t, topic_type in topics:
                if t == self.topic:
                    break
            else:
                raise ROSWebException("%s is not a published topic"%self.topic)
            
            msg_class = roslib.scriptutil.get_message_class(topic_type)
            self.sub = rospy.Subscriber(self.topic, msg_class, self.topic_callback)
            self.initialized = True
        except ROSWebException:
            raise
        except Exception, e:
            # fatal for now
            rospy.signal_shutdown(str(e))
            raise
        finally:
            self.cond.release()

    def topic_callback(self, data):
        try:
            self.cond.acquire()
            self.last_message = data
            self.cond.notifyAll()
        finally:
            self.cond.release()

class ROSWebHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def __init__(self, *args):
        self.factory = RWTFactory()
        BaseHTTPServer.BaseHTTPRequestHandler.__init__(self, *args)

    def _do_GET_topic(self, topic):
        rwt = self.factory.get(topic)
        try:
            rwt.init()
        except ROSWebException, e:
            traceback.print_exc()
            self.send_response(404)
            return
        try:
            print "waiting"
            rwt.cond.acquire()
            # timeout?
            rwt.cond.wait()
            # get the last message under lock
            msg = rwt.last_message
            print "done waiting", msg
        finally:
            rwt.cond.release()
            
        self.send_response(200)
        # serialize message to JSON
        resp = ros_message_to_json(msg)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(resp)))
        self.end_headers()
        self.wfile.write(ros_message_to_json(msg))

    def do_GET(self):
        topic = self.path
        print "TOPIC", topic
        if topic in ['/', '/favicon.ico']:
            self.send_response(404)
            return
        if is_topic(topic):
            self._do_GET_topic(topic)
        else:
            self.send_response(404)
            return

def is_topic(s):
    if not s:
        return False
    # normalize
    if s[0] != '/':
        s = '/' + s
    if '.' in s:
        return False
    return True
        
        
_JSON_ESCAPE = ['\\', '"', '\/', '\b', '\f', '\n', '\r', '\t']

def value_to_json(v):
    if type(v) == str:
        buff = cStringIO.StringIO()
        buff.write('"')
        for c in v:
            if c in _JSON_ESCAPE:
                buff.write('\\'+c)
            else:
                buff.write(c)
        buff.write('"')                
        return buff.getvalue()
    elif type(v) in (int, float, long):
        return "%s"%v
    elif type(v) in (list, tuple):
        return '['+','.join([value_to_json(x) for x in v]) + ']'
    elif type(v) == bool:
        if v:
            return 'true'
        else:
            return 'false'
    elif isinstance(v, rospy.Message):
        return ros_message_to_json(v)
    elif isinstance(v, rospy.Time) or isinstance(v, rospy.Duration):
        return v.to_seconds()
    else:
        raise ROSWebException("unknown type: %s"%type(v))
        
def ros_message_to_json(msg):
    buff = cStringIO.StringIO()
    buff.write('{')
    buff.write(','.join(['"%s": %s'%(f, value_to_json(getattr(msg, f))) for f in msg.__slots__]))
    buff.write('}')
    return buff.getvalue()

def rosweb_main():
    try:
        try:
            rospy.init_node('rosweb', disable_signals=True)
            server = BaseHTTPServer.HTTPServer(('', 8080), ROSWebHandler)

            # unfortunately this only responds to ctrl-C. It appears
            # that BaseHTTPServer does not respect the handle_request
            # timeout in Python 2.5.
            print "starting Web server"
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    finally:
        print "exiting"
        rospy.signal_shutdown('rosweb exiting')
        server.server_close()

if __name__ == '__main__':
    rosweb_main()
