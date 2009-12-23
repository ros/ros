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
# Revision $Id: rosweb.py 4149 2009-04-13 17:00:09Z sfkwc $

import roslib; roslib.load_manifest('rosjson')

import cStringIO
import os

import rospy

class ROSJSONException(rospy.ROSException): pass

_JSON_ESCAPE = {'\\':r'\\', '"':'\\"', '\b':r'\b', '\f':r'\f', '\n':r'\n', '\r':r'\r', '\t':r'\t'}

## Convert value to JSON representation
## @param v Any: value to convert to JSON. Supported types are Python primitives (str, int, float, long, bool) as well as rospy Message-related types (Message, Time, Duration).
## @return str: JSON string representation of \a v
## @throws ROSJSONException if \a msg cannot be converted to JSON
def value_to_json(v):
    if type(v) == str:
        buff = cStringIO.StringIO()
        
        buff.write('"')
        for c in v:
            if c in _JSON_ESCAPE:
                buff.write(_JSON_ESCAPE[c])
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
    elif isinstance(v, (roslib.rostime.Time, roslib.rostime.Duration)):
        return v.to_sec()
    else:
        raise ROSJSONException("unknown type: %s"%type(v))
        
## Convert ROS message to JSON representation. JSON representation is
## a simple dictionary of dictionaries, where a dictionary represents
## a ROS message. Time and duration are represented by their float value
## in seconds.
##
## @param msg rospy.Message: message instance to convert
## @throws ROSJSONException if \a msg cannot be converted to JSON
def ros_message_to_json(msg):
    if not isinstance(msg, rospy.Message):
        raise ROSJSONException("not a valid rospy Message instance: %s"%msg.__class__.__name__)
    buff = cStringIO.StringIO()
    buff.write('{')
    buff.write(','.join(['"%s": %s'%(f, value_to_json(getattr(msg, f))) for f in msg.__slots__]))
    buff.write('}')
    return buff.getvalue()
