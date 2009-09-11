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

import sys
import string
import time

import rospy

import roslib.names
import roslib.scriptutil

class RosUtilException(Exception): pass

def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise RosUtilException("remote call failed: %s"%msg)
    return val

## subroutine for getting the topic type
## (identical to rostopic._get_topic_type)
## @return str, str, str: topic type, real topic name, and rest of name referenced
## if the \a topic points to a field within a topic, e.g. /rosout/msg
def _get_topic_type(topic):
    val = _succeed(roslib.scriptutil.get_master().getPublishedTopics('/', '/'))
    matches = [(t, t_type) for t, t_type in val if t == topic or topic.startswith(t+'/')]
    if matches:
        t, t_type = matches[0]
        if t_type == roslib.names.ANYTYPE:
            return None, None, None
        if t_type == topic:
            return t_type, None
        return t_type, t, topic[len(t):]
    else:
        return None, None, None

## get the topic type (identical to rostopic.get_topic_type)
## @return str, str, str: topic type, real topic name, and rest of name referenced
## if the \a topic points to a field within a topic, e.g. /rosout/msg
def get_topic_type(topic):
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    else:
        print >> sys.stderr, "WARNING: topic [%s] does not appear to be published yet"%topic
        while not rospy.is_shutdown():
            topic_type, real_topic, rest = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, rest
            else:
                time.sleep(0.1)


## @param str field_name: name of field to index into
## @param str slot_num: index of slot to return
## @return fn(msg_field)->msg_field[slot_num]
def _array_eval(field_name, slot_num):
    def fn(f):
        return getattr(f, field_name).__getitem__(slot_num)
    return fn

## @param str field_name: name of field to return
## @return fn(msg_field)->msg_field.field_name
def _field_eval(field_name):
    def fn(f):
        return getattr(f, field_name)
    return fn

def generate_field_evals(fields):
    try:
        evals = []
        fields = [f for f in fields.split('/') if f]
        for f in fields:
            if '[' in f:
                field_name, rest = f.split('[')
                slot_num = string.atoi(rest[:rest.find(']')])
                evals.append(_array_eval(field_name, slot_num))
            else:
                evals.append(_field_eval(f))
        return evals
    except Exception, e:
        raise RosUtilException("cannot parse field reference [%s]: %s"%(fields, str(e)))
    
