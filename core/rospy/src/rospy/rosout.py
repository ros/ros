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

"""Internal use: support for /rosout logging in rospy"""

import logging
import sys
import traceback

import rospy.names 

from rospy.core import add_log_handler, get_caller_id
from rospy.exceptions import ROSException
from rospy.registration import get_topic_manager
from rospy.rostime import Time
from rospy.topics import Publisher, Subscriber

#Log message for rosout
import roslib.msg

_ROSOUT = '/rosout'
_rosout_pub = None

def init_rosout():
    logger = logging.getLogger("rospy.rosout")
    try:
        global _rosout_pub
        if _rosout_pub is None:
            logger.info("initializing %s core topic"%_ROSOUT)
            _rosout_pub = Publisher(_ROSOUT, roslib.msg.Log, latch=True)
            logger.info("connected to core topic %s"%_ROSOUT)
        return True
    except Exception, e:
        logger.error("Unable to initialize %s: %s\n%s", _ROSOUT, e, traceback.format_exc())
        return False

_in_rosout = False
## log an error to the /rosout topic
def _rosout(level, msg):
    global _in_rosout
    try:
        if _rosout_pub is not None:
            # protect against infinite recursion
            if not _in_rosout:
                try:
                    _in_rosout = True
                    msg = str(msg)
                    topics = list(get_topic_manager().get_topics() or [])
                    l = roslib.msg.Log(level=level, name=str(rospy.names.get_caller_id()), msg=str(msg), topics=topics)
                    l.header.stamp = Time.now()
                    _rosout_pub.publish(l)
                finally:
                    _in_rosout = False
    except Exception, e:
        #traceback.print_exc()
        # don't use logerr in this case as that is recursive here
        logger = logging.getLogger("rospy.rosout")        
        logger.error("Unable to report rosout: %s\n%s", e, traceback.format_exc())
        return False

def _rosout_debug(msg):
    _rosout(roslib.msg.Log.DEBUG, msg)
def _rosout_info(msg):
    _rosout(roslib.msg.Log.INFO, msg)
def _rosout_warn(msg):
    _rosout(roslib.msg.Log.WARN, msg)
def _rosout_error(msg):
    _rosout(roslib.msg.Log.ERROR, msg)
def _rosout_fatal(msg):
    _rosout(roslib.msg.Log.FATAL, msg)

## Load loggers for publishing to /rosout
## @param level int: roslib.msg.Log level. Loggers >= level will be loaded.
def load_rosout_handlers(level):
    if roslib.msg.Log.DEBUG >= level:
        add_log_handler(roslib.msg.Log.DEBUG, _rosout_debug)
    if roslib.msg.Log.INFO >= level:
        add_log_handler(roslib.msg.Log.INFO, _rosout_info)
    if roslib.msg.Log.WARN >= level:
        add_log_handler(roslib.msg.Log.WARN, _rosout_warn)
    if roslib.msg.Log.ERROR >= level:
        add_log_handler(roslib.msg.Log.ERROR, _rosout_error)
    if roslib.msg.Log.FATAL >= level:
        add_log_handler(roslib.msg.Log.FATAL, _rosout_fatal)
