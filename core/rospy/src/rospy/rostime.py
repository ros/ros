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
# Revision $Id: client.py 2258 2008-09-30 23:03:06Z sfkwc $

import logging
import threading
import time
import traceback

import rospy.core 
import rospy.exceptions
import rospy.init
import rospy.names
import rospy.topics

import roslib.msg
import roslib.rosenv

## /rostime support

_rostime_current = None
_rostime_cond = threading.Condition()
_ROSTIME = '/time'
_USE_SIMTIME = '/use_sim_time'
_rostime_sub = None

def _is_use_simtime():
    # in order to prevent circular dependencies, this does not use the
    # builtin libraries for interacting with the parameter server, at least
    # until I reorganize the client vs. internal APIs better.
    import roslib.rosenv
    master_uri = rospy.init.get_local_master_uri() or roslib.rosenv.get_master_uri()
    m = rospy.core.xmlrpcapi(master_uri)
    code, msg, val = m.getParam(rospy.names.get_caller_id(), _USE_SIMTIME)
    if code == 1 and val:
        return True
    return False
    
## Callback to update ROS time from /time
def _update_rostime(time_msg):
    global _rostime_current
    _rostime_current = time_msg.rostime
    try:
        _rostime_cond.acquire()
        _rostime_cond.notifyAll()
    finally:
        _rostime_cond.release()

## Switch ROS to wallclock time
def switch_to_wallclock():
    global _rostime_current
    _rostime_current = None
    try:
        _rostime_cond.acquire()
        _rostime_cond.notifyAll()
    finally:
        _rostime_cond.release()
    
def init_rostime():
    logger = logging.getLogger("rospy.rostime")
    try:
        global _rostime_sub, _rostime_current
        if _rostime_sub is None:
            logger.info("initializing %s core topic"%_ROSTIME)
            _rostime_sub = rospy.topics.Subscriber(_ROSTIME, roslib.msg.Time, _update_rostime)
            logger.info("connected to core topic %s"%_ROSTIME)

            if _is_use_simtime():
                _rostime_current = rospy.core.Time(0, 0)
        return True
    except Exception, e:
        logger.error("Unable to initialize %s: %s\n%s", _ROSTIME, e, traceback.format_exc())
        return False

## \ingroup clientapi
## Get the current time as a Time object    
## @return Time: current time as a rospy.Time object
def get_rostime():
    if _rostime_current is not None:
        return _rostime_current
    else:
        return rospy.core.Time.now()

## \ingroup clientapi
## Get the current time as float secs (time.time() format)
## @return float: time in secs (time.time() format)    
def get_time():
    if _rostime_current is not None:
        return _rostime_current.to_seconds()
    else:
        return rospy.core.Time.now().to_seconds()

# TODO: may want more specific exceptions for sleep
## \ingroup clientapi
## sleep for the specified duration in ROS time
## @param duration float or Duration: seconds (or rospy.Duration) to sleep
## @throws ROSException if ROS time is set backwards or ROS shutdown
## occurs before sleep completes
def sleep(duration):
    # make a copy of current rostime
    initial_rostime = _rostime_current
    if initial_rostime is None:
        if isinstance(duration, rospy.core.Duration):
            time.sleep(duration.to_seconds())
        else:
            time.sleep(duration)
    else:
        if not isinstance(duration, rospy.core.Duration):
            duration = rospy.core.Duration.from_seconds(duration)
        sleep_t = initial_rostime + duration
        
        # break loop if sleep_t is reached, time moves backwards, or
        # node is shutdown
        while _rostime_current < sleep_t and \
              _rostime_current >= initial_rostime and \
                  not rospy.core.is_shutdown():
            try:
                _rostime_cond.acquire()
                _rostime_cond.wait(0.1)
            finally:
                _rostime_cond.release()

        if _rostime_current < initial_rostime:
            raise rospy.exceptions.ROSException("ROS time moved backwards")
        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSException("ROS shutdown request")
