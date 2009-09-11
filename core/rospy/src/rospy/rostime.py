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

import threading
import time
import traceback

import rospy.exceptions

import roslib.msg
import roslib.rosenv
import roslib.rostime

## /time support. This hooks into the rospy Time representation and
## allows it to be overriden with data from the /time topic.

_rostime_initialized = False
_rostime_current = None
_rostime_cond = threading.Condition()

# subclass for future-proofing in case we do extend Duration
class Duration(roslib.rostime.Duration): pass

## Time class for rospy. Related to roslib's Time, but has additional
## now() factory method that can initialize Time to the current ROS time.
class Time(roslib.rostime.Time):
    
    ## create new Time instance representing current time
    ## @return Time instance for current time
    def now():
        if not _rostime_initialized:
            raise rospy.exceptions.ROSInitException("time is not initialized. Have you called init_node()?")
        if _rostime_current is not None:
            # initialize with sim time
            return _rostime_current
        else:
            # initialize with wallclock
            float_secs = time.time()
            secs = int(float_secs)
            nsecs = int((float_secs - secs) * 1000000000)
            return Time(secs, nsecs)

    now = staticmethod(now)

    # have to reproduce super class implementation to return correct typing
    
    ## create new Time instance using time.time() value (float
    ## seconds)
    ## @param float_secs: time value in time.time() format
    ## @return Time instance for specified time
    def from_seconds(float_secs):
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        return Time(secs, nsecs)
    
    from_seconds = staticmethod(from_seconds)
    
## Callback to update ROS time from /time
def _set_rostime(t):
    if isinstance(t, roslib.rostime.Time):
        t = Time(t.secs, t.nsecs)
    elif not isinstance(t, Time):
        raise ValueError("must be Time instance")
    global _rostime_current
    _rostime_current = t
    try:
        _rostime_cond.acquire()
        _rostime_cond.notifyAll()
    finally:
        _rostime_cond.release()
    
## \ingroup clientapi
## Get the current time as a Time object    
## @return Time: current time as a rospy.Time object
def get_rostime():
    return Time.now()

## \ingroup clientapi
## Get the current time as float secs (time.time() format)
## @return float: time in secs (time.time() format)    
def get_time():
    return Time.now().to_seconds()

## Mark rostime as initialized. This flag enables other routines to
## throw exceptions if rostime is being used before the underlying
## system is initialized.
## @param val bool: value for initialization state
def set_rostime_initialized(val):
    global _rostime_initialized
    _rostime_initialized = val

## @return True if rostime has been initialized
def is_rostime_initialized():
    return _rostime_initialized    

## internal API for helper routines that need to wait on time updates
## @return threading.Cond: rostime conditional var
def get_rostime_cond():
    return _rostime_cond

## Switch ROS to wallclock time. This is mainly for testing purposes.
def switch_to_wallclock():
    global _rostime_current
    _rostime_current = None
    try:
        _rostime_cond.acquire()
        _rostime_cond.notifyAll()
    finally:
        _rostime_cond.release()

