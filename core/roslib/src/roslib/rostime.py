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
# Revision $Id: rostime.py 3461 2009-01-21 02:12:16Z sfkwc $

import time

## Time representation

def _canon(secs, nsecs):
    #canonical form: nsecs is always positive, nsecs < 1 second
    while nsecs >= 1000000000:
        secs += 1
        nsecs -= 1000000000
    while nsecs < 0:
        secs -= 1
        nsecs += 1000000000
    return secs,nsecs

## Base class of Time and Duration representations. Representation
## is secs+nanoseconds since epoch.
class _TVal(object):
    __slots__ = ['secs', 'nsecs']
    ## @param secs int/float: seconds. If secs is a float, then nsecs must not be set or 0
    ## @param nsecs int: nanoseconds
    def __init__(self, secs=0, nsecs=0):
        if type(secs) != int:
            # float secs constructor
            if nsecs != 0:
                raise ValueError("if secs is a float, nsecs cannot be set")
            float_secs = secs
            secs = int(float_secs)
            nsecs = int((float_secs - secs) * 1000000000)

        self.secs, self.nsecs = _canon(secs, nsecs)

    ## @return True if time is zero (secs and nsecs are zero)
    def is_zero(self):
        return self.secs == 0 and self.nsecs == 0
    
    ## set time using separate secs and nsecs values
    def set(self, secs, nsecs):
        self.secs = secs
        self.nsecs = nsecs

    ## canonicalize the field representation in this instance.  should
    #  only be used when manually setting secs/nsecs slot values, as
    #  in deserialization.
    def canon(self):
        self.secs, self.nsecs = _canon(self.secs, self.nsecs)
        
    def to_seconds(self):
        return float(self.secs) + float(self.nsecs) / 1e9
    def tons(self):
        return self.secs * long(1e9) + self.nsecs

    def __str__(self):
        return str(self.tons())

    def __repr__(self):
        return "rostime._TVal[%d]"%self.tons()

    def __nonzero__(self):
        return self.secs or self.nsecs

    def __lt__(self, other):
        try:
            return self.__cmp__(other) < 0
        except TypeError:
            return NotImplemented
    def __le__(self, other):
        try:
            return self.__cmp__(other) <= 0
        except TypeError:
            return NotImplemented
    def __gt__(self, other):
        try:
            return self.__cmp__(other) > 0
        except TypeError:
            return NotImplemented
    def __ge__(self, other):
        try:
            return self.__cmp__(other) >= 0
        except TypeError:
            return NotImplemented
    def __ne__(self, other):
        return not self.__eq__(other)
    def __cmp__(self, other):
        if not isinstance(other, _TVal):
            raise TypeError("Cannot compare to non-TVal")
        nanos = self.tons() - other.tons()
        if nanos > 0:
            return 1
        if nanos == 0:
            return 0
        return -1
    def __eq__(self, other):
        if not isinstance(other, _TVal):
            return False
        return self.tons() == other.tons()

## \ingroup clientapi
## Time contains the ROS-wide 'time' primitive representation, which
## consists of two integers: seconds since epoch and nanoseconds since
## seconds. Time instances are mutable.
class Time(_TVal):
    __slots__ = ['secs', 'nsecs']
    def __init__(self, secs=0, nsecs=0):
        super(Time, self).__init__(secs, nsecs)
        if self.secs < 0:
            raise TypeError("time values must be positive")

    ## create new Time instance using time.time() value (float
    ## seconds)
    ## @param float_secs: time value in time.time() format
    ## @return Time instance for specified time
    def from_seconds(float_secs):
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        return Time(secs, nsecs)
    
    from_seconds = staticmethod(from_seconds)

    ## get Time in time.time() format. alias of to_seconds()
    ## @return float: time in floating point secs (time.time() format)
    def to_time(self):
        return self.to_seconds()

    def __repr__(self):
        return "rostime.Time[%d]"%self.tons()

    def __add__(self, other):
        if not isinstance(other, Duration):
            return NotImplemented
        return Time(self.secs + other.secs, self.nsecs + other.nsecs)

    def __sub__(self, other):
        if isinstance(other, Time):
            return Duration(self.secs - other.secs, self.nsecs - other.nsecs)
        elif isinstance(other, Duration):
            return Time(self.secs - other.secs, self.nsecs - other.nsecs)
        else:
            return NotImplemented


    def __cmp__(self, other):
        if not isinstance(other, Time):
            raise TypeError("cannot compare to non-Time")
        nanos = self.tons() - other.tons()
        if nanos > 0:
            return 1
        if nanos == 0:
            return 0
        return -1

    def __eq__(self, other):
        if not isinstance(other, Time):
            return False
        return self.secs == other.secs and self.nsecs == other.nsecs

## \ingroup clientapi
## representation of ROS 'duration' primitive    
class Duration(_TVal):
    __slots__ = ['secs', 'nsecs']
    def __init__(self, secs=0, nsecs=0):
        super(Duration, self).__init__(secs, nsecs)

    def __repr__(self):
        return "rostime.Duration[%d]"%self.tons()

    ## create new Duration instance from float seconds format
    ## @param float_seconds: time value in specified as float seconds
    ## @return Duration instance for specified float_seconds
    def from_seconds(float_seconds):
        secs = int(float_seconds)
        nsecs = int((float_seconds - secs) * 1000000000)
        return Duration(secs, nsecs)
    
    from_seconds = staticmethod(from_seconds)

    def __neg__(self):
        return Duration(-self.secs, -self.nsecs)
    def __abs__(self):
        if self.secs > 0:
            return self
        return self.__neg__()

    def __add__(self, other):
        if isinstance(other, Time):
            return other.__add__(self)
        elif isinstance(other, Duration):
            return Duration(self.secs+other.secs, self.nsecs+other.nsecs)
        else:
            return NotImplemented
    def __sub__(self, other):
        if not isinstance(other, Duration):
            return NotImplemented
        return Duration(self.secs-other.secs, self.nsecs-other.nsecs)        

    def __mul__(self, val):
        if not type(val) == int:
            return NotImplemented
        return Duration(self.secs * val, self.nsecs * val)

    def __cmp__(self, other):
        if not isinstance(other, Duration):
            raise TypeError("Cannot compare to non-Duration")
        nanos = self.tons() - other.tons()
        if nanos > 0:
            return 1
        if nanos == 0:
            return 0
        return -1

    def __eq__(self, other):
        if not isinstance(other, Duration):
            return False
        return self.secs == other.secs and self.nsecs == other.nsecs
