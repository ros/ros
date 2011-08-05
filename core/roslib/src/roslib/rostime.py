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
ROS Time representation, including Duration
"""

import itertools
import time
import sys

if sys.version > '3':
	long = int

def _canon(secs, nsecs):
    #canonical form: nsecs is always positive, nsecs < 1 second
    while nsecs >= 1000000000:
        secs += 1
        nsecs -= 1000000000
    while nsecs < 0:
        secs -= 1
        nsecs += 1000000000
    return secs,nsecs

class TVal(object):
    """
    Base class of L{Time} and L{Duration} representations. Representation
    is secs+nanoseconds since epoch.
    """

    __slots__ = ['secs', 'nsecs']
    def __init__(self, secs=0, nsecs=0):
        """
        @param secs: seconds. If secs is a float, then nsecs must not be set or 0
        @type  secs: int/float
        @param nsecs: nanoseconds
        @type  nsecs: int
        """
        if type(secs) != int:
            # float secs constructor
            if nsecs != 0:
                raise ValueError("if secs is a float, nsecs cannot be set")
            float_secs = secs
            secs = int(float_secs)
            nsecs = int((float_secs - secs) * 1000000000)

        self.secs, self.nsecs = _canon(secs, nsecs)

    def is_zero(self):
        """
        @return: True if time is zero (secs and nsecs are zero)
        @rtype: bool
        """
        return self.secs == 0 and self.nsecs == 0
    
    def set(self, secs, nsecs):
        """
        Set time using separate secs and nsecs values
        
        @param secs: seconds since epoch
        @type  secs: int
        @param nsecs: nanoseconds since seconds
        @type  nsecs: int
        """
        self.secs = secs
        self.nsecs = nsecs

    def canon(self):
        """
        Canonicalize the field representation in this instance.  should
        only be used when manually setting secs/nsecs slot values, as
        in deserialization.
        """
        self.secs, self.nsecs = _canon(self.secs, self.nsecs)
        
    def to_sec(self):
        """
        @return: time as float seconds (same as time.time() representation)
        @rtype: float
        """
        return float(self.secs) + float(self.nsecs) / 1e9

    def to_nsec(self):
        """
        @return: time as nanoseconds
        @rtype: long
        """
        return self.secs * long(1e9) + self.nsecs
        
    def __hash__(self):
        """
        Time values are hashable. Time values with identical fields have the same hash.
        """
        return ("%s.%s"%(self.secs, self.nsecs)) .__hash__()

    def __str__(self):
        return str(self.to_nsec())

    def __repr__(self):
        return "rostime.TVal[%d]"%self.to_nsec()

    def __nonzero__(self):
        """
        Check if time value is zero
        """
        return self.secs or self.nsecs

    def __lt__(self, other):
        """
        < test for time values
        """
        try:
            return self.__cmp__(other) < 0
        except TypeError:
            return NotImplemented
    def __le__(self, other):
        """
        <= test for time values
        """
        try:
            return self.__cmp__(other) <= 0
        except TypeError:
            return NotImplemented
    def __gt__(self, other):
        """
        > test for time values
        """
        try:
            return self.__cmp__(other) > 0
        except TypeError:
            return NotImplemented
    def __ge__(self, other):
        """
        >= test for time values
        """
        try:
            return self.__cmp__(other) >= 0
        except TypeError:
            return NotImplemented
    def __ne__(self, other):
        return not self.__eq__(other)
    def __cmp__(self, other):
        if not isinstance(other, TVal):
            raise TypeError("Cannot compare to non-TVal")
        nanos = self.to_nsec() - other.to_nsec()
        if nanos > 0:
            return 1
        if nanos == 0:
            return 0
        return -1
    def __eq__(self, other):
        if not isinstance(other, TVal):
            return False
        return self.to_nsec() == other.to_nsec()

class Time(TVal):
    """
    Time contains the ROS-wide 'time' primitive representation, which
    consists of two integers: seconds since epoch and nanoseconds since
    seconds. Time instances are mutable.
    """
    __slots__ = ['secs', 'nsecs']
    def __init__(self, secs=0, nsecs=0):
        """
        Constructor: secs and nsecs are integers. You may prefer to use the static L{from_sec()} factory
        method instead.
        
        @param secs: seconds since epoch
        @type  secs: int
        @param nsecs: nanoseconds since seconds (since epoch)
        @type  nsecs: int
        """
        super(Time, self).__init__(secs, nsecs)
        if self.secs < 0:
            raise TypeError("time values must be positive")

    def __getstate__(self):
        """
        support for Python pickling
        """
        return [self.secs, self.nsecs]

    def __setstate__(self, state):
        """
        support for Python pickling
        """
        self.secs, self.nsecs = state

    def from_sec(float_secs):
        """
        Create new Time instance using time.time() value (float
        seconds)
        
        @param float_secs: time value in time.time() format
        @type  float_secs: float
        @return: Time instance for specified time
        @rtype: L{Time}
        """
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        return Time(secs, nsecs)
    
    from_sec = staticmethod(from_sec)    

    def to_time(self):
        """
        Get Time in time.time() format. alias of L{to_sec()}
        
        @return: time in floating point secs (time.time() format)
        @rtype: float
        """
        return self.to_sec()

    def __repr__(self):
        return "rostime.Time[%d]"%self.to_nsec()

    def __add__(self, other):
        """
        Add duration to this time
        
        @param other: duration
        @type  other: L{Duration}
        """
        if not isinstance(other, Duration):
            return NotImplemented
        return Time(self.secs + other.secs, self.nsecs + other.nsecs)

    def __sub__(self, other):
        """
        Subtract time or duration from this time
        @param other: duration
        @type  other: L{Duration}/L{Time}
        @return: L{Duration} if other is a L{Time}, L{Time} if other is a L{Duration}
        """
        if isinstance(other, Time):
            return Duration(self.secs - other.secs, self.nsecs - other.nsecs)
        elif isinstance(other, Duration):
            return Time(self.secs - other.secs, self.nsecs - other.nsecs)
        else:
            return NotImplemented

    def __cmp__(self, other):
        """
        Compare to another time
        @param other: Time
        @type  other: L{Time}
        """
        if not isinstance(other, Time):
            raise TypeError("cannot compare to non-Time")
        nanos = self.to_nsec() - other.to_nsec()
        if nanos > 0:
            return 1
        if nanos == 0:
            return 0
        return -1

    def __eq__(self, other):
        """
        Equals test for Time. Comparison assumes that both time
        instances are in canonical representation; only compares fields.
        
        @param other: Time
        @type  other: L{Time}
        """
        if not isinstance(other, Time):
            return False
        return self.secs == other.secs and self.nsecs == other.nsecs

class Duration(TVal):
    """
    Duration represents the ROS 'duration' primitive, which consists
    of two integers: seconds and nanoseconds. The Duration class
    allows you to add and subtract Duration instances, including
    adding and subtracting from L{Time} instances.
    """
    __slots__ = ['secs', 'nsecs']
    def __init__(self, secs=0, nsecs=0):
        """
        Create new Duration instance. secs and nsecs are integers and correspond to the ROS 'duration' primitive.

        @param secs: seconds
        @type  secs: int
        @param nsecs: nanoseconds
        @type  nsecs: int
        """
        super(Duration, self).__init__(secs, nsecs)

    def __getstate__(self):
        """
        support for Python pickling
        """
        return [self.secs, self.nsecs]

    def __setstate__(self, state):
        """
        support for Python pickling
        """
        self.secs, self.nsecs = state

    def __repr__(self):
        return "rostime.Duration[%d]"%self.to_nsec()

    def from_sec(float_seconds):
        """
        Create new Duration instance from float seconds format.
        
        @param float_seconds: time value in specified as float seconds
        @type  float_seconds: float
        @return: Duration instance for specified float_seconds
        @rtype: Duration
        """
        secs = int(float_seconds)
        nsecs = int((float_seconds - secs) * 1000000000)
        return Duration(secs, nsecs)
    
    from_sec = staticmethod(from_sec)    

    def __neg__(self):
        """
        @return: Negative value of this duration
        @rtype:  L{Duration}
        """
        return Duration(-self.secs, -self.nsecs)
    def __abs__(self):
        """
        Absolute value of this duration
        @return: positive duration
        @rtype:  L{Duration}
        """
        if self.secs > 0:
            return self
        return self.__neg__()

    def __add__(self, other):
        """
        Add duration to this duration, or this duration to a time, creating a new time value as a result.
        @param other: duration or time
        @type  other: L{Duration}/L{Time}
        @return: L{Duration} if other is a L{Duration}, L{Time} if other is a L{Time}
        """
        if isinstance(other, Time):
            return other.__add__(self)
        elif isinstance(other, Duration):
            return Duration(self.secs+other.secs, self.nsecs+other.nsecs)
        else:
            return NotImplemented
    def __sub__(self, other):
        """
        Subtract duration from this duration, returning a new duration
        @param other: duration
        @type  other: L{Duration}
        @return: L{Duration} 
        """
        if not isinstance(other, Duration):
            return NotImplemented
        return Duration(self.secs-other.secs, self.nsecs-other.nsecs)        

    def __mul__(self, val):
        """
        Multiply this duration by an integer or float
        @param val: multiplication factor
        @type  val: int/float
        @return: Duration multiplied by val
        @rtype:  L{Duration}
        """
        t = type(val)
        if t in (int, long):
            return Duration(self.secs * val, self.nsecs * val)
        elif t == float:
            return Duration.from_sec(self.to_sec() * val)
        else:
            return NotImplemented

    def __floordiv__(self, val):
        """
        Floor divide this duration by an integer or float
        @param val: division factor
        @type  val: int/float
        @return: Duration multiplied by val
        @rtype:  L{Duration}
        """
        t = type(val)
        if t in (int, long):
            return Duration(self.secs // val, self.nsecs // val)
        elif t == float:
            return Duration.from_sec(self.to_sec() // val)
        else:
            return NotImplemented

    def __div__(self, val):
        """
        Divide this duration by an integer or float
        @param val: division factor
        @type  val: int/float
        @return: Duration multiplied by val
        @rtype:  L{Duration}
        """
        # unlike __floordiv__, this uses true div for float arg
        t = type(val)
        if t in (int, long):
            return Duration(self.secs // val, self.nsecs // val)
        elif t == float:
            return Duration.from_sec(self.to_sec() / val)
        else:
            return NotImplemented

    def __truediv__(self, val):
        """
        Divide this duration by an integer or float
        @param val: division factor
        @type  val: int/float
        @return: Duration multiplied by val
        @rtype:  L{Duration}
        """
        t = type(val)
        if t in (int, long):
            return Duration(self.secs / val, self.nsecs / val)
        elif t == float:
            return Duration.from_sec(self.to_sec() / val)
        else:
            return NotImplemented

    def __cmp__(self, other):
        if not isinstance(other, Duration):
            raise TypeError("Cannot compare to non-Duration")
        nanos = self.to_nsec() - other.to_nsec()
        if nanos > 0:
            return 1
        if nanos == 0:
            return 0
        return -1

    def __eq__(self, other):
        if not isinstance(other, Duration):
            return False
        return self.secs == other.secs and self.nsecs == other.nsecs
