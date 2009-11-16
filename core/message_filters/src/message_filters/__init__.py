# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Message Filter Objects
======================
"""

import threading
import rospy

class SimpleFilter:

    def __init__(self):
        self.callbacks = {}

    def registerCallback(self, cb, *args):
        """
        Register a callback function `cb` to be called when this filter
        has output.
        The filter calls the function ``cb`` with a filter-dependent list of arguments,
        followed by the call-supplied arguments ``args``.
        """

        conn = len(self.callbacks)
        self.callbacks[conn] = (cb, args)
        return conn

    def signalMessage(self, *msg):
        for (cb, args) in self.callbacks.values():
            cb(*(msg + args))

class Subscriber(SimpleFilter):
    
    """
    ROS subscription filter.  Identical arguments as :class:`rospy.Subscriber`.

    This class acts as a highest-level filter, simply passing messages
    from a ROS subscription through to the filters which have connected
    to it.
    """
    def __init__(self, *args):
        SimpleFilter.__init__(self)
        self.topic = args[0]
        self.sub = rospy.Subscriber(*args, **{"callback" : self.callback})

    def callback(self, msg):
        self.signalMessage(msg)

    def getTopic(self):
        return self.topic

class Cache(SimpleFilter):

    """
    Stores a time history of messages.

    Given a stream of messages, the most recent ``cache_size`` messages
    are cached in a ring buffer, from which time intervals of the cache
    can then be retrieved by the client.
    """

    def __init__(self, f, cache_size = 1):
        SimpleFilter.__init__(self)
        self.connectInput(f)
        self.cache_size = cache_size

    def connectInput(self, f):
        self.incoming_connection = f.registerCallback(self.add)

    def add(self, msg):
        # Add msg to cache... XXX TODO

        self.signalMessage(msg)

class TimeSynchronizer(SimpleFilter):

    """
    Synchronizes messages by their timestamps.

    :class:`TimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. TimeSynchronizer
    listens on multiple input message filters ``fs``, and invokes the callback
    when it has a collection of messages with matching timestamps.

    The signature of the callback function is::

        def callback(msg1, ... msgN):

    where N is the number of input message filters, and each message is
    the output of the corresponding filter in ``fs``.
    The required ``queue size`` parameter specifies how many sets of
    messages it should store from each input filter (by timestamp)
    while waiting for messages to arrive and complete their "set".
    """

    def __init__(self, fs, queue_size):
        SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()

    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.input_connections = [f.registerCallback(self.add, q) for (f, q) in zip(fs, self.queues)]

    def add(self, msg, my_queue):
        self.lock.acquire()
        my_queue[msg.header.stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # common is the set of timestamps that occur in all queues
        common = reduce(set.intersection, [set(q) for q in self.queues])
        for t in sorted(common):
            # msgs is list of msgs (one from each queue) with stamp t
            msgs = [q[t] for q in self.queues]
            self.signalMessage(*msgs)
            for q in self.queues:
                del q[t]
        self.lock.release()
