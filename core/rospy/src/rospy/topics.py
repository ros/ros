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

## rospy implementation of topics.
#
# Publisher and Subscriber are the client-API facing instantiation of
# topics. Internally, _TopicImpl instances
# (_PublisherImpl/_SubscriberImpl) are used to manage actual
# transport connections.  The _TopicManager is responsible for
# tracking the system-wide state of publications and subscribtions as
# well as the _TopicImpl instances. More info is below.
# 
# _TopicManager
#
# The _TopicManager does the backend topic bookkeeping for the local
# node.  Use get_topic_manager() to access singleton. Actual topic
# implementations are done through the
# _TopicImpl/_PublisherImpl/_SubscriberImpl
# hierarchy. Client code generates instances of type
# Publisher/Subscriber, which enable to client to create
# multiple publishers/subscribers of that topic that get controlled
# access to the underlying share connections.
#
# Common parent classes for all rospy topics. The rospy topic autogenerators
# create classes that are children of these implementations.
# 
# TopicListener
# 
# Subscribe to new topics created by the local node. This is mainly
# a hook for creating registration calls to the master.

import struct, cStringIO, thread, threading, logging, time
from itertools import chain

from roslib.message import Message, SerializationError

from rospy.core import *
from rospy.exceptions import ROSSerializationException
from rospy.msg import serialize_message, AnyMsg
from rospy.registration import get_topic_manager, set_topic_manager, Registration, get_registration_listeners
from rospy.tcpros import get_tcpros_handler, DEFAULT_BUFF_SIZE
from rospy.transport import DeadTransport

logger = logging.getLogger('rospy.topics')

#######################################################################
# Base classes for all client-API instantiated pub/sub
#
# There are two trees: Topic and _TopicImpl. Topic is the client API
# for interfacing with topics, while _TopicImpl implements the
# underlying connection details. 

##Base class of client API topic references
class Topic(object):
    
    ## @param self
    ## @param name str: graph resource name of topic, e.g. 'laser'. 
    ## @param data_class Message: message class for serialization
    ## @param reg_type Registration.PUB or Registration.SUB
    ## @throws ROSException if parameters are invalid
    def __init__(self, name, data_class, reg_type):
        if not name:
            raise ROSException("topic parameter 'name' is not initialized")
        if data_class is None:
            raise ROSException("topic parameter 'data_class' is not initialized")
        if not type(data_class) == type:
            raise ROSException("data_class [%s] is not a class"%data_class) 
        if not issubclass(data_class, Message):
            raise ROSException("data_class [%s] is not a message data class"%data_class.__class__.__name__)
        
        self.name = resolve_name(name) #NOTE: remapping occurs here!
        self.data_class = data_class
        self.type = data_class._type
        self.md5sum = data_class._md5sum
        self.reg_type = reg_type
        self.impl = get_topic_manager().acquire_impl(reg_type, self.name, data_class)

    ## unpublish/unsubscribe from topic. Topic instance is no longer
    ## valid after this call.
    ## @param self
    def unregister(self):
        get_topic_manager().release_impl(self.reg_type, self.name)
        self.impl = self.name = self.type = self.md5sum = self.data_class = None
    
## Base class of internal topic implementations. Each topic has a
## singleton _TopicImpl implementation for managing the underlying
## connections.
class _TopicImpl(object):
    
    ## Base constructor
    ## @param self
    ## @param name str: graph resource name of topic, e.g. 'laser'. 
    ## @param data_class Message: message data class 
    def __init__(self, name, data_class):
        self.name = resolve_name(name) #NOTE: remapping occurs here!
        self.data_class = data_class
        self.type = data_class._type
        self.handler = None
        self.seq = 0
        # lock is used for to serialize call order that methods that
        # modify self.connections. Because add/removing connections is
        # a rare event, we go through the extra hassle of making a
        # copy of the connections/dead_connections/callbacks lists
        # when modifying, then setting the reference to the new copy.
        # With this pattern, other code can use these lists without
        # having to acquire c_lock
        self.c_lock = threading.RLock()
        self.connections = []
        self.closed = False
        # number of Topic instances using this
        self.ref_count = 0
        #STATS
        self.dead_connections = [] #for retaining stats on old conns

    ## close I/O
    ## @param self
    def close(self):
        self.closed = True
        try:
            self.c_lock.acquire()
            for c in self.connections:
                try:
                    c.close()
                except:
                    # seems more logger.error internal than external logerr
                    logger.error(traceback.format_exc())
            del self.connections[:]
        finally:
            self.c_lock.release()

    ## Query whether or not a connection with the associated \a
    ## endpoint has been added to this object.
    ## @param self
    ## @param endpoint_id str: endpoint ID associated with connection. 
    def has_connection(self, endpoint_id):
        # save reference to avoid lock
        conn = self.connections
        for c in conn:
            if c.endpoint_id == endpoint_id:
                return True
        return False

    ## Check to see if this topic is connected to other publishers/subscribers
    ## @param self
    def has_connections(self):
        if self.connections:
            return True
        return False

    ## Add a connection to this topic. 
    ## @param self
    ## @param c Transport: connection instance
    ## @return bool: True if connection was added
    def add_connection(self, c):
        try:
            # c_lock is to make add_connection thread-safe, but we
            # still make a copy of self.connections so that the rest of the
            # code can use self.connections in an unlocked manner
            self.c_lock.acquire()
            new_connections = self.connections[:]
            new_connections.append(c)
            self.connections = new_connections

            # connections make a callback when closed
            c.set_cleanup_callback(self.remove_connection)
            
            return True
        finally:
            self.c_lock.release()

    ## Remove connection from topic.
    ## @param self
    ## @param c Transport: connection instance to remove
    def remove_connection(self, c):
        try:
            # c_lock is to make remove_connection thread-safe, but we
            # still make a copy of self.connections so that the rest of the
            # code can use self.connections in an unlocked manner
            self.c_lock.acquire()
            new_connections = self.connections[:]
            new_dead_connections = self.dead_connections[:]                        
            if c in new_connections:
                new_connections.remove(c)
                new_dead_connections.append(DeadTransport(c))
            self.connections = new_connections
            self.dead_connections = new_dead_connections
        finally:
            self.c_lock.release()

    ## Get the stats for this topic
    ## @param self
    ## @return list: stats for topic in getBusInfo() format
    ## ((connection_id, destination_caller_id, direction, transport, topic_name, connected)*)
    def get_stats_info(self): # STATS
        # save referenceto avoid locking
        connections = self.connections
        dead_connections = self.dead_connections
        return [(c.id, c.endpoint_id, c.direction, c.transport_type, self.name, True) for c in connections] + \
               [(c.id, c.endpoint_id, c.direction, c.transport_type, self.name, False) for c in dead_connections]

    ## Get the stats for this topic (API stub)
    ## @param self
    def get_stats(self): # STATS
        raise Exception("subclasses must override")

## \ingroup clientapi Client API
## Class for registering as a subscriber to a specified topic, where
#  the messages are of a given type. Client code is not expected to
#  use this class directly. Instead, this is the base class of
#  auto-generated FooTypeSub classes from the message generator.
#  Clients can use these classes as a shortcut for having to come up
#  with the full type name correctly.
# 
#  Implementation note: Subscriber attaches to a _SubscriberImpl
#  singleton for that topic.  The underlying impl manages the
#  connections for that publication and enables thread-safe access
class Subscriber(Topic):

    ## Constructor. NOTE: for the \a queue_size and \a buff_size
    ## parameters, rospy does not attempt to do intelligent merging
    ## between multiple Subscriber instances for the same topic. As
    ## they share the same underlying transport, multiple Subscribers
    ## to the same topic can conflict with one another if they set
    ## these parameters differently.
    ##
    ## @param self
    ## @param name str: graph resource name of topic, e.g. 'laser'.
    ## @param data_class class: data type class to use for messages,
    ##   e.g. std_msgs.msg.String
    ## @param callback str: function to call ( fn(data)) when data is
    ##   received. If callback_args is set, the function must accept
    ##   the callback_args as a second argument, i.e. fn(data,
    ##   callback_args).  NOTE: Additional callbacks can be added using
    ##   add_callback().
    ## @param callback_args: additional opaque arguments to pass to the
    ##   callback. This is useful when you wish to reuse the same
    ##   callback for multiple subscriptions. NOTE: only one callback_args
    ##   can be set, regardless of the number of callbacks.
    ## @param queue_size int: maximum number of messages to receive at
    ##   a time. This will generally be 1 or None (infinite,
    ##   default). \a buff_size should be increased if this parameter
    ##   is set as incoming data still needs to sit in the incoming
    ##   buffer before being discarded.
    ## @param buff_size int: incoming message buffer size in bytes. If
    ##   \a queue_size is set, this should be set to a number greater
    ##   than the queue_size times the average message size.
    ## @throws ROSException if parameters are invalid
    def __init__(self, name, data_class, callback=None, callback_args=None,
                 queue_size=None, buff_size=DEFAULT_BUFF_SIZE):
        super(Subscriber, self).__init__(name, data_class, Registration.SUB)
        #add in args that factory cannot pass in
        self.impl.set_queue_size(queue_size)
        self.impl.set_buff_size(buff_size)        
        self.impl.set_callback_args(callback_args)
        self.impl.add_callback(callback)

    ## @param self
    ## @param cb fn: callback to register        
    def add_callback(self, cb):
        """Register a callback to be invoked whenever a new message is received"""
        self.impl.add_callback(cb)
        
## _Topic*Impl classes manage the underlying connections for a given topic. The
#  separation of the _Topic*Impl classes and the Topic* classes that a client
#  instantiates allows the client to generate multiple Topic* instances for a given
#  topic. It also hides the underlying API for managing the connections.
class _SubscriberImpl(_TopicImpl):

    ## @param self    
    ## @param name str: graph resource name of topic, e.g. 'laser'. 
    #  @param data_class Message: Message data class
    def __init__(self, name, data_class):
        super(_SubscriberImpl, self).__init__(name, data_class)
        self.callbacks = [] #client-methods to invoke on new messages. should only modify under lock
        self.cb_args    = None
        self.queue_size = None
        self.buff_size = DEFAULT_BUFF_SIZE
        
    ## Set the receive queue size. If more than \a queue_size messages are waiting to be deserialized,
    ## they are discarded.
    ## @param self
    ## @param queue_size int: incoming queue size. Must be positive integer or None.
    def set_queue_size(self, queue_size):
        if queue_size == -1:
            self.queue_size = None
        elif queue_size == 0:
            raise ROSException("queue size may not be set to zero")
        elif queue_size is not None and type(queue_size) != int:
            raise ROSException("queue size must be an integer")
        else:
            self.queue_size = queue_size

    ## Set the receive buffer size. The exact meaning of this is
    ## transport dependent.
    ## @param self
    ## @param buff_size int: receive buffer size
    def set_buff_size(self, buff_size):
        if type(buff_size) != int:
            raise ROSException("buffer size must be an integer")
        elif buff_size <= 0:
            raise ROSException("buffer size must be a positive integer")
        self.buff_size = buff_size
        
    ## Get the stats for this topic subscriber
    ## @param self
    ## @return list: stats for topic in getBusStats() publisher format (topicName, connStats),
    ##   where connStats = [connectionId, bytesReceived, numSent, dropEstimate, connected]*
    def get_stats(self): # STATS
        # save reference to avoid locking
        conn = self.connections
        dead_conn = self.dead_connections        
        #for now drop estimate is -1
        stats = (self.name, 
                 [(c.id, c.stat_bytes, c.stat_num_msg, -1, not c.done)
                  for c in chain(conn, dead_conn)] )
        return stats

    ## If callback args is set (i.e. not None), then it will be passed
    ## in as an opaque second argument to callbacks.  If it is not
    ## set, then the callback function will only receive the message
    ## data as a parameter.
    ## @param self
    ## @param cb_args arguments to pass to callback in addition to data parameter
    def set_callback_args(self, cb_args):
        self.cb_args = cb_args
        
    ## Register a callback to be invoked whenever a new message is received
    ## @param self
    ## @param cb callback function to invoke with message data
    ##   instance, i.e. fn(data). If callback args is set
    ##   (set_callback_args()), then it must accept two arguments,
    ##   i.e. fn(data, callback_args).
    def add_callback(self, cb):
        try:
            # we lock in order to serialize calls to add_callback, but
            # we copy self.callbacks so we can it
            self.c_lock.acquire()
            new_callbacks = self.callbacks[:]
            new_callbacks.append(cb)
            self.callbacks = new_callbacks
        finally:
            self.c_lock.release()
        
    ## Called by underlying connection transport for each new message received
    ## @param self
    ## @param msgs [Message]: message data
    def receive_callback(self, msgs):
        # save reference to avoid lock
        callbacks = self.callbacks
        for msg in msgs:
            for cb in callbacks:
                try:
                    if self.cb_args is not None:
                        cb(msg, self.cb_args)
                    else:
                        cb(msg)
                except Exception, e:
                    if not is_shutdown():
                        logerr("bad callback: %s\n%s"%(cb, traceback.format_exc()))
                    else:
                        logger.warn("during shutdown, bad callback: %s\n%s"%(cb, traceback.format_exc()))                        

## \ingroup clientapi
class SubscribeListener(object):

    ## listener callback when a peer has subscribed from a topic
    ## @param self
    ## @param topic_name str: topic name. NOTE: topic name will be resolved/remapped
    ## @param topic_publish fn(data): method to publish message data to all subscribers
    ## @param peer_publish fn(data): method to publish message data to
    ##   new subscriber.  NOTE: behavior for the latter is
    ##   transport-dependent as some transports may be broadcast only.
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        pass

    ## listener callback when a peer has unsubscribed from a topic
    ## @param self
    ## @param topic_name str: topic name. NOTE: topic name will be resolved/remapped
    ## @param num_peers int: number of remaining peers subscribed to topic
    def peer_unsubscribe(self, topic_name, num_peers):
        pass


## \ingroup clientapi Client API
## Class for registering as a publisher of a specified topic, where
#  the messages are of a given type. Client code is not expected to
#  use this class directly. Instead, this is the base class of
#  auto-generated FooTypePub classes from the message generator.
#  Clients can use these classes as a shortcut for having to come up
#  with the full type name correctly.
#              
#  Implementation note: Publisher attaches to a
#  _PublisherImpl singleton for that topic.  The underlying impl
#  manages the connections for that publication and enables
#  thread-safe access
class Publisher(Topic):

    ## Constructor
    ## @param self
    ## @param name str: resource name of topic, e.g. 'laser'. 
    ## @param data_class message obj: message class for serialization
    ## @param subscriber_listener SubscriberListener: listener for
    ##   subscription events. May be None.
    ## @param tcp_nodelay bool: If True, sets TCP_NODELAY on
    ##   publisher's socket (disables Nagle algorithm). This results
    ##   in lower latency publishing at the cost of efficiency.
    ## @throws ROSException if parameters are invalid     
    def __init__(self, name, data_class, subscriber_listener=None, tcp_nodelay=False):
        super(Publisher, self).__init__(name, data_class, Registration.PUB)
        if subscriber_listener:
            self.impl.add_subscriber_listener(subscriber_listener)
        if tcp_nodelay:
            get_tcpros_handler().set_tcp_nodelay(name, tcp_nodelay)

    ## Publish message data object to this topic. Publish can either be called with the
    ## message instance to publish or with the constructor args for a new Message instance, i.e.
    ## \verbatim
    ##   pub.publish(message_instance)
    ##   pub.publish(message_field_1, message_field_2...)            
    ##   pub.publish(message_field_1='foo', message_field_2='bar')
    ## \endverbatim
    ## @param self
    ## @param args : Message instance, Message arguments, or no args if keyword arguments are used
    ## @param kwds : Message keyword arguments. If \a kwds are used, \a args must be unset
    ## @throws ROSException If rospy node has not been initialized
    ## @throws ROSSerializationException If unable to serialize
    ## message. This is usually a type error with one of the fields.
    def publish(self, *args, **kwds):
        if not is_initialized():
            raise ROSException("ROS node has not been initialized yet. Please call init_node() first")
        if args and kwds:
            raise TypeError("publish() can be called with arguments or keywords, but not both.")
        elif kwds:
            data = self.data_class(**kwds)
        else:
            if len(args) == 1 and (
                isinstance(args[0], self.data_class) or
                isinstance(args[0], AnyMsg)):
                data = args[0]
            else:
                data = self.data_class(*args)
        try:
            try:
                self.impl.acquire()
                self.impl.publish(data)
            except SerializationError, e:
                raise ROSSerializationException(str(e))
        finally:
            self.impl.release()            

## _Topic*Impl classes manage the underlying connections for a given topic. The
## separation of the _Topic*Impl classes and the Topic* classes that a client
## instantiates allows the client to generate multiple Topic* instances for a given
## topic. It also hides the underlying API for managing the connections.
class _PublisherImpl(_TopicImpl):
    
    ## @param self
    ## @param name str: name of topic, e.g. 'laser'. 
    #  @param data_class Message: Message data class    
    def __init__(self, name, data_class):
        super(_PublisherImpl, self).__init__(name, data_class)
        self.buff = cStringIO.StringIO()
        self.publock = threading.Lock() #for acquire()/release
        self.subscriber_listeners = []
        #STATS
        self.message_data_sent = 0

    ## Get the stats for this topic publisher
    ## @param self
    ## @return list: stats for topic in getBusStats() publisher format [topicName, messageDataBytes, connStats],
    ##   where connStats = [id, bytes, numMessages, connected]*
    def get_stats(self): # STATS
        # save reference to avoid lock
        conn = self.connections
        dead_conn = self.dead_connections        
        return (self.name, self.message_data_sent,
                [(c.id, c.stat_bytes, c.stat_num_msg, not c.done) for c in chain(conn, dead_conn)] )

    ## Add a SubscriberListener for subscribe events.
    ## @param self
    ## @param l SubscriberListener: listener instance
    def add_subscriber_listener(self, l):
        self.subscriber_listeners.append(l)
        
    ## lock for thread-safe publishing to this transport
    ## @param self
    def acquire(self):
        self.publock.acquire()
        
    ## lock for thread-safe publishing to this transport
    ## @param self
    def release(self):
        self.publock.release()
        
    ## Add a connection to this topic. This must be a PubTransport.
    ## @param self
    ## @param c Transport: connection instance
    ## @return bool: True if connection was added
    def add_connection(self, c):
        super(_PublisherImpl, self).add_connection(c)
        def publish_single(data):
            self.publish(data, c)
        for l in self.subscriber_listeners:
            l.peer_subscribe(self.name, self.publish, publish_single)
        return True
            
    ## Remove existing connection from this topic.
    ## @param self
    #  @param c Transport: connection instance to remove
    ## Remove existing connection from this topic.
    #  @param c Transport: connection instance to remove
    def remove_connection(self, c):
        super(_PublisherImpl, self).remove_connection(c)
        num = len(self.connections)                
        for l in self.subscriber_listeners:
            l.peer_unsubscribe(self.name, num)
            
    ## Publish the data to the topic. If the topic has no subscribers,
    ## the method will return without any affect. Access to publish()
    ## should be locked using acquire() and release() in order to
    ## ensure proper message publish ordering.
    ## @param self
    ## @param data Message: message data instance to publish
    ## @param connection_override Transport: publish to this connection instead of all
    ## @return bool: True if the data was published, False otherwise.
    ## @throws roslib.message.SerializationError if Message instance is unable to serialize itself
    def publish(self, data, connection_override=None):
        "Publish data to the topic, should be called under acquire() lock"
        if not self.has_connections():
            #publish() falls through
            return False

        if connection_override is None:
            #copy connections so we can iterate safely
            conns = self.connections
        else:
            conns = [connection_override]

        # serialize the message
        b = self.buff
        self.seq += 1 #count messages published to the topic
        serialize_message(b, self.seq, data)

        # send the buffer to all connections
        err_con = []
        data = b.getvalue()
        for c in conns:
            try:
                c.write_data(data)
            except Exception, e:
                logerr("publisher connection to [%s] terminated, see errorlog for details:\n%s"%(c.endpoint_id, traceback.format_exc()))
                err_con.append(c)

        # reset the buffer and update stats
        self.message_data_sent += b.tell() #STATS
        b.seek(0)
        b.truncate(0)

        # remove any bad connections
        for c in err_con:
            try:
                # connection will callback into remove_connection when
                # we close it
                c.close()
            except:
                pass
        

#################################################################################
# TOPIC MANAGER/LISTENER

## Tracks Topic objects.
## See get_topic_manager() for singleton access
class _TopicManager(object):
    """Tracks Topic objects"""
    
    ## @param self
    def __init__(self):
        super(_TopicManager, self).__init__()
        self.pubs = {} #: { topic: _PublisherImpl }
        self.subs = {} #: { topic: _SubscriberImpl }
        self.topics = set() # [str] list of topic names
        self.lock = threading.Condition()
        logger.info("topicmanager initialized")

    ## get topic publisher and subscriber connection info for getBusInfo() api
    ## @param self
    ## @return list: [bus info stats]
    ##   See getBusInfo() API for more data structure details.
    def get_pub_sub_info(self):
        try:
            self.lock.acquire()
            info = []
            for s in chain(self.pubs.itervalues(), self.subs.itervalues()):
                info.extend(s.get_stats_info())
            return info
        finally:
            self.lock.release()
            
    ## get topic publisher and subscriber stats for getBusStats() api
    ## @param self
    ## @return list: [publisherStats, subscriberStats].
    ##   See getBusStats() API for more data structure details.
    def get_pub_sub_stats(self):
        try:
            self.lock.acquire()
            return [s.get_stats() for s in self.pubs.itervalues()],\
                   [s.get_stats() for s in self.subs.itervalues()]
        finally:
            self.lock.release()
            
    ## Remove all registered publication and subscriptions, closing them on removal
    ## @param self
    def remove_all(self):
        for t in chain(self.pubs.itervalues(), self.subs.itervalues()):
            t.close()
        self.pubs.clear()
        self.subs.clear()        
        
    ## Add _TopicImpl instance to map
    ## @param self
    ## @param ps _TopicImpl: a pub/sub impl instance
    ## @param map dict: topic->_TopicImpl map to record instance in
    ## @param reg_type Registration.PUB or Registration.SUB
    def _add(self, ps, map, reg_type):
        topic = ps.name
        logger.debug("tm._add: %s, %s, %s", topic, ps.type, reg_type)
        try:
            self.lock.acquire()
            map[topic] = ps
            self.topics.add(topic)
            
            # NOTE: this call can take a lengthy amount of time (at
            # least until its reimplemented to use queues)
            get_registration_listeners().notify_added(topic, ps.type, reg_type)
        finally:
            self.lock.release()

    ## recalculate self.topics. expensive
    ## @param self
    def _recalculate_topics(self):
        self.topics = set([x.name for x in self.pubs.itervalues()] +
                          [x.name for x in self.subs.itervalues()])
    
    ## Remove _TopicImpl instance from map
    ## @param self
    ## @param ps _TopicImpl: a pub/sub impl instance
    ## @param map dict: topic->_TopicImpl map to remove instance in
    ## @param reg_type Registration.PUB or Registration.SUB
    def _remove(self, ps, map, reg_type):
        topic = ps.name
        logger.debug("tm._remove: %s, %s, %s", topic, ps.type, reg_type)
        try:
            self.lock.acquire()
            del map[topic]
            self. _recalculate_topics()
            
            # NOTE: this call can take a lengthy amount of time (at
            # least until its reimplemented to use queues)
            get_registration_listeners().notify_removed(topic, ps.type, reg_type)
        finally:
            self.lock.release()

    ## Acquire a TopicImpl for the specified topic (create one if it
    ## doesn't exist).  Every Topic instance has a TopicImpl that
    ## actually controls the topic resources so that multiple Topic
    ## instances use the same underlying connections. 'Acquiring' a
    ## topic implementation marks that another Topic instance is
    ## using the TopicImpl.
    ## @param self
    ## @param topic str: Topic name
    ## @param reg_type Registration.PUB or Registration.SUB
    ## @param data_class Class: message class for topic
    def acquire_impl(self, reg_type, topic, data_class):
        if reg_type == Registration.PUB:
            map = self.pubs
            impl_class = _PublisherImpl
        elif reg_type == Registration.SUB:
            map = self.subs
            impl_class = _SubscriberImpl
        else:
            raise Exception("invalid reg_type: %s"%s)
        try:
            self.lock.acquire()
            impl = map.get(topic, None)            
            if not impl:
                impl = impl_class(topic, data_class)
                self._add(impl, map, reg_type)
            impl.ref_count += 1
            return impl
        finally:
            self.lock.release()

    ## Release a TopicImpl for the specified topic.  Every Topic
    ## instance has a TopicImpl that actually controls the topic
    ## resources so that multiple Topic instances use the same
    ## underlying connections. 'Acquiring' a topic implementation
    ## marks that another Topic instance is using the TopicImpl.
    ## @param self
    ## @param topic str: topic name
    ## @param reg_type Registration.PUB or Registration.SUB
    def release_impl(self, reg_type, topic):
        if reg_type == Registration.PUB:
            map = self.pubs
        else:
            map = self.subs
        try:
            self.lock.acquire()
            impl = map.get(topic, None)
            assert impl is not None, "cannot release topic impl as impl does not exist"
            impl.ref_count -= 1
            assert impl.ref_count >= 0, "topic impl's reference count has gone below zero"
            if impl.ref_count == 0:
                logger.debug("topic impl's ref count is zero, deleting topic %s...", topic)
                impl.close()
                self._remove(impl, map, reg_type)
                logger.debug("... done deletig topic %s", topic)                
        finally:
            self.lock.release()

    ## @param self
    ## @param topic str: topic name
    ## @return [_PublisherImpls]: list of _PublisherImpls
    def get_publisher_impl(self, topic):
        return self.pubs.get(topic, None)
    ## @param self
    ## @param topic str: topic name
    ## @return _SubscriberImpl: subscriber for the specified topic. Note,
    #  this is different from getPublishers(), which returns a list
    #  of publishers.
    def get_subscriber_impl(self, topic):
        return self.subs.get(topic, None)
    ## @param self
    ## @param topic str: topic name
    ## @return bool: True if manager has subscription for specified \a topic
    def has_subscription(self, topic):
        return topic in self.subs
    ## @param self
    ## @param topic str: topic name
    ## @return bool: True if manager has publication for specified \a topic
    def has_publication(self, topic):
        return topic in self.pubs

    ## @param self
    ## @return [str]: list of topic names this node subscribes to/publishes
    def get_topics(self):
        return self.topics
    
    def _get_list(self, map):
        return [[k, v.type] for k, v in map.iteritems()]
    ## @param self
    ## @return [[str,str],]: list of topics subscribed to by this node, [ [topic1, topicType1]...[topicN, topicTypeN]]
    def get_subscriptions(self):
        return self._get_list(self.subs)
    ## @param self
    ## @return [[str,str],]: list of topics published by this node, [ [topic1, topicType1]...[topicN, topicTypeN]]
    def get_publications(self):
        return self._get_list(self.pubs)

# #519 backwards compatibility
TopicPub = Publisher
TopicSub = Subscriber

set_topic_manager(_TopicManager())

