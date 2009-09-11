#!/usr/bin/env python
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

import roslib; roslib.load_manifest('rostopic')

NAME='rostopic'

import os
import sys
import socket
import threading
import time

import roslib.names
import roslib.scriptutil
import rospy

from optparse import OptionParser

## send ANSI clear sequence before printing each message
_echo_clear = False
## don't print string fields in message
_echo_nostr = False
## don't print array fields in message
_echo_noarr = False

class RosTopicException(Exception): pass

def succeed(args):
    code, msg, val = args
    if code != 1:
        raise RosTopicException("remote call failed: %s"%msg)
    return val

class RosTopicHz(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.msg_count = 0
        self.msg_t0 = -1.0

    ## ros sub callback
    def callback_hz(self, data):
        try:
            curr_rostime = rospy.get_rostime()
            
            self.lock.acquire()

            # time reset
            if curr_rostime.is_zero():
                if self.msg_count > 0:
                    print "time has reset, resetting counters"
                    self.msg_t0 = -1.0
                    self.msg_count = 0
                return
            
            curr = curr_rostime.to_seconds()
            if self.msg_t0 < 0.0 or self.msg_t0 > curr:
                self.msg_t0 = curr
                self.msg_count = 1
            else:
                self.msg_count += 1
        finally:
            self.lock.release()

    ## print the average publishing rate to screen
    ## @param self
    def print_hz(self):
        if self.msg_count < 2:
            return
        try:
            self.lock.acquire()
            rate = (self.msg_count - 1) / (rospy.get_time() - self.msg_t0)
        finally:
            self.lock.release()
        print "average rate: %shz"%rate
    
## periodically print the publishing rate of a topic to console until
## shutdown
def rostopic_hz(topic):
    _, real_topic, _ = get_topic_type(topic) #pause hz until topic is published
    if rospy.is_shutdown():
        return
    rospy.init_node(NAME, anonymous=True)
    rt = RosTopicHz()
    # we use a large buffer size as we don't know what sort of messages we're dealing with.
    # may parameterize this in the future
    sub = rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback_hz)
    print "subscribed to [%s]"%real_topic
    while not rospy.is_shutdown():
        time.sleep(1.0)
        rt.print_hz()
    
## generates a function that returns the relevant field (aka 'subtopic') of a Message object
## @param pattern str: subtopic, e.g. /x. Must have a leading '/' if specified.
## @return fn(Message) -> value
def msgevalgen(pattern):
    if not pattern or pattern == '/':
        return None
    def msgeval(msg):
        # I will probably replace this with some less beautiful but more efficient
        return eval('msg'+'.'.join(pattern.split('/')))
    return msgeval
    
## subroutine for getting the topic type
## @return str, str, fn: topic type, real topic name and fn to evaluate the message instance
## if the \a topic points to a field within a topic, e.g. /rosout/msg
def _get_topic_type(topic):
    val = succeed(roslib.scriptutil.get_master().getPublishedTopics('/', '/'))
    matches = [(t, t_type) for t, t_type in val if t == topic or topic.startswith(t+'/')]
    if matches:
        #TODO logic for multiple matches if we are prefix matching
        t, t_type = matches[0]
        if t_type == roslib.names.ANYTYPE:
            return None, None, None
        if t_type == topic:
            return t_type, None
        return t_type, t, msgevalgen(topic[len(t):])
    else:
        return None, None, None
    
## get the topic type
## @raise RosTopicException if master cannot be contacted
## @return str, str, fn: topic type, real topic name and fn to evaluate the message instance
## if the \a topic points to a field within a topic, e.g. /rosout/msg
def get_topic_type(topic):
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    else:
        print >> sys.stderr, "WARNING: topic [%s] does not appear to be published yet"%topic
        while not rospy.is_shutdown():
            topic_type, real_topic, msg_eval = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, msg_eval
            else:
                time.sleep(0.1)

## get the topic message class
## @raise RosTopicException if topic type cannot be determined
## @return Message, str, str: message class for topic, real topic
## name, and function for evaluating message objects into the subtopic
## (or None)
def get_topic_class(topic):
    topic_type, real_topic, msg_eval = get_topic_type(topic)
    return roslib.scriptutil.get_message_class(topic_type), real_topic, msg_eval

from itertools import izip

## print fields used by _str_plot
## @param plot bool: if True, print in plotting-friendly format. non-scalar values are excluded
def _str_plot_fields(val, f):
    # CSV
    if type(val) in [int, float] or \
           isinstance(val, rospy.Time) or isinstance(val, rospy.Duration):
        return f
    elif isinstance(val, rospy.Message):
        sub = [s for s in [_str_plot_fields(getattr(val, a), f+"."+a) for a in val.__slots__] if s]
        if sub:
            return ','.join([s for s in sub])
    elif not _echo_nostr and type(val) == str:
        return f
    elif not _echo_noarr and type(val) in [list, tuple]:
        if len(val) == 0:
            return None
        val0 = val[0]
        # no arrays of arrays
        if type(val0) in [int, float] or \
               isinstance(val0, rospy.Time) or isinstance(val0, rospy.Duration):
            return ','.join(["%s%s"%(f,x) for x in xrange(0,len(val))])
        elif not _echo_nostr and type(val0) == str:
            return ','.join(["%s%s"%(f,x) for x in xrange(0,len(val))])
        elif isinstance(val0, rospy.Message):
            labels = ["%s%s"%(f,x) for x in xrange(0,len(val))]
            sub = [s for s in [_str_plot_fields(v, sf) for v,sf in izip(val, labels)] if s]
            if sub:
                return ','.join([s for s in sub])
    return None

_first = True

## convert value to matlab/octave-friendly CSV string representation.
## Reads the state of the echo_nostrs and echo_noarr global vars to
## determine which fields are printed.
## @param plot bool: if True, print in plotting-friendly format. non-scalar values are excluded
def _str_plot(val):
    global _first
    if _first:
        # this breaks our abstraction a bit but is cleaner than the correct way
        print "%"+_str_plot_fields(val, 'm')
        _first = False

    # CSV
    if type(val) in [int, float] or \
           isinstance(val, rospy.Time) or isinstance(val, rospy.Duration):
        return str(val)
    elif isinstance(val, rospy.Message):
        sub = [s for s in [_str_plot(getattr(val, a)) for a in val.__slots__] if s]
        if sub:
            return ','.join([s for s in sub])
    elif not _echo_nostr and type(val) == str:
        return val
    elif not _echo_noarr and type(val) in [list, tuple]:
        if len(val) == 0:
            return None
        val0 = val[0]
        # no arrays of arrays
        if type(val0) in [int, float] or \
               isinstance(val0, rospy.Time) or isinstance(val0, rospy.Duration):
            return ','.join([str(v) for v in val])
        elif not _echo_nostr and type(val0) == str:
            return ','.join([v for v in val])            
        elif isinstance(val0, rospy.Message):
            sub = [s for s in [_str_plot(v) for v in val] if s]
            if sub:
                return ','.join([s for s in sub])
    return None
        
def callback_echo_gen(msg_eval, plot=False, filter_fn=None):
    #TODO: _echo_clear: need to be fancier and instead of clearing pad the next output
    sep = '---\n'
    if plot:
        str_fn = _str_plot
        sep = ''
    else:
        str_fn = roslib.message.strify_message
        if _echo_clear:
            sep = '\033[2J\033[;H'
    if msg_eval:
        def callback_echo(data):
            if filter_fn is not None and not filter_fn(data):
                return
            try:
                print sep+str_fn(msg_eval(data))
            except IOError:
                rospy.signal_shutdown('IOError')
    else:
        def callback_echo(data):
            if filter_fn is not None and not filter_fn(data):
                return
            try:
                print sep+str_fn(data)
            except IOError:
                rospy.signal_shutdown('IOError')
    return callback_echo

def rostopic_type(topic):
    print >> sys.stdout, get_topic_type(topic)[0]

## @param topic str: topic name
## @param plot bool: if True, echo in plotting-friendly format
## @param bag_file str: name of bag file to echo messages from or None
## @param filter_fn fn(topic, msg): function that evaluates to True if message is to be echo'd
def rostopic_echo_bag(topic, plot, bag_file, filter_fn=None):
    import rosrecord
    if not os.path.exists(bag_file):
        raise RosTopicException("bag file [%s] does not exist"%bag_file)
    first = True
    for t, msg, _ in rosrecord.logplayer(bag_file):
        if t[0] != '/':
            t = roslib.scriptutil.script_resolve_name('rostopic', t)
        if filter_fn is not None:
            if not filter_fn(msg):
                continue
        if t == topic:
            if plot:
                print _str_plot(msg)
            else:
                print str(msg)
        elif topic.startswith(t+'/'):
            if plot:
                print _str_plot(msgevalgen(topic[len(t):])(msg))
            else:
                print str(msgevalgen(topic[len(t):])(msg))
        if rospy.is_shutdown():
            break
    
## @param topic str: topic name
## @param plot bool: if True, echo in plotting-friendly format
## @param bag_file str: name of bag file to echo messages from or None
## @param filter_fn fn(topic, msg): function that evaluates to True if message is to be echo'd
def rostopic_echo(topic, plot, bag_file=None, filter_fn=None):
    if bag_file:
        rostopic_echo_bag(topic, plot, bag_file, filter_fn=filter_fn)
    else:
        rospy.init_node(NAME, anonymous=True)
        msg_class, real_topic, msg_eval = get_topic_class(topic)
        if not plot:
            print "topic type is [%s]"%msg_class._type
        sub = rospy.Subscriber(real_topic, msg_class,
                               callback_echo_gen(msg_eval, plot=plot, filter_fn=filter_fn))
        rospy.spin()

_caller_apis = {}
def get_api(master, caller_id):
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api:
        code, msg, caller_api = master.lookupNode('/rostopic', caller_id)
        if code != 1:
            caller_api = 'unknown address'%caller_id
        else:
            _caller_apis[caller_id] = caller_api
    return caller_api

def rostopic_list_bag(bag_file, topic=None):
    import rosrecord
    if not os.path.exists(bag_file):
        raise RosTopicException("bag file [%s] does not exist"%bag_file)
    if topic:
        count = 0
        earliest = None
        latest = None
        for top, msg, t in rosrecord.logplayer(bag_file, raw=True):
            if top == topic:
                count += 1
                if earliest == None:
                    earliest = t
                latest = t
        import time
        earliest, latest = [time.strftime("%d %b %Y %H:%M:%S", time.localtime(t.to_time())) for t in (earliest, latest)]
        print "%s message(s) from %s to %s"%(count, earliest, latest)
    else:
        topics = set()
        for top, msg, _ in rosrecord.logplayer(bag_file, raw=True):
            if top not in topics:
                print top
                topics.add(top)
            if rospy.is_shutdown():
                break

def rostopic_list(topic):
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'
    
    master = roslib.scriptutil.get_master()
    state = succeed(master.getSystemState('/rostopic'))

    pubs, subs, _ = state
    publists = [publist for t, publist in pubs if t == topic]
    sublists = [sublist for t, sublist in subs if t == topic]

    pub_topics = succeed(roslib.scriptutil.get_master().getPublishedTopics('/rostopic', '/'))    
    if topic:
        #print '-'*80
        print "\nType: %s\n"%topic_type(topic, pub_topics)

        import itertools
        if publists:
            print "Publishers: "
            for p in itertools.chain(*publists):
                print " * %s (%s)"%(p, get_api(master, p))
        else:
            print "Publishers: None"
        print ''

        if sublists:
            print "Subscribers: "
            for p in itertools.chain(*sublists):
                print " * %s (%s)"%(p, get_api(master, p))
        else:
            print "Subscribers: None"
        print ''
                    
    else:
        #print '-'*80
        print "\nPublished topics:"
        topics = [t for t,_ in pubs]
        topics.sort()
        print '\n'.join([" * %s [%s]"%(t, topic_type(t, pub_topics)) for t in topics])
        print ''
        print "Subscribed topics:"
        topics = [t for t,_ in subs]
        topics.sort()
        print '\n'.join([" * %s [%s]"%(t, topic_type(t, pub_topics)) for t in topics])

##########################################################################################
# COMMAND PROCESSING #####################################################################
    
def rostopic_cmd_echo():
    def expr_eval(expr):
        def eval_fn(m):
            return eval(expr)
        return eval_fn
    
    args = sys.argv[2:]
    parser = OptionParser(usage="usage: %prog echo [options] /topic", prog=NAME)
    parser.add_option("-b", "--bag",
                      dest="bag", default=None,
                      help="echo messages from .bag file", metavar="BAGFILE")
    parser.add_option("-p", 
                      dest="plot", default=False,
                      action="store_true",
                      help="echo in a plotting friendly format")
    parser.add_option("--filter", 
                      dest="filter_expr", default=None,
                      metavar="FILTER-EXPRESSION",
                      help="Python expression to filter messages that are printed. Expression can use Python builtins as well as m (the message) and topic (the topic name).")
    parser.add_option("--nostr", 
                      dest="nostr", default=False,
                      action="store_true",
                      help="exclude string fields")
    parser.add_option("--noarr",
                      dest="noarr", default=False,
                      action="store_true",
                      help="exclude arrays")
    parser.add_option("-c", "--clear",
                      dest="clear", default=False,
                      action="store_true",
                      help="clear screen before printing next message")

    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("topic must be specified")        
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    topic = roslib.scriptutil.script_resolve_name('rostopic', args[0])    
    if not options.plot:
        print "rostopic: topic is [%s]"%topic
        
    global _echo_clear, _echo_nostr, _echo_noarr
    _echo_clear = options.clear
    _echo_nostr = options.nostr
    _echo_noarr = options.noarr

    filter_fn = None
    if options.filter_expr:
        filter_fn = expr_eval(options.filter_expr)
    
    try:
        rostopic_echo(topic, options.plot, bag_file=options.bag, filter_fn=filter_fn)
    except socket.error:
        print >> sys.stderr, "Network communication failed. Most likely failed to communicate with master."
    
def _optparse_topic_only(cmd, argv=sys.argv):
    args = argv[2:]
    parser = OptionParser(usage="usage: %%prog %s /topic"%cmd, prog=NAME)
    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("topic must be specified")        
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    return roslib.scriptutil.script_resolve_name('rostopic', args[0])

def rostopic_cmd_type():
    rostopic_type(_optparse_topic_only('type'))
    
def rostopic_cmd_hz(argv=sys.argv):
    rostopic_hz(_optparse_topic_only('hz', argv=argv))

def rostopic_cmd_list():
    args = sys.argv[2:]
    parser = OptionParser(usage="usage: %prog list [/topic]", prog=NAME)
    parser.add_option("-b", "--bag",
                      dest="bag", default=None,
                      help="list topics in .bag file", metavar="BAGFILE")
    (options, args) = parser.parse_args(args)
    topic = None
    if len(args) == 1:
        topic = roslib.scriptutil.script_resolve_name('rostopic', args[0])
    elif len(args) > 1:
        parser.error("you may only specify one input topic")
    if options.bag:
        rostopic_list_bag(options.bag, topic)        
    else:
        rostopic_list(topic)
    
def fullusage():
    print """Commands:
\trostopic hz\tdisplay publishing rate of topic    
\trostopic echo\tprint messages to screen
\trostopic type\tprint topic type
\trostopic list\tprint information about active topics

Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
"""
    sys.exit(os.EX_USAGE)

def rostopicmain(argv=sys.argv):
    if len(argv) == 1:
        fullusage()
    try:
        command = argv[1]
        if command == 'echo':
            rostopic_cmd_echo()
        elif command == 'hz':
            rostopic_cmd_hz(argv)
        elif command == 'type':
            rostopic_cmd_type()
        elif command == 'list':
            rostopic_cmd_list()
        else:
            fullusage()
    except socket.error:
        print >> sys.stderr, "Network communication failed. Most likely failed to communicate with master."
    except RosTopicException, e:
        print >> sys.stderr, str(e)

if __name__ == '__main__':
    rostopicmain(argv=['rostopic', 'hz', 'cloud_pcd'])
