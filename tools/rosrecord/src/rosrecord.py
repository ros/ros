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

PKG = 'rosrecord'
import roslib; roslib.load_manifest(PKG)

import warnings
warnings.warn('rosrecord is deprecated; use the rosbag package (http://www.ros.org/wiki/rosbag) instead', category=DeprecationWarning)

import optparse
import os
import sys

import rospy
import rosbag
from bisect import bisect

class ROSRecordException(Exception):
    """
    Base exception type for rosrecord-related errors.
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

class BagReader(object):
    """
    Bag file reader
    """
    def __init__(self, f):
        """
        Open a bag file for reading
        
        @param f: either a path to bag file to open for reading, or a stream to read from 
        @type  f: str or file
        @raise ROSRecordException: if version not supported
        """
        try:
            self._bag = rosbag.Bag(f, 'r')
        except Exception, e:
            raise ROSRecordException(str(e))

    def close(self):
        """
        Close the bag file
        """
        try:
            if self._bag:
                self._bag.close()
        except Exception, e:
            raise ROSRecordException(str(e))

    @property
    def datatypes(self): return dict([(info.topic, info.datatype) for info in self._bag._connections.values()])
    
    @property
    def filename(self): return self._bag.filename

    def read_index(self):
        """
        Read the index from the file (if it exists)
    
        @return dict(topic, [(stamp, pos)...]): bag file index
        """
        try:
            index = {}
            
            connection_index = self._bag.get_index()
            for (connection_id, connection_topic, connection_datatype), entries in connection_index.items():
                if connection_topic not in index:
                    index[connection_topic] = []
                topic_index = index[connection_topic]

                for entry in entries:
                    topic_index.insert(bisect(topic_index, entry), entry)
            
            self.index = self._bag.get_index()

        except rosbag.ROSBagException:
            self.index = None

        return self.index

    def logplayer(self, raw=False, seek=None):
        if not self._bag:
            return

        if seek is not None:
            raise ROSRecordException('seek not supported')

        try:
            try:
                for topic, msg, t in self._bag.read_messages(raw=raw):
                    if msg is None or rospy.is_shutdown():
                        break
                    yield topic, msg, t
            except KeyboardInterrupt:
                pass # break iterator
        finally:
            self._bag.close()

    def raw_messages(self):
        """
        Generates raw messages
    
        @return (pos, topic, msg, t)...: tuple of position, topic, message data and time stamp
        """
        try:
            for topic, msg, t in self._bag.read_messages(raw=True):
                if msg is None or rospy.is_shutdown():
                    break
                (datatype, data, md5sum, position, pytype) = msg
                yield position, topic, msg, t
        
        except rosbag.ROSBagException, e:
            rospy.logerr('ROSBagException: couldn\'t read message - %s' % str(e))
        except IOError:
            rospy.logerr('IOError: couldn\'t read message')
        except KeyError:
            rospy.logerr('KeyError: couldn\'t read message')

    def load_message(self, pos, index=None):
        """
        Load message from the bag file at a given position
    
        @param pos: byte offset in file of start of message record
        @type  pos: int
        @return (datatype, msg, t): tuple of datatype, deserialized message, and timestamp   
        """
        msgs = list(self.load_messages([pos], index))
        if len(msgs) == 0:
            return None, None, None
        
        return msgs[0]
    
    ## Returns iterator of (datatype, message, timestamp)
    def load_messages(self, positions, index=None):
        try:
            for pos in positions:
                _, raw_msg, t = self._bag._read_message(pos, True)
                if raw_msg is not None:
                    (datatype, message_data, md5, bag_pos, pytype) = raw_msg
                    
                    msg = pytype()
                    msg.deserialize(message_data)

                    yield (datatype, msg, t)
        
        except rosbag.ROSBagException, e:
            rospy.logerr('ROSRecordException: couldn\'t read %s - %s' % (str(pos), str(e))) 
        except IOError:
            rospy.logerr('IOError: couldn\'t read %d' % pos)
        except KeyError:
            rospy.logerr('KeyError: couldn\'t read %d' % pos)

def logplayer(f, raw=False, seek=None):
    """
    Iterator for messages in a bag file. Return type of iterator depends on value of 'raw' parameter, which
    is provided for improved iteration performance.
    
    If raw is False:
      - topic: topic name (str)
      - msg: Message instance (Message)
      - t: time (rospy.Time)
    
     If raw is True:
      - topic: topic name (str)
      - msg: (datatype, data, md5sum, bag_position).
         - datatype: msg type name
         - data: serialized message bytes.
         - bag_position: offset into the file, which can be used for seeking.
         - md5sum: md5sum of msg type for identify msg version
         - NOTE: More elements may be added to this in the future, so this should
           be unpacked assuming non-fixed length.
      - t: time (rospy.Time)
     
    @param filename: name of file to playback from
    @type  filename: str
    @param raw: see description for raw behavior
    @type  raw: bool
    @param seek: initial position to seek to (default None)
    @type  seek: int
    @raise ROSRecordException: if file format is unrecognized
    @return: iterator for messages. See description for iterator behavior.
    @rtype: iterator
    """
    r = BagReader(f)
    
    return r.logplayer(raw, seek)

class Rebagger(object):
    """
    Utility class for writing Message instances to a ROS .bag file
    """
    def __init__(self, filename):
        self._bag = rosbag.Bag(filename, 'w')
    
    def add(self, topic, msg, t=None, raw=False):
        """
        Add a message to the bag
        @param msg: message to add to bag
        @type  msg: Message
        @param raw: if True, msg is in raw format (msg_type, serialized_bytes, md5sum, pytype)
        @param raw: bool
        @param t: ROS time of message publication
        @type  t: U{roslib.message.Time}
        """
        try:
            self._bag.write(topic, msg, t, raw)
        except:
            raise ROSRecordException('error adding message to bag')
    
    def close(self):
        try:
            if self._bag:
                self._bag.close()
        except:
            raise ROSRecordException('error closing Rebagger')

def rebag(inbag, outbag, filter_fn, verbose_pattern=None, raw=False):
    """
    Filter the contents of inbag to outbag using filter_fn
    @param inbag: filename of input bag file.
    @type  inbag: str
    @param outbag: filename of output bag file. Existing bag file will be overwritten
    @type  outbag: str
    @param filter_fn: Python function that returns True if msg is to be kept
    @type  filter_fn: fn(topic, msg, time)
    @param raw: if True, msg will be kept deserialized. This is useful
    if you are removing messages that no longer exist.
    @type  raw: bool
    @param verbose_pattern: Python function that returns string to
    print for verbose debugging
    @type  verbose_pattern: fn(topic, msg, time)
    """
    rebag = Rebagger(outbag)
    if verbose_pattern:
        for topic, msg, t in logplayer(inbag, raw=raw):
            if filter_fn(topic, msg, t):
                print "MATCH", verbose_pattern(topic, msg, t)
                rebag.add(topic, msg, t, raw=raw)          
                if rospy.is_shutdown():
                    break
            else:
                print "NO MATCH", verbose_pattern(topic, msg, t)          
    else: #streamlined
        for topic, msg, t in logplayer(inbag, raw=raw):
            if filter_fn(topic, msg, t):
                rebag.add(topic, msg, t, raw=raw)
                if rospy.is_shutdown():
                    break
    rebag.close()

def _rebag_main():
    """
    main routine for rosrebag command-line tool
    """

    rospy.logwarn("Rosrebag has been deprecated.  Please use 'rosbag filter' instead\n")

    ## filter function that uses command line expression to test topic/message/time
    def expr_eval(expr):
        def eval_fn(topic, m, t):
            return eval(expr)
        return eval_fn
    
    parser = optparse.OptionParser(usage="""Usage: %prog in.bag out.bag filter-expression

filter-expression can be any Python-legal expression.
The following variables are available:
 * topic: name of topic
 * m: message
 * t: time of message (t.secs, t.nsecs)
""", prog='rosrebag')
    parser.add_option('--print', dest="verbose_pattern", default=None,
                      metavar="PRINT-EXPRESSION", help="Python expression to print for verbose debugging. Uses same variables as filter-expression.")

    options, args = parser.parse_args()
    if len(args) == 0:
        parser.print_usage()
        sys.exit(0)
    elif len(args) != 3:
        parser.error("invalid arguments")

    inbag, outbag, expr = args
    if options.verbose_pattern:
        verbose_pattern = expr_eval(options.verbose_pattern)
    else:
        verbose_pattern = None    
    if not os.path.isfile(inbag):
        print >> sys.stderr, "cannot locate input bag file [%s]" % inbag
        sys.exit(1)
        
    rebag(inbag, outbag, expr_eval(expr), verbose_pattern=verbose_pattern)
