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
# $Author$

"""
Python utility for iterating over messages in a ROS .bag file.

See http://ros.org/wiki/ROS/LogFormat
"""

#Removing compression until we work out how to play nicely with index: JML
#import bz2
#import gzip
import optparse
import os
import struct
import sys
import time
from cStringIO import StringIO

import roslib.genpy
import roslib.gentools
import roslib.msgs
import roslib.message
import roslib.rostime
import rospy

class ROSRecordException(Exception):
  """
  Base exception type for rosrecord-related errors.
  """
  pass

HEADER_V1_1 = "#ROSRECORD V1.1"
HEADER_V1_2 = "#ROSRECORD V1.2"

g_message_defs = {}   # message definitions are stored by md5sum, so can be shared across bag files  

class BagReader(object):
  """
  Python bag file reader
  """
  def __init__(self, f):
    """
    Open a bag file for reading
    
    @param f: either a path to bag file to open for reading, or a stream to read from 
    @type  f: str or file
    @raise ROSRecordException: if version not supported
    """
    self._open(f)

  def _open(self, f):
    if isinstance(f, file):
      self.file = f
    else:
      self.filename = f

#Removing compression until we work out how to play nicely with index: JML
#      ext = os.path.splitext(self.filename)[1]
#      if ext == '.gz':
#        self.file = gzip.open(self.filename)
#      elif ext == '.bz2':
#        self.file = bz2.BZ2File(self.filename)
#      else:
      self.file = open(self.filename, 'r')

    version_readers = {
      HEADER_V1_1 : self._next_msg_v1_1,
      HEADER_V1_2 : self._next_msg_v1_2
    }

    try:
      self.version = self.file.readline().rstrip()
      if not self.version in version_readers:
        raise ROSRecordException('rosrecord.py only supports %s. File version is %s' % (','.join(version_readers.keys()), self.version))
    except:
      self.file.close()
      raise
    self.next_msg = version_readers[self.version]

    self.first_record_pos = self.file.tell()

    self.header    = None   # dict of the raw file header record header
    self.index_pos = None   # position in the bag file of the first index data record
    self.index     = None   # topic indexes
    self.datatypes = None   # datatype of topics

  def close(self):
    """
    Close the bag file
    """
    if self.file is not None:
      if not self.file.closed:
        self.file.close()
      self.file = None

  def read_index(self):
    """
    Read the index from the file (if it exists)

    @return dict(topic, [(stamp, pos)...]): bag file index
    """
    try:
      try:
        # The file header record is the first in the file (if it exists)
        self.file.seek(self.first_record_pos)
        self.next_msg(True)

        if self.index_pos is not None:
          # Seek to the first index data record
          self.file.seek(self.index_pos)
          
          # Read to the next non-index data record
          self.next_msg(True)
      except KeyboardInterrupt:
        pass # break iterator
    except:
      self.file.close()

    return self.index
  
  def raw_messages(self):
    """
    Generates raw messages

    @return (pos, topic, msg, t)...: tuple of position, topic, message data and time stamp
    """
    while True:
      try:
        pos = self.file.tell()
        topic, msg, t = self.next_msg(True)        
        if msg is None or rospy.is_shutdown():
          break
        yield pos, topic, msg, t
    
      except rosrecord.ROSRecordException, e:
        rospy.logerr('ROSRecordException: couldn\'t read msg at pos %d - %s' % (pos, e))
        break
      except IOError:
        rospy.logerr('IOError: couldn\'t read msg at pos %d' % pos)
        break
      except KeyError:
        rospy.logerr('KeyError: couldn\'t read msg at pos %d' % pos)
        break

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
        self.file.seek(pos)

        _, raw_msg, t = self.next_msg(True) 
        if raw_msg is not None:
          (datatype, message_data, md5, bag_pos, pytype) = raw_msg

          msg = pytype()
          msg.deserialize(message_data)

          if index:
            index.datatype_defs_read.add(datatype)

          yield (datatype, msg, t)

    except rosrecord.ROSRecordException, e:
      rospy.logerr('ROSRecordException: couldn\'t read %d - %s' % (pos, e)) 
    except IOError:
      rospy.logerr('IOError: couldn\'t read %d' % pos)
    except KeyError:
      rospy.logerr('KeyError: couldn\'t read %d' % pos)

  def read_datatype_defs(self, index):
    index.datatype_defs_read = set()

    try:
      for datatype, pos in index._data.datatype_first_pos.items():
        self.file.seek(pos)

        _, raw_msg, _ = self.next_msg(True)
        if raw_msg is not None:
          (datatype, message_data, md5, bag_pos, pytype) = raw_msg

          msg = pytype()
          msg.deserialize(message_data)

          index.datatype_defs_read.add(datatype)

    except rosrecord.ROSRecordException, e:
      rospy.logerr('ROSRecordException: couldn\'t read %d - %s' % (pos, e)) 
    except IOError:
      rospy.logerr('IOError: couldn\'t read %d' % pos)
    except KeyError:
      rospy.logerr('KeyError: couldn\'t read %d' % pos)

  def logplayer(self, raw=False, seek=None):
    if self.file is None:
      return
    
    if seek is not None:
      self.file.seek(seek)
    else:
      self.file.seek(self.first_record_pos)
      
    try:
      try:
        while True:
          topic, msg, t = self.next_msg(raw)
          if msg is None or rospy.is_shutdown():
            break
          yield topic, msg, t
      except KeyboardInterrupt:
        pass # break iterator
    finally:
      self.file.close()

  def _next_msg_v1_1(self, raw=False):
    """
    Get the next message. also calls any registered handlers
    @param raw: see return
    @type  raw: bool
  
    @return: (topic, data, time). If not raw, data will be the
    deserialized Message. If raw, data will be a sequence
    (datatype,data, md5sum, bag_position).
    
    More elements may be added to this
    sequence in the future. Message next message in bag for which there
    is a matching handler
    @rtype: (str, data, rospy.Time)
    """  
    f = self.file
    
    bag_pos = f.tell()
    # read topic/md5/type string headers
    topic = f.readline().rstrip()
    if not topic:
      return None, None, None
    md5sum = f.readline().rstrip()
    datatype = f.readline().rstrip()
    # migration for rostools->roslib rename
    if datatype in ['rostools/Header', 'rostools/Log', 'rostools/Time']:
      datatype = datatype.replace('rostools', 'roslib')
  
    # read time stamp
    data = f.read(12)
    if len(data) != 12:
      print >> sys.stderr, "WARNING: bag file appears to be corrupt (1)"
      return None, None, None
    (time_sec, time_nsec, length) = struct.unpack("<LLL", data)

    # read msg
    data = f.read(length)
    if len(data) != length:
      print >> sys.stderr, "WARNING: bag file appears to be corrupt (2)"
      return None, None, None
  
    try:
      pytype = g_message_defs[md5sum]
    except KeyError:
      try:
        pytype = roslib.message.get_message_class(datatype)
      except Exception:
        pytype = None
  
      if pytype is None:
        raise ROSRecordException("Cannot deserialize messages of type [%s]: cannot locate message class"%datatype)
      else:
        if pytype._md5sum != md5sum:
          (package, type) = datatype.split('/')
          if (roslib.gentools.compute_md5_v1(roslib.gentools.get_file_dependencies(roslib.msgs.msg_file(package,type))) == md5sum):
            print "In V1.1 Logfile, found old md5sum for type [%s].  Allowing implicit migration to new md5sum."%datatype
          else:
            raise ROSRecordException("Cannot deserialize messages of type [%s]: md5sum is outdated in V1.1 bagfile"%datatype)
        g_message_defs[md5sum] = pytype
  
    if raw:
      return topic, (datatype, data, pytype._md5sum, bag_pos, pytype), rospy.Time(time_sec, time_nsec)
    else:
      msg = pytype()
      msg.deserialize(data)
      return topic, msg, rospy.Time(time_sec, time_nsec)
  
  def _next_msg_v1_2(self, raw=False):
    """
    Get the next message. also calls any registered handlers
    @param raw: See return
    @type  raw: bool
    @return: (topic, data, time). If not raw, data will be the
    deserialized Message. If raw, data will be a sequence (datatype,
    data, md5sum, bag_position). More elements may be added to this
    sequence in the future. Message next message in bag for which there
    is a matching handler
    @rtype: (str, data, rospy.Time)
    """
    f = self.file
    
    def read_sized(f):
      data = f.read(4)
      if len(data) != 4:
        return None
      (size,) = struct.unpack("<L", data)
      r = f.read(size)
      if len(r) != size:
        return None
      return r
  
    bag_pos = f.tell()

    # Read record header
    record_header = read_sized(f)
    if record_header is None:
      return None, None, None
    
    # Read record data
    record_data = read_sized(f)
    if record_data is None:
      print >> sys.stderr, "WARNING: bag file appears to be corrupt (5)"
      return None, None, None
  
    # Parse header into a dict header
    hdr = {}
    while record_header != "":
      if len(record_header) < 4:
        print >> sys.stderr, "WARNING: bag file appears to be corrupt (2)"
        return None, None, None
      (size,) = struct.unpack("<L", record_header[:4])
      record_header = record_header[4:]
  
      if len(record_header) < size:
        print >> sys.stderr, "WARNING: bag file appears to be corrupt (3)"
        return None, None, None
      (name, sep, value) = record_header[:size].partition('=')
      if sep == "":
        print >> sys.stderr, "WARNING: bag file appears to be corrupt (4)"
        return None, None, None
      hdr[name] = value
      record_header = record_header[size:]
    
    try:
      op = hdr['op']
    except KeyError:
      cur = f.tell()
      f.seek(0, os.SEEK_END)
      end = f.tell()
      f.seek(cur)
  
      print >> sys.stderr, "WARNING: Found incomplete message header. %d bytes left."%(end - cur,)
      return None, None, None

    if op == chr(1):
      # opcode 0x01: Message definition
      required = set(['topic', 'md5', 'type', 'def'])
      assert required.issubset(set(hdr.keys()))
      topic    = hdr['topic']
      md5sum   = hdr['md5']
      datatype = hdr['type']
      msg_def  = hdr['def']
       
      try:
        g_message_defs[md5sum] = roslib.genpy.generate_dynamic(datatype, msg_def)[datatype]
      except roslib.genpy.MsgGenerationException, e:
        raise ROSRecordException(str(e))
  
      if g_message_defs[md5sum]._md5sum != md5sum:
        print "In V1.2 logfile, md5sum for type [%s] does not match definition.  Updating to new md5sum."%datatype
        
      return self._next_msg_v1_2(raw)
  
    elif op == chr(2):
      # opcode 0x02: Message data
      required = set(['topic', 'md5', 'type', 'sec', 'nsec'])
      assert required.issubset(set(hdr.keys()))
      topic        = hdr['topic']
      md5sum       = hdr['md5']
      datatype     = hdr['type']
      (time_sec,)  = struct.unpack("<L", hdr['sec'])
      (time_nsec,) = struct.unpack("<L", hdr['nsec'])

      try:
        pytype = g_message_defs[md5sum]
      except KeyError:
        raise ROSRecordException("Cannot deserialize messages of type [%s].  Message was not preceeded in bagfile by definition"%datatype)
    
      if raw:
        msg = (datatype, record_data, pytype._md5sum, bag_pos, pytype)
        return topic, msg, rospy.Time(time_sec, time_nsec)
      else:
        msg = pytype()
        msg.deserialize(record_data)
        
      return topic, msg, rospy.Time(time_sec, time_nsec)
      
    elif op == chr(3):
      # opcode 0x03: File header
      self.header = hdr
      
      if 'index_pos' in hdr:
          (index_pos,) = struct.unpack('<Q', hdr['index_pos'])
          self.index     = {}
          self.index_pos = index_pos
          self.datatypes = {}
      
      return self._next_msg_v1_2(raw)
  
    elif op == chr(4):
      # opcode 0x04: Index data
      required = set(['topic', 'ver', 'count'])
      assert required.issubset(set(hdr.keys()))
      topic    = hdr['topic']
      datatype = hdr['type']
      (ver,)   = struct.unpack("<L", hdr['ver'])
      (count,) = struct.unpack("<L", hdr['count'])

      if self.index is not None:
        if ver == 0:
          topic_index = []
        
          data_offset = 0
          for i in range(count):
            (time_sec,)  = struct.unpack('<L', record_data[data_offset:data_offset + 4]); data_offset += 4
            (time_nsec,) = struct.unpack('<L', record_data[data_offset:data_offset + 4]); data_offset += 4
            (offset,)    = struct.unpack('<Q', record_data[data_offset:data_offset + 8]); data_offset += 8
            
            stamp = roslib.rostime.Time(time_sec, time_nsec)
            
            topic_index.append((stamp, offset))
        
          self.index[topic] = topic_index
          self.datatypes[topic] = datatype

      return self._next_msg_v1_2(raw)

def open_log_file(f):
  """
  @return: file stream, ready to call next_record on
  @rtype: stream
  @raise ROSRecordException: if file format is unrecognized    
  """
  r = BagReader(f)

  return r.file, r.version

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

class Rebagger_v1_1(object):
  """
  Utility class for writing Message instances to a ROS .bag file
  """
  def __init__(self, filename):
    """
    Creates a new rebagger instance. This opens the bag file for writing.
    
    @param filename: name of bag file to write to
    @type  filename: str
    """
    self.buff = StringIO()
    self.f = open(filename, 'w')
    self.f.write(HEADER_V1_1+'\n')
    
  def add(self, topic, msg, t=None, raw=False):
    """
    Add a message to the bag
    @param msg: Message: message to add to bag
    @type  msg: L{roslib.message.Message}
    @param raw: if True, msg is in "raw" format
    @type  raw: bool
    @param t: ROS time of message publication
    @type  t: roslib.rostime.Time (rospy.Time)
    """
    # note: timestamp does not respect sim time as Rebagger is not required to be running in a rospy node
    if raw:
      f = self.f
      msg_type = msg[0]
      serialized_bytes = msg[1]
      md5sum = msg[2]
      f.write(topic+'\n'+md5sum+'\n'+msg_type+'\n')
      if not t:
        t = roslib.rostime.Time.from_sec(time.time())
      f.write(struct.pack("<LLL", t.secs, t.nsecs, len(serialized_bytes)))
      f.write(serialized_bytes)
    else:
      f = self.f
      buff = self.buff
      md5sum = msg.__class__._md5sum
      msg_type = msg.__class__._type
      f.write(topic+'\n'+md5sum+'\n'+msg_type+'\n')    
      msg.serialize(buff)
      if not t:
        t = roslib.rostime.Time.from_sec(time.time())
      f.write(struct.pack("<LLL", t.secs, t.nsecs, buff.tell()))
      f.write(buff.getvalue())

      buff.seek(0)
      buff.truncate(0)
    
  def close(self):
    if self.f:
      self.f.close()
    self.buff = None

class Rebagger(object):
  """
  Utility class for writing Message instances to a ROS .bag file
  """
  def __init__(self, filename):
    self.buff = StringIO()
    self.f = open(filename, 'w')
    self.f.write(HEADER_V1_2+'\n')

    self.defined = set()

  def add_hdr_data(self, hdr, data):
    s = ""
    for k,v in hdr.items():
      s += struct.pack("<L", len(k) + 1 + len(v))
      s += k
      s += '='
      s += v
    self.f.write(struct.pack("<L", len(s)))
    self.f.write(s)
    self.f.write(struct.pack("<L", len(data)))
    self.f.write(data)
    
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
    if raw:
      f = self.f
      msg_type = msg[0]
      serialized_bytes = msg[1]

      if (len(msg) == 5):
        md5sum = msg[4]._md5sum
      else:
        md5sum = msg[2]

      if not msg_type in self.defined:
        if (len(msg) == 5):
          pytype = msg[4]
        else:
          try:
            pytype = roslib.message.get_message_class(msg_type)
          except Exception:
            pytype = None
          if pytype is None:
            raise ROSRecordException("cannot locate message class and no message class provided for [%s]"%msg_type)

        if (pytype._md5sum != md5sum):
          print >> sys.stderr, "WARNING: md5sum of loaded type [%s] does not match that specified in Rebagger.add"%msg_type
          #raise ROSRecordException("md5sum of loaded type does not match that of data being recorded")

        self.defined.add(msg_type)
        self.add_hdr_data({ 'op' : chr(1), 'topic' : topic, 'md5' : md5sum, 'type' : msg_type, 'def' : pytype._full_text }, '')

      # note: timestamp does not respect sim time as Rebagger is not required to be running in a rospy node
      if not t:
        t = roslib.rostime.Time.from_sec(time.time())
      self.add_hdr_data({ 'op' : chr(2),
                          'topic' : topic,
                          'md5' : md5sum,
                          'type' : msg_type,
                          'sec' : struct.pack("<L", t.secs),
                          'nsec' : struct.pack("<L", t.nsecs)}, serialized_bytes)
    else:
      f = self.f
      buff = self.buff
      md5sum = msg.__class__._md5sum
      msg_type = msg.__class__._type

      if not msg_type in self.defined:
        self.defined.add(msg_type)
        self.add_hdr_data({ 'op' : chr(1), 'topic' : topic, 'md5' : md5sum, 'type' : msg_type, 'def' : msg._full_text }, '')
      msg.serialize(buff)
      # note: timestamp does not respect sim time as Rebagger is not required to be running in a rospy node
      if not t:
        t = roslib.rostime.Time.from_sec(time.time())
      self.add_hdr_data({ 'op' : chr(2),
                          'topic' : topic,
                          'md5' : md5sum,
                          'type' : msg_type,
                          'sec' : struct.pack("<L", t.secs),
                          'nsec' : struct.pack("<L", t.nsecs)}, buff.getvalue())
      buff.seek(0)
      buff.truncate(0)
    
  def close(self):
    if self.f:
      self.f.close()
    self.buff = None

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
  if verbose_pattern:
    rebag = Rebagger(outbag)
    for topic, msg, t in logplayer(inbag, raw=raw):
      if filter_fn(topic, msg, t):
        print "MATCH", verbose_pattern(topic, msg, t)
        rebag.add(topic, msg, t, raw=raw)          
        if rospy.is_shutdown():
          break
      else:
        print "NO MATCH", verbose_pattern(topic, msg, t)          
  else: #streamlined
    rebag = Rebagger(outbag)
    for topic, msg, t in logplayer(inbag, raw=raw):
      if filter_fn(topic, msg, t):
        rebag.add(topic, msg, t, raw=raw)
        if rospy.is_shutdown():
          break
    
def _rebag_main():
  """
  main routine for rosrebag command-line tool
  """
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
  inbag = args[0]
  outbag = args[1]
  expr = args[2]
  if options.verbose_pattern:
    verbose_pattern = expr_eval(options.verbose_pattern)
  else:
    verbose_pattern = None    
  if not os.path.isfile(inbag):
    print >> sys.stderr, "cannot locate input bag file [%s]"%inbag
    sys.exit(1)
  rebag(inbag, outbag, expr_eval(expr), verbose_pattern=verbose_pattern)
