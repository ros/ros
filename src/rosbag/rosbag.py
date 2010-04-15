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
# Revision $Id: rosrecord.py 8058 2010-01-22 21:58:53Z tfield $
# $Author: tfield $

"""
Python utility for accessing ROS bag files.

See http://www.ros.org/wiki/Bags/Format
"""

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

class ROSBagException(Exception):
    """
    Base exception type for rosbag-related errors.
    """
    pass

g_message_defs = {}   # message definitions are stored by md5sum, so can be shared across bag files

class BagReader:
    def __init__(self, major_version, minor_version):
        self.major_version = major_version
        self.minor_version = minor_version

class BagReader101:
    def __init__(self):
        BagReader.__init__(1, 1)

    def next_message(self, raw=False):
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
                        print "In V1.1 Logfile, found old md5sum for type [%s].  Allowing implicit migration to new md5sum." % datatype
                    else:
                        raise ROSRecordException("Cannot deserialize messages of type [%s]: md5sum is outdated in V1.1 bagfile" % datatype)
                g_message_defs[md5sum] = pytype

        if raw:
            return topic, (datatype, data, pytype._md5sum, bag_pos, pytype), rospy.Time(time_sec, time_nsec)
        else:
            msg = pytype()
            msg.deserialize(data)
            return topic, msg, rospy.Time(time_sec, time_nsec)

class BagReader102:
    def __init__(self):
        BagReader.__init__(1, 1)

    def next_message(self, raw=False):
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
                print "In V1.2 logfile, md5sum for type [%s] does not match definition.  Updating to new md5sum." % datatype

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
                raise ROSRecordException("Cannot deserialize messages of type [%s].  Message was not preceeded in bagfile by definition" % datatype)

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

            return self.next_message(raw)

class BagReader103:
    def __init__(self):
        BagReader.__init__(1, 1)

    def next_message(self, raw=False):
        pass

class Bag(object):
    def __init__(self):
        self.file     = None
        self.filename = None

    def open(self, f):
        if isinstance(f, file):
            self.file = f
        else:
            self.filename = f
            self.file = open(self.filename, 'r')

        try:
            version_line = self.file.readline().rstrip()
            
            matches = re.match("#ROS(.*) v(\d).(\d)", version_line)
            if matches is not None and len(matches.groups()) == 3:
                version_type, major_version_str, minor_version_str = matches.groups()

                version = int(major_version_str) * 100 + int(minor_version_str)
                
                if   version == 101: reader = BagReader101()
                elif version == 102: reader = BagReader102()
                elif version == 103: reader = BagReader103()
            else:
                raise BagException('rosbag does not support %s' % version_line)
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
