# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

import roslib; roslib.load_manifest('rosbag')

import roslib.genpy
import rospy

import bz2
import cStringIO
import os
import re
import struct

class ROSBagException(Exception):
    """
    Base class for exceptions in rosbag
    """
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg

class ROSBagFormatException(ROSBagException):
    """
    Exceptions for errors relating to the bag file format
    """
    def __init__(self, msg):
        ROSBagException.__init__(self, msg)

class TopicInfo:
    def __init__(self, topic, datatype, md5sum, msg_def):
        self.topic    = topic
        self.datatype = datatype
        self.md5sum   = md5sum
        self.msg_def  = msg_def

    def __str__(self):
        return '%s: %s [%s]' % (self.topic, self.datatype, self.md5sum)

class ChunkInfo:
    def __init__(self, pos, start_time, end_time):
        self.pos        = pos
        self.start_time = start_time
        self.end_time   = end_time
        
        self.topic_counts = {}
        
    def __str__(self):
        s  = 'chunk_pos:  %d\n' % self.pos
        s += 'start_time: %s\n' % str(self.start_time)
        s += 'end_time:   %s\n' % str(self.end_time)
        s += 'topics:     %d\n' % len(self.topic_counts)
        max_topic_len = max([len(t) for t in self.topic_counts])
        s += '\n'.join(['  - %-*s %d' % (max_topic_len, topic, count) for topic, count in self.topic_counts.items()])
        return s

class ChunkHeader:
    COMPRESSION_NONE = 'none'
    COMPRESSION_BZ2  = 'bz2'
    COMPRESSION_ZLIB = 'zlib'
    
    def __init__(self, compression, compressed_size, uncompressed_size):
        self.compression       = compression
        self.compressed_size   = compressed_size
        self.uncompressed_size = uncompressed_size

    def __str__(self):
        s  = 'compression:  %s\n' % self.compression
        s += 'size:         %d\n' % self.compressed_size
        s += 'uncompressed: %d (%.2f%%)' % (self.uncompressed_size, 100 * (float(self.compressed_size) / self.uncompressed_size))
        return s

class IndexEntry:
    def __init__(self, time, chunk_pos, offset):
        self.time      = time
        self.chunk_pos = chunk_pos
        self.offset    = offset

class Bag(object):
    def __init__(self):
        self.file     = None
        self.filename = None
        
        self.topic_count = 0
        self.chunk_count = 0
        
        self.topic_infos   = {}
        self.chunk_infos   = []
        self.topic_indexes = {}
        
        self.serializer = None 

    def open(self, f, mode):
        """
        Opens the bag file
        @param f: either a filename or a file object
        @type  f: str or file
        """
        if isinstance(f, file):
            self.file     = f
            self.filename = None
        elif isinstance(f, str):
            rospy.loginfo('Opening %s' % f)
            self.file     = open(f, 'r')
            self.filename = f
        else:
            raise ROSBagException('open must be passed a file or str')

        # Read the version line
        try:
            self.version = self._read_version()
            rospy.loginfo('Version: %d' % self.version)
        except:
            self.file.close()
            raise
        
        if self.version == 103:
            self.serializer = _BagSerializer103(self)
        else:
            raise ROSBagFormatException('unknown bag version %d' % self.version)
        
        self.serializer.start_reading()

    def close(self):
        pass
    
    def getMessages(self):
        for topic, entries in self.topic_indexes.items():
            for entry in entries:
                self.serializer.read_message_data_record(topic, entry.chunk_pos, entry.offset)

        return []

    ### Record I/O
    
    def _read_version(self):
        version_line = self.file.readline().rstrip()
        
        matches = re.match("#ROS(.*) V(\d).(\d)", version_line)
        if matches is None or len(matches.groups()) != 3:
            raise ROSBagException('rosbag does not support %s' % version_line)
        
        version_type, major_version_str, minor_version_str = matches.groups()

        version = int(major_version_str) * 100 + int(minor_version_str)
        
        return version

    ### Low-level file I/O
    
    def _tell(self):
        return self.file.tell()

    def _seek(self, offset, whence=os.SEEK_SET):
        self.file.seek(offset, whence)

class _BagSerializer(object):
    def __init__(self, bag):
        self.bag = bag

def _read_uint8 (f): return _unpack_uint8 (f.read(1))
def _read_uint32(f): return _unpack_uint32(f.read(4))
def _read_uint64(f): return _unpack_uint64(f.read(8))
def _read_time  (f): return _unpack_time  (f.read(8))

def _read_str_field   (header, field): return _read_field(header, field, lambda v: v)
def _read_uint8_field (header, field): return _read_field(header, field, _unpack_uint8)
def _read_uint32_field(header, field): return _read_field(header, field, _unpack_uint32)
def _read_uint64_field(header, field): return _read_field(header, field, _unpack_uint64)
def _read_time_field  (header, field): return _read_field(header, field, _unpack_time)

def _unpack_uint8(v):  return struct.unpack('<B', v)[0]
def _unpack_uint32(v): return struct.unpack('<L', v)[0]
def _unpack_uint64(v): return struct.unpack('<Q', v)[0]
def _unpack_time(v):   return rospy.Time(*struct.unpack('<LL', v))

def _read(f, size):
    data = f.read(size)
    if len(data) != size:
        raise ROSBagException('Expecting %d bytes, read %d' % (size, len(data)))   
    return data

def _read_sized(f):
    size = _read_uint32(f)
    return _read(f, size)

def _read_field(header, field, unpack_fn):
    if field not in header:
        raise ROSBagFormatException('Expected "%s" field in record' % field)
    
    try:
        value = unpack_fn(header[field])
    except Exception, ex:
        raise ROSBagFormatException('Error reading field "%s": %s' % (field, str(ex)))
    
    return value

def _read_record_header(f):
    bag_pos = f.tell()

    # Read record header
    try:
        record_header = _read_sized(f)
    except ROSBagException, ex:
        raise ROSBagFormatException('Error reading record header: %s' % str(ex))

    # Parse header into a dict
    header_dict = {}
    while record_header != '':
        # Read size
        if len(record_header) < 4:
            raise ROSBagFormatException('Error reading record header field')           
        (size,) = struct.unpack("<L", record_header[:4])
        record_header = record_header[4:]

        # Read bytes
        if len(record_header) < size:
            raise ROSBagFormatException('Error reading record header field')
        (name, sep, value) = record_header[:size].partition('=')
        if sep == '':
            raise ROSBagFormatException('Error reading record header field')

        header_dict[name] = value
        
        record_header = record_header[size:]

    return header_dict

def _read_record_data(f):
    try:
        record_data = _read_sized(f)
    except ROSBagException, ex:
        raise ROSBagFormatException('Error reading record data: %s' % str(ex))

    return record_data

def _assert_op(header, expected_op):
    if expected_op != _read_uint8_field(header, 'op'):
        raise ROSBagFormatException('Expected op code: %d, got %d' % (expected_op, op))

class _BagSerializer103(_BagSerializer):
    OP_MSG_DEF     = 0x01
    OP_MSG_DATA    = 0x02
    OP_FILE_HEADER = 0x03
    OP_INDEX_DATA  = 0x04
    OP_CHUNK       = 0x05
    OP_CHUNK_INFO  = 0x06

    def __init__(self, bag):
        _BagSerializer.__init__(self, bag)
        
        self.index_data_pos  = 0
        self.curr_chunk_info = None
        self.message_types   = {}
        
        self.decompressed_chunk_pos = None
        self.decompressed_chunk     = None
    
    def start_reading(self):
        self.read_file_header_record()

        print 'index_data_pos: ' + str(self.index_data_pos)
        print 'chunk_count:    ' + str(self.chunk_count)
        print 'topic_count:    ' + str(self.topic_count)

        # Seek to the end of the chunks
        self.bag._seek(self.index_data_pos)

        # Read the message definition records (one for each topic)
        for i in range(self.topic_count):
            topic_info = self.read_message_definition_record()

            self.bag.topic_infos[topic_info.topic] = topic_info
            
            print topic_info

        # Read the chunk info records
        self.chunk_infos = []
        for i in range(self.chunk_count):
            self.read_chunk_info_record()
    
        # Read the topic indexes
        for chunk_info in self.chunk_infos:
            self.curr_chunk_info = chunk_info
            
            self.bag._seek(chunk_info.pos)
    
            # Skip over the chunk data
            chunk_header = self.read_chunk_header()
            self.bag._seek(chunk_header.compressed_size, os.SEEK_CUR)
    
            # Read the topic index records after the chunk
            for i in range(len(chunk_info.topic_counts)):
                (topic, index) = self.read_topic_index_record()
                
                self.bag.topic_indexes[topic] = index

    def read_file_header_record(self):
        header = _read_record_header(self.bag.file)
        
        _assert_op(header, self.OP_FILE_HEADER)

        self.index_data_pos = _read_uint64_field(header, 'index_pos')
        self.chunk_count    = _read_uint32_field(header, 'chunk_count')
        self.topic_count    = _read_uint32_field(header, 'topic_count')

        _read_record_data(self.bag.file)

    def read_message_definition_record(self):
        header = _read_record_header(self.bag.file)

        _assert_op(header, self.OP_MSG_DEF)

        topic    = _read_str_field(header, 'topic')
        datatype = _read_str_field(header, 'type')
        md5sum   = _read_str_field(header, 'md5')
        msg_def  = _read_str_field(header, 'def')

        _read_record_data(self.bag.file)

        return TopicInfo(topic, datatype, md5sum, msg_def)
    
    def get_message_type(self, topic_info):
        datatype, msg_def = topic_info.datatype, topic_info.msg_def
        
        message_type = self.message_types.get(datatype)
        if message_type is None:
            try:
                message_type = roslib.genpy.generate_dynamic(datatype, msg_def)[datatype]
            except roslib.genpy.MsgGenerationException, ex:
                raise ROSBagException('Error generating datatype %s: %s' % (datatype, str(ex)))

            self.message_types[datatype] = message_type

        return message_type

    def read_chunk_info_record(self):
        header = _read_record_header(self.bag.file)

        _assert_op(header, self.OP_CHUNK_INFO)

        chunk_info_version = _read_uint32_field(header, 'ver')
        
        if chunk_info_version == 1:
            chunk_pos   = _read_uint64_field(header, 'chunk_pos')
            start_time  = _read_time_field  (header, 'start_time')
            end_time    = _read_time_field  (header, 'end_time')
            topic_count = _read_uint32_field(header, 'count') 
    
            chunk_info = ChunkInfo(chunk_pos, start_time, end_time)
    
            _read_uint32(self.bag.file) # skip the record data size

            for i in range(topic_count):
                topic_name  = _read_sized(self.bag.file)
                topic_count = _read_uint32(self.bag.file)
    
                chunk_info.topic_counts[topic_name] = topic_count
                
                self.chunk_infos.append(chunk_info)
        else:
            raise ROSBagFormatException('Unknown chunk info record version: %d' % chunk_info_version)

    def read_chunk_header(self):
        header = _read_record_header(self.bag.file)
        
        _assert_op(header, self.OP_CHUNK)

        compression       = _read_str_field   (header, 'compression')
        uncompressed_size = _read_uint32_field(header, 'size')

        compressed_size = _read_uint32(self.bag.file)  # read the record data size
        
        return ChunkHeader(compression, compressed_size, uncompressed_size)

    def read_topic_index_record(self):
        header = _read_record_header(self.bag.file)

        _assert_op(header, self.OP_INDEX_DATA)
        
        index_version = _read_uint32_field(header, 'ver')
        topic         = _read_str_field   (header, 'topic')
        count         = _read_uint32_field(header, 'count')
    
        _read_uint32(self.bag.file) # skip the record data size

        if index_version == 0:
            return (topic, self.read_topic_index_data_version_0(count, topic))
        elif index_version == 1:
            return (topic, self.read_topic_index_data_version_1(count, topic))
        else:
            raise ROSBagFormatException('unknown topic index data version: %d' % index_version)

    def read_topic_index_data_version_0(self, count, topic):
        pass
    
    def read_topic_index_data_version_1(self, count, topic):
        topic_index = []
        
        for i in range(count):
            time   = _read_time(self.bag.file)
            offset = _read_uint32(self.bag.file)
            
            topic_index.append(IndexEntry(time, self.curr_chunk_info.pos, offset))
            
        return topic_index

    def read_message_data_record(self, topic, chunk_pos, offset):
        # Decompress chunk if needed
        if self.decompressed_chunk_pos != chunk_pos:
            # Seek to the start of the chunk
            self.bag._seek(chunk_pos)
    
            # Read the chunk header
            chunk_header = self.read_chunk_header()
    
            # Read and decompress the chunk
            if chunk_header.compression != ChunkHeader.COMPRESSION_NONE:
                compressed_chunk = _read(self.bag.file, chunk_header.compressed_size)
                
                self.decompressed_chunk     = bz2.decompress(compressed_chunk)
                self.decompressed_chunk_pos = chunk_pos

        if self.decompressed_chunk_pos == chunk_pos:
            f = cStringIO.StringIO(self.decompressed_chunk)
        else:
            f = self.bag.file

        f.seek(offset, os.SEEK_CUR)

        while True:
            header = _read_record_header(f)
            op = _read_uint8_field(header, 'op')
            if op != self.OP_MSG_DEF:
                break

            _read_record_data(f)

        if op != self.OP_MSG_DATA:
            raise ROSBagFormatException('Expecting OP_MSG_DATA, got %d' % op)

        msg_topic = _read_str_field(header, 'topic')
        if topic != msg_topic:
            raise ROSBagFormatException('Expecting topic "%s", got "%s"' % (topic, msg_topic))

        record_data = _read_record_data(f)

        topic_info = self.bag.topic_infos[topic]

        try:
            msg_type = self.get_message_type(topic_info)
        except KeyError:
            raise ROSBagException('Cannot deserialize messages of type [%s].  Message was not preceeded in bagfile by definition' % topic_info.datatype)
        
        msg = msg_type()
        msg.deserialize(record_data)
        
        print msg
        
        return msg
