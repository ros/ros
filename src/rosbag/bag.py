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

# @todo:
# - write/reader latching and caller-id values
# - recreate index for unindexed or broken bags

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)

import bz2
from cStringIO import StringIO
import heapq
import os
import re
import struct

import roslib.genpy
import rospy

class ROSBagException(Exception):
    """
    Base class for exceptions in rosbag.
    """
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg

class ROSBagFormatException(ROSBagException):
    """
    Exceptions for errors relating to the bag file format.
    """
    def __init__(self, msg):
        ROSBagException.__init__(self, msg)

class Compression:
    """
    Allowable compression types
    """
    NONE = 'none'
    BZ2  = 'bz2'
    ZLIB = 'zlib'

class Bag(object):
    """
    Bag objects serialize messages to and from disk using the bag format.
    """
    def __init__(self, filename, mode='r', compression=Compression.NONE, chunk_threshold=768 * 1024):
        """
        Open a bag file.  The mode can be 'r', 'w', or 'a' for reading (default),
        writing or appending.  The file will be created if it doesn't exist
        when opened for writing or appending; it will be truncated when opened
        for writing.  Simultaneous reading and writing is allowed when in writing
        or appending mode.        
        @param filename: filename of bag to open
        @type  filename: str
        @param mode: mode, either 'r', 'w', or 'a'
        @type  mode: str
        @param compression: compression mode, see Compression
        @type  compression: str
        @param chunk_threshold: size in bytes of chunks
        @type  chunk_threshold: int
        @raise ValueError: if any argument is invalid
        @raise ROSBagException: if error in opening file
        @raise ROSBagFormatException: if bag is 
        """
        if not filename:
            raise ValueError('filename is invalid')
        if mode not in ['r', 'w', 'a']:
            raise ValueError('mode is invalid')
        allowed_compressions = [Compression.NONE, Compression.BZ2, Compression.ZLIB]
        if compression not in allowed_compressions:
            raise ValueError('compression must be one of: %s' % ', '.join(allowed_compressions))
        if chunk_threshold < 0:
            raise ValueError('chunk_threshold must be greater than or equal to zero')

        self._filename        = None
        self._mode            = None                
        self._version         = None
        self._compression     = compression
        self._chunk_threshold = chunk_threshold

        self._file            = None
        self._reader          = None
        self._topic_infos     = {}    # topic -> TopicInfo
        self._file_header_pos = None

        self._index_data_pos  = 0     # (1.2+)
        self._topic_indexes   = {}    # topic -> IndexEntry[] (1.2+)

        self._topic_count     = 0     # (1.3)
        self._chunk_count     = 0     # (1.3)
        self._chunk_infos     = []    # ChunkInfo[] (1.3)
        self._chunk_headers   = {}    # chunk_pos -> ChunkHeader (1.3)
        
        self._buffer                   = StringIO()        

        self._chunk_open               = False
        self._curr_chunk_info          = None
        self._curr_chunk_data_pos      = None
        self._curr_chunk_topic_indexes = {}

        self._curr_compression = Compression.NONE
        self._output_file      = self._file
        
        self._open(filename, mode)
    
    @property
    def filename(self):
        """Get the filename."""
        return self._filename
    
    @property
    def version(self):
        """Get the version."""
        return self._version
    
    @property
    def mode(self):
        """Get the mode."""
        return self._mode

    @property
    def size(self):
        """Get the size in bytes."""
        pos = self._file.tell()
        self._file.seek(0, os.SEEK_END)
        size = self._file.tell()
        self._file.seek(pos)
        return size

    # compression
        
    def _get_compression(self):
        """Get the compression."""
        return self._compression
    
    def _set_compression(self, compression):
        allowed_compressions = [Compression.NONE, Compression.BZ2, Compression.ZLIB]
        if compression not in allowed_compressions:
            raise ValueError('compression must be one of: %s' % ', '.join(allowed_compressions))        
        
        self.flush()
        self._compression = compression
        
    compression = property(_get_compression, _set_compression)
    
    # chunk_threshold
    
    def _get_chunk_threshold(self):
        """Get the chunk threshold."""
        return self._chunk_threshold
    
    def _set_chunk_threshold(self, chunk_threshold):
        if chunk_threshold < 0:
            raise ValueError('chunk_threshold must be greater than or equal to zero')
        
        self.flush()
        self._chunk_threshold = chunk_threshold

    def write(self, topic, msg, t):
        """
        Write a message to the bag.
        @param topic: name of topic
        @type  topic: str
        @param msg: message to add to bag
        @type  msg: Message
        @param t: ROS time of message publication
        @type  t: U{roslib.rostime.Time}
        @raise ValueError: if arguments are invalid
        """
        if not topic:
            raise ValueError('topic is invalid')
        if not msg:
            raise ValueError('msg is invalid')
        
        # Seek to end (in case previous operation was a read)
        self._file.seek(0, os.SEEK_END)
        
        # Open a chunk, if needed
        if not self._chunk_open:
            self._start_writing_chunk(t)

        # Update the current chunk's topic index
        index_entry = _IndexEntry103(t, self._curr_chunk_info.pos, self._get_chunk_offset())
        self._curr_chunk_topic_indexes.setdefault(topic, []).append(index_entry)
        curr_topic_count = self._curr_chunk_info.topic_counts.setdefault(topic, 0)
        self._curr_chunk_info.topic_counts[topic] = curr_topic_count + 1

        # Write message definition record, if necessary
        if topic not in self._topic_infos:
            topic_info = _TopicInfo(topic, msg.__class__._type, msg.__class__._md5sum, msg._full_text)
            self._write_message_definition_record(topic_info)
            self._topic_infos[topic] = topic_info

        # Serialize the message to the buffer
        self._buffer.seek(0)
        self._buffer.truncate(0)
        msg.serialize(self._buffer)

        # Write message data record
        self._write_message_data_record(topic, t, self._buffer.getvalue())
        
        # Check if we want to stop this chunk
        chunk_size = self._get_chunk_offset()
        if chunk_size > self._chunk_threshold:
            self._stop_writing_chunk()

    def getMessages(self):
        """
        Return a list of all messages in the bag file.
        """
        self.flush()
        
        return self._reader.get_messages()
    
    def flush(self):
        """
        Write the open chunk to disk so subsequent reads will read all messages.
        """
        if self._chunk_open:
            self._stop_writing_chunk()

    def close(self):
        """
        Close the bag file.
        """
        if self._file:
            if self._mode in 'wa':
                self._stop_writing()
            
            self._file.close()

    def dump(self):
        """
        Print a summary of the contents of the bag.
        """
        print '---- %s [%s] ----' % (self._filename, self._mode)

        print 'TOPICS:'
        for topic_info in self._topic_infos:
            print '  * ' + topic_info
        print

        print 'CHUNK HEADERS:'
        for pos in sorted(self._chunk_headers):
            print '  %d: %s' % (pos, str(self._chunk_headers[pos]))
        print

        print 'CHUNKS:'
        for i, chunk_info in enumerate(self._chunk_infos):
            print '  %d: %s' % (i, str(chunk_info))
        print
            
        print 'INDEX:'
        for topic in sorted(self._topic_indexes):
            print '  * ' + topic
            for entry in self._topic_indexes[topic]:
                print '    - ' + str(entry)
                                
        print '----'

    ### Implementation ###

    def _open(self, filename, mode):
        if   mode == 'r': self._open_read(filename)
        elif mode == 'w': self._open_write(filename)
        elif mode == 'a': self._open_append(filename)
        else:
            raise ROSBagException('unknown mode: %s' % mode)

    def _open_read(self, filename):
        self._file     = open(filename, 'rb')
        self._filename = filename        
        self._mode     = 'r'

        try:
            self._version = self._read_version()
        except:
            self._version = None
            self._file.close()
            raise

        self._create_reader()
        self._reader.start_reading()

    def _open_write(self, filename):
        self._file     = open(filename, 'wb')
        self._filename = filename
        self._mode     = 'w'

        self._start_writing()

    def _open_append(self, filename):
        file_exists = True
        try:
            f = open(filename, 'r')
            f.close()
        except:
            file_exists = False
        if file_exists:
            self._file = open(filename, 'r+b')
        else:
            self._file = open(filename, 'w+b')
        
        self._filename = filename        
        self._mode     = 'a'
        
        try:
            self._version = self._read_version()
        except:
            self._version = None
            self._file.close()
            raise

        if self._version != 103:
            self._file.close()
            raise ROSBagException('bag version %d is unsupported for appending' % self._version)

        self._file_header_pos = self._file.tell()

        self._create_reader()
        self._reader.start_reading()

        # Truncate the file to chop off the index
        self._file.truncate(self._index_data_pos)
        self._reader.index_data_pos = 0
    
        # Rewrite the file header, clearing the index position (so we know if the index is invalid)
        self._file.seek(self._file_header_pos);
        self._write_file_header_record(0, 0, 0)

        # Seek to the end of the file
        self._file.seek(0, os.SEEK_END)
        
    def _create_reader(self):
        """
        @raise ROSBagException: if the bag version is unsupported
        """
        if self._version == 103:
            self._reader = _BagReader103(self)
        elif self._version == 102:
            # Get the op code of the first record.  If it's FILE_HEADER, then we have
            # an indexed 1.2 bag.
            first_record_pos = self._file.tell()
            header = _read_record_header(self._file)
            op = _read_uint8_field(header, 'op')
            self._file.seek(first_record_pos)

            if op == _OP_FILE_HEADER:
                self._reader = _BagReader102_Indexed(self)
            else:
                self._reader = _BagReader102_Unindexed(self)
        else:
            raise ROSBagException('unknown bag version %d' % self._version)

    def _read_version(self):
        """
        @raise ROSBagException: if the file is empty, or the version line can't be parsed
        """
        version_line = self._file.readline().rstrip()
        if len(version_line) == 0:
            raise ROSBagException('empty file')
        
        matches = re.match("#ROS(.*) V(\d).(\d)", version_line)
        if matches is None or len(matches.groups()) != 3:
            raise ROSBagException('rosbag does not support %s' % version_line)
        
        version_type, major_version_str, minor_version_str = matches.groups()

        version = int(major_version_str) * 100 + int(minor_version_str)
        
        return version

    def _start_writing(self):        
        self._file.write(_VERSION + '\n')
        self._file_header_pos = self._file.tell()
        self._write_file_header_record(0, 0, 0)

    def _stop_writing(self):
        # Write the open chunk (if any) to file
        self.flush()

        # Remember this location as the start of the index
        self._index_data_pos = self._file.tell()

        # Write topic infos
        for topic_info in self._topic_infos.values():
            self._write_message_definition_record(topic_info)

        # Write chunk infos
        for chunk_info in self._chunk_infos:
            self._write_chunk_info_record(chunk_info)

        # Re-write the file header
        self._file.seek(self._file_header_pos)
        self._write_file_header_record(self._index_data_pos, len(self._topic_infos), len(self._chunk_infos))

    # @todo: update with chunk logic
    def _write_raw(self, topic, raw_msg_data):
        """
        Add a message to the bag
        @param topic: name of topic
        @type  topic: str
        @param raw_msg_data: raw message data
        @type  raw_msg_data: tuple of (msg_type, serialized_bytes, md5sum, pytype)
        @param t: ROS time of message publication
        @type  t: U{roslib.message.Time}
        """
        msg_type, serialized_bytes, md5sum, pytype = raw_msg_data

        # Write message definition record, if necessary
        if topic not in self._topic_infos:
            if pytype is None:
                try:
                    pytype = roslib.message.get_message_class(msg_type)
                except Exception:
                    pytype = None
            if pytype is None:
                raise ROSBagException('cannot locate message class and no message class provided for [%s]' % msg_type)

            if pytype._md5sum != md5sum:
                print >> sys.stderr, 'WARNING: md5sum of loaded type [%s] does not match that specified' % msg_type
                #raise ROSRecordException('md5sum of loaded type does not match that of data being recorded')

            topic_info = _TopicInfo(topic, msg_type, md5sum, pytype._full_text)
            self._write_message_definition_record(topic_info)
            self._topic_infos[topic] = topic_info

        # Write message data record
        self._write_message_data_record(topic, t, serialized_bytes)

    def _start_writing_chunk(self, t):
        self._curr_chunk_info = _ChunkInfo(self._file.tell(), t, t)
        self._write_chunk_header(_ChunkHeader(self._compression, 0, 0))
        self._curr_chunk_data_pos = self._file.tell()
        self._set_compression_mode(self._compression)
        self._chunk_open = True
    
    def _get_chunk_offset(self):
        if self._compression == Compression.NONE:
            return self._file.tell() - self._curr_chunk_data_pos
        else:
            return self._output_file.compressed_bytes_in

    def _stop_writing_chunk(self):
        # Add this chunk to the index
        self._chunk_infos.append(self._curr_chunk_info)
        
        # Update the topic indexes with the current chunk index
        for topic, entries in self._curr_chunk_topic_indexes.items():
            self._topic_indexes.setdefault(topic, []).extend(entries)

        # Get the uncompressed and compressed sizes
        uncompressed_size = self._get_chunk_offset()
        self._set_compression_mode(Compression.NONE)
        compressed_size = self._file.tell() - self._curr_chunk_data_pos

        # Rewrite the chunk header with the size of the chunk (remembering current offset)
        end_of_chunk_pos = self._file.tell()
        self._file.seek(self._curr_chunk_info.pos)
        chunk_header = _ChunkHeader(self._compression, compressed_size, uncompressed_size, self._curr_chunk_data_pos)
        self._write_chunk_header(chunk_header)
        self._chunk_headers[self._curr_chunk_info.pos] = chunk_header

        # Write out the topic indexes and clear them
        self._file.seek(end_of_chunk_pos)
        for topic, entries in self._curr_chunk_topic_indexes.items():
            self._write_topic_index_record(topic, entries)
        self._curr_chunk_topic_indexes.clear()

        # Flag that we're starting a new chunk
        self._chunk_open = False

    def _set_compression_mode(self, compression):
        # Flush the compressor, if needed
        if self._curr_compression == Compression.BZ2:
            self._output_file.flush()
        
        # Create the compressor
        if compression == Compression.BZ2:
            self._output_file = _BZ2CompressorFileFacade(self._file)
        else:
            self._output_file = self._file

        self._curr_compression = compression

    def _write_file_header_record(self, index_pos, topic_count, chunk_count):
        header = {
            'op':          _pack_uint8(_OP_FILE_HEADER),
            'index_pos':   _pack_uint64(index_pos),
            'topic_count': _pack_uint32(topic_count),
            'chunk_count': _pack_uint32(chunk_count)
        }
        _write_record(self._file, header, padded_size=_FILE_HEADER_LENGTH)

    def _write_message_definition_record(self, topic_info):
        header = {
            'op':    _pack_uint8(_OP_MSG_DEF),
            'topic': topic_info.topic,
            'md5':   topic_info.md5sum,
            'type':  topic_info.datatype,
            'def':   topic_info.msg_def
        }
        _write_record(self._output_file, header)
        
    def _write_message_data_record(self, topic, t, serialized_bytes):
        header = {
            'op':    _pack_uint8(_OP_MSG_DATA),
            'topic': topic,
            'time':  _pack_time(t)
        }
        _write_record(self._output_file, header, serialized_bytes)

    def _write_chunk_header(self, chunk_header):
        header = {
            'op':          _pack_uint8(_OP_CHUNK),
            'compression': chunk_header.compression,
            'size':        _pack_uint32(chunk_header.uncompressed_size)
        }
        _write_record_header(self._file, header)

        self._file.write(_pack_uint32(chunk_header.compressed_size))

    def _write_topic_index_record(self, topic, entries):        
        header = {
            'op':    _pack_uint8(_OP_INDEX_DATA),
            'topic': topic,
            'ver':   _pack_uint32(_INDEX_VERSION),
            'count': _pack_uint32(len(entries))
        }

        buffer = self._buffer
        buffer.seek(0)
        buffer.truncate(0)            
        for entry in entries:
            buffer.write(_pack_time  (entry.time))
            buffer.write(_pack_uint32(entry.offset))
            
        _write_record(self._file, header, buffer.getvalue())            

    def _write_chunk_info_record(self, chunk_info):
        header = {
            'op':         _pack_uint8 (_OP_CHUNK_INFO),
            'ver':        _pack_uint32(_CHUNK_INDEX_VERSION),
            'chunk_pos':  _pack_uint64(chunk_info.pos),
            'start_time': _pack_time(chunk_info.start_time),
            'end_time':   _pack_time(chunk_info.end_time),
            'count':      _pack_uint32(len(chunk_info.topic_counts))
        }
        
        buffer = self._buffer
        buffer.seek(0)
        buffer.truncate(0)
        for topic, count in chunk_info.topic_counts.items():
            buffer.write(_pack_uint32(len(topic)))
            buffer.write(topic)
            buffer.write(_pack_uint32(count))

        _write_record(self._file, header, buffer.getvalue())    

### Implementation ###

_OP_MSG_DEF     = 0x01
_OP_MSG_DATA    = 0x02
_OP_FILE_HEADER = 0x03
_OP_INDEX_DATA  = 0x04
_OP_CHUNK       = 0x05
_OP_CHUNK_INFO  = 0x06

_VERSION             = '#ROSBAG V1.3'
_FILE_HEADER_LENGTH  = 4096
_INDEX_VERSION       = 1
_CHUNK_INDEX_VERSION = 1

class _TopicInfo(object):
    def __init__(self, topic, datatype, md5sum, msg_def):
        self.topic    = topic
        self.datatype = datatype
        self.md5sum   = md5sum
        self.msg_def  = msg_def

    def __str__(self):
        return '%s: %s [%s]' % (self.topic, self.datatype, self.md5sum)

class _ChunkInfo(object):
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

class _ChunkHeader(object):
    def __init__(self, compression, compressed_size, uncompressed_size, data_pos=0):
        self.compression       = compression
        self.compressed_size   = compressed_size
        self.uncompressed_size = uncompressed_size
        self.data_pos          = data_pos

    def __str__(self):
        ratio = 100 * (float(self.compressed_size) / self.uncompressed_size)
        return 'compression: %s, size: %d, uncompressed: %d (%.2f%%)' % (self.compression, self.compressed_size, self.uncompressed_size, ratio)

class _IndexEntry102(object):
    def __init__(self, time, offset):
        self.time   = time
        self.offset = offset
        
    def __str__(self):
        return '%d.%d: %d' % (self.time.secs, self.time.nsecs, self.offset)

class _IndexEntry103(object):
    def __init__(self, time, chunk_pos, offset):
        self.time      = time
        self.chunk_pos = chunk_pos
        self.offset    = offset

    def __str__(self):
        return '%d.%d: %d+%d' % (self.time.secs, self.time.nsecs, self.chunk_pos, self.offset)

def _read_uint8 (f): return _unpack_uint8 (f.read(1))
def _read_uint32(f): return _unpack_uint32(f.read(4))
def _read_uint64(f): return _unpack_uint64(f.read(8))
def _read_time  (f): return _unpack_time  (f.read(8))

def _unpack_uint8(v):  return struct.unpack('<B', v)[0]
def _unpack_uint32(v): return struct.unpack('<L', v)[0]
def _unpack_uint64(v): return struct.unpack('<Q', v)[0]
def _unpack_time(v):   return rospy.Time(*struct.unpack('<LL', v))

def _pack_uint8(v):  return struct.pack('<B', v)
def _pack_uint32(v): return struct.pack('<L', v)
def _pack_uint64(v): return struct.pack('<Q', v)
def _pack_time(v):   return _pack_uint32(v.secs) + _pack_uint32(v.nsecs)

def _read(f, size):
    data = f.read(size)
    if len(data) != size:
        raise ROSBagException('expecting %d bytes, read %d' % (size, len(data)))   
    return data

def _read_sized(f):
    size = _read_uint32(f)
    return _read(f, size)

def _write_sized(f, v):
    f.write(_pack_uint32(len(v)))
    f.write(v)

def _read_field(header, field, unpack_fn):
    if field not in header:
        raise ROSBagFormatException('expected "%s" field in record' % field)
    
    try:
        value = unpack_fn(header[field])
    except Exception, ex:
        raise ROSBagFormatException('error reading field "%s": %s' % (field, str(ex)))
    
    return value

def _read_str_field   (header, field): return _read_field(header, field, lambda v: v)
def _read_uint8_field (header, field): return _read_field(header, field, _unpack_uint8)
def _read_uint32_field(header, field): return _read_field(header, field, _unpack_uint32)
def _read_uint64_field(header, field): return _read_field(header, field, _unpack_uint64)
def _read_time_field  (header, field): return _read_field(header, field, _unpack_time)

def _write_record(f, header, data='', padded_size=None):
    header_str = _write_record_header(f, header)

    if padded_size is not None:
        header_len = len(header_str)
        if header_len < padded_size:
            data = ' ' * (padded_size - header_len)
        else:
            data = ''

    _write_sized(f, data)

def _write_record_header(f, header):
    header_str = ''.join([_pack_uint32(len(k) + 1 + len(v)) + k + '=' + v for k, v in header.items()])
    _write_sized(f, header_str)
    return header_str

def _read_record_header(f, req_op=None):
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

    # Check the op code of the header, if supplied
    if req_op is not None:
        op = _read_uint8_field(header_dict, 'op')
        if req_op != op:
            raise ROSBagFormatException('Expected op code: %d, got %d' % (req_op, op))

    return header_dict

def _read_record_data(f):
    try:
        record_data = _read_sized(f)
    except ROSBagException, ex:
        raise ROSBagFormatException('Error reading record data: %s' % str(ex))

    return record_data

class _BagReader(object):
    def __init__(self, bag):
        self.bag = bag
        
        self.message_types = {}  # md5sum -> type
        
    def read_message_definition_record(self, header=None):
        if not header:
            header = _read_record_header(self.bag._file, _OP_MSG_DEF)

        topic    = _read_str_field(header, 'topic')
        datatype = _read_str_field(header, 'type')
        md5sum   = _read_str_field(header, 'md5')
        msg_def  = _read_str_field(header, 'def')

        _read_record_data(self.bag._file)

        return _TopicInfo(topic, datatype, md5sum, msg_def)
    
    def get_message_type(self, topic_info):
        md5sum, datatype, msg_def = topic_info.md5sum, topic_info.datatype, topic_info.msg_def
        
        message_type = self.message_types.get(md5sum)
        if message_type is None:            
            try:
                message_type = roslib.genpy.generate_dynamic(datatype, msg_def)[datatype]
            except roslib.genpy.MsgGenerationException, ex:
                raise ROSBagException('Error generating datatype %s: %s' % (datatype, str(ex)))

            self.message_types[md5sum] = message_type

        return message_type

class _BagReader102_Unindexed(_BagReader):
    """
    Support class for reading unindexed v1.2 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)

    def start_reading(self):
        pass

    def get_messages(self):
        f = self.bag._file

        while True:
            # Read MSG_DEF records
            while True:
                try:
                    header = _read_record_header(f)
                except:
                    return

                op = _read_uint8_field(header, 'op')
                if op != _OP_MSG_DEF:
                    break

                topic_info = self._read_message_definition_record(header)
                self.bag._topic_infos[topic_info.topic] = topic_info

            # Check that we have a MSG_DATA record
            if op != _OP_MSG_DATA:
                raise ROSBagFormatException('Expecting OP_MSG_DATA, got %d' % op)
    
            topic = _read_str_field(header, 'topic')
            topic_info = self.bag._topic_infos[topic]
    
            # Get the message type
            try:
                msg_type = self.get_message_type(topic_info)
            except KeyError:
                raise ROSBagException('Cannot deserialize messages of type [%s].  Message was not preceeded in bagfile by definition' % topic_info.datatype)

            secs  = _read_uint32_field(header, 'secs')
            nsecs = _read_uint32_field(header, 'nsecs')
            t = roslib.rostime.Time(secs, nsecs)
   
            # Read the message content
            record_data = _read_record_data(f)
            
            # Deserialize the message
            msg = msg_type()
            msg.deserialize(record_data)

            yield (topic, msg, t)

class _BagReader102_Indexed(_BagReader):
    """
    Support class for reading indexed v1.2 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)

    def get_messages(self):
        f = self.bag._file

        for entry, _ in _mergesort(self.bag._topic_indexes.values(), key=lambda entry: entry.time):
            f.seek(entry.offset)
            topic, msg = self.read_message_data_record(entry)
            yield (topic, msg, entry.time)

    def start_reading(self):
        self.read_file_header_record()

        # Seek to the beginning of the topic index records
        self.bag._file.seek(self.bag._index_data_pos)

        while True:
            topic_index = self.read_topic_index_record()
            if topic_index is None:
                break
            (topic, index) = topic_index
            self.bag._topic_indexes[topic] = index

        # Read the message definition records (one for each topic)
        for topic, index in self.bag._topic_indexes.items():
            self.bag._file.seek(index[0].offset)
            
            topic_info = self.read_message_definition_record()
            self.bag._topic_infos[topic_info.topic] = topic_info

    def read_file_header_record(self):
        header = _read_record_header(self.bag._file, _OP_FILE_HEADER)

        self.bag._index_data_pos = _read_uint64_field(header, 'index_pos')

        _read_record_data(self.bag._file)  # skip over padding

    def read_topic_index_record(self):
        f = self.bag._file

        try:
            header = _read_record_header(f, _OP_INDEX_DATA)
        except struct.error:
            return None
        
        index_version = _read_uint32_field(header, 'ver')
        topic         = _read_str_field   (header, 'topic')
        count         = _read_uint32_field(header, 'count')
        
        if index_version != 0:
            raise ROSBagFormatException('expecting index version 0, got %d' % index_version)
    
        _read_uint32(f) # skip the record data size

        topic_index = []
                
        for i in range(count):
            time   = _read_time  (f)
            offset = _read_uint64(f)
            
            topic_index.append(_IndexEntry102(time, offset))
            
        return (topic, topic_index)
    
    def read_message_data_record(self):
        f = self.bag._file

        # Skip any MSG_DEF records
        while True:
            header = _read_record_header(f)
            op = _read_uint8_field(header, 'op')
            if op != _OP_MSG_DEF:
                break
            _read_record_data(f)

        # Check that we have a MSG_DATA record
        if op != _OP_MSG_DATA:
            raise ROSBagFormatException('Expecting OP_MSG_DATA, got %d' % op)
        
        topic = _read_str_field(header, 'topic')

        # Get the message type
        topic_info = self.bag._topic_infos[topic]
        try:
            msg_type = self.get_message_type(topic_info)
        except KeyError:
            raise ROSBagException('Cannot deserialize messages of type [%s].  Message was not preceeded in bagfile by definition' % topic_info.datatype)

        # Read the message content
        record_data = _read_record_data(f)
        
        # Deserialize the message
        msg = msg_type()
        msg.deserialize(record_data)
        
        return (topic, msg)

class _BagReader103(_BagReader):
    """
    Support class for reading v1.3 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)
        
        self.decompressed_chunk_pos = None
        self.decompressed_chunk     = None
        self.decompressed_chunk_io  = None

    def get_messages(self):
        for entry, _ in _mergesort(self.bag._topic_indexes.values(), key=lambda entry: entry.time):
            topic, msg = self.read_message_data_record(entry)
            yield (topic, msg, entry.time)

    def start_reading(self):
        self.read_file_header_record()

        # Check if the index position has been written, i.e. the bag was closed successfully
        if self.bag._index_data_pos == 0:
            print 'invalid index data position'
            # @todo: reconstruct the index by reading in the chunks
            return

        # Seek to the end of the chunks
        self.bag._file.seek(self.bag._index_data_pos)        

        # Read the message definition records (one for each topic)
        for i in range(self.bag._topic_count):
            topic_info = self.read_message_definition_record()
            self.bag._topic_infos[topic_info.topic] = topic_info

        # Read the chunk info records
        self.bag._chunk_infos = []
        for i in range(self.bag._chunk_count):
            chunk_info = self.read_chunk_info_record()
            self.bag._chunk_infos.append(chunk_info)

        # Read the chunk headers and topic indexes
        self.bag._topic_indexes = {}
        self.bag._chunk_headers = {}
        for chunk_info in self.bag._chunk_infos:
            self.bag._curr_chunk_info = chunk_info
            
            self.bag._file.seek(chunk_info.pos)
    
            # Remember the chunk header
            chunk_header = self.read_chunk_header()
            self.bag._chunk_headers[chunk_info.pos] = chunk_header

            # Skip over the chunk data
            self.bag._file.seek(chunk_header.compressed_size, os.SEEK_CUR)

            # Read the topic index records after the chunk
            for i in range(len(chunk_info.topic_counts)):
                (topic, index) = self.read_topic_index_record()
                self.bag._topic_indexes[topic] = index

    def read_file_header_record(self):
        header = _read_record_header(self.bag._file, _OP_FILE_HEADER)

        self.bag._index_data_pos = _read_uint64_field(header, 'index_pos')
        self.bag._chunk_count    = _read_uint32_field(header, 'chunk_count')
        self.bag._topic_count    = _read_uint32_field(header, 'topic_count')

        _read_record_data(self.bag._file)

    def read_chunk_info_record(self):
        f = self.bag._file
        
        header = _read_record_header(f, _OP_CHUNK_INFO)

        chunk_info_version = _read_uint32_field(header, 'ver')
        
        if chunk_info_version == 1:
            chunk_pos   = _read_uint64_field(header, 'chunk_pos')
            start_time  = _read_time_field  (header, 'start_time')
            end_time    = _read_time_field  (header, 'end_time')
            topic_count = _read_uint32_field(header, 'count') 
    
            chunk_info = _ChunkInfo(chunk_pos, start_time, end_time)
    
            _read_uint32(f)   # skip the record data size

            for i in range(topic_count):
                topic_name  = _read_sized(f)
                topic_count = _read_uint32(f)
    
                chunk_info.topic_counts[topic_name] = topic_count
                
            return chunk_info
        else:
            raise ROSBagFormatException('Unknown chunk info record version: %d' % chunk_info_version)

    def read_chunk_header(self):
        header = _read_record_header(self.bag._file, _OP_CHUNK)

        compression       = _read_str_field   (header, 'compression')
        uncompressed_size = _read_uint32_field(header, 'size')

        compressed_size = _read_uint32(self.bag._file)  # read the record data size

        data_pos = self.bag._file.tell()

        return _ChunkHeader(compression, compressed_size, uncompressed_size, data_pos)

    def read_topic_index_record(self):
        f = self.bag._file

        header = _read_record_header(f, _OP_INDEX_DATA)
        
        index_version = _read_uint32_field(header, 'ver')
        topic         = _read_str_field   (header, 'topic')
        count         = _read_uint32_field(header, 'count')
        
        if index_version != 1:
            raise ROSBagFormatException('expecting index version 1, got %d' % index_version)
    
        _read_uint32(f) # skip the record data size

        topic_index = []
                
        for i in range(count):
            time   = _read_time  (f)
            offset = _read_uint32(f)
            
            topic_index.append(_IndexEntry103(time, self.bag._curr_chunk_info.pos, offset))
            
        return (topic, topic_index)

    def read_message_data_record(self, entry):
        chunk_pos, offset = entry.chunk_pos, entry.offset

        chunk_header = self.bag._chunk_headers.get(chunk_pos)
        if chunk_header is None:
            raise ROSBagException('no chunk at position %d' % chunk_pos)

        if chunk_header.compression == Compression.NONE:
            f = self.bag._file
            f.seek(chunk_header.data_pos + offset)
        else:
            if self.decompressed_chunk_pos != chunk_pos:
                # Seek to the chunk data, read and decompress
                self.bag._file.seek(chunk_header.data_pos)
                compressed_chunk = _read(self.bag._file, chunk_header.compressed_size)

                if chunk_header.compression == Compression.BZ2:
                    self.decompressed_chunk = bz2.decompress(compressed_chunk)
                else:
                    raise ROSBagException('unsupported compression type: %s' % chunk_header.compression)
                
                self.decompressed_chunk_pos = chunk_pos

                if self.decompressed_chunk_io:
                    self.decompressed_chunk_io.close()
                self.decompressed_chunk_io = StringIO(self.decompressed_chunk)

            f = self.decompressed_chunk_io
            f.seek(offset)

        # Skip any MSG_DEF records
        while True:
            header = _read_record_header(f)
            op = _read_uint8_field(header, 'op')
            if op != _OP_MSG_DEF:
                break
            _read_record_data(f)

        # Check that we have a MSG_DATA record
        if op != _OP_MSG_DATA:
            raise ROSBagFormatException('Expecting OP_MSG_DATA, got %d' % op)

        topic = _read_str_field(header, 'topic')

        # Get the message type
        topic_info = self.bag._topic_infos[topic]
        try:
            msg_type = self.get_message_type(topic_info)
        except KeyError:
            raise ROSBagException('Cannot deserialize messages of type [%s].  Message was not preceeded in bagfile by definition' % topic_info.datatype)

        # Read the message content
        record_data = _read_record_data(f)
        
        # Deserialize the message
        msg = msg_type()
        msg.deserialize(record_data)
        
        return (topic, msg)

## See http://code.activestate.com/recipes/511509
def _mergesort(list_of_lists, key=None):
    """
    Perform an N-way merge operation on sorted lists.

    @param list_of_lists: (really iterable of iterable) of sorted elements
    (either by naturally or by C{key})
    @param key: specify sort key function (like C{sort()}, C{sorted()})
    @param iterfun: function that returns an iterator.

    Yields tuples of the form C{(item, iterator)}, where the iterator is the
    built-in list iterator or something you pass in, if you pre-generate the
    iterators.

    This is a stable merge; complexity O(N lg N)

    Examples::

    print list(x[0] for x in mergesort([[1,2,3,4],
                                        [2,3.5,3.7,4.5,6,7],
                                        [2.6,3.6,6.6,9]]))
    [1, 2, 2, 2.6, 3, 3.5, 3.6, 3.7, 4, 4.5, 6, 6.6, 7, 9]

    # note stability
    print list(x[0] for x in mergesort([[1,2,3,4],
                                        [2,3.5,3.7,4.5,6,7],
                                        [2.6,3.6,6.6,9]], key=int))
    [1, 2, 2, 2.6, 3, 3.5, 3.6, 3.7, 4, 4.5, 6, 6.6, 7, 9]

    print list(x[0] for x in mergesort([[4,3,2,1],
                                        [7,6.5,4,3.7,3.3,1.9],
                                        [9,8.6,7.6,6.6,5.5,4.4,3.3]],
                                        key=lambda x: -x))
    [9, 8.6, 7.6, 7, 6.6, 6.5, 5.5, 4.4, 4, 4, 3.7, 3.3, 3.3, 3, 2, 1.9, 1]
    """

    heap = []
    for i, itr in enumerate(iter(pl) for pl in list_of_lists):
        try:
            item = itr.next()
            toadd = (key(item), i, item, itr) if key else (item, i, itr)
            heap.append(toadd)
        except StopIteration:
            pass
    heapq.heapify(heap)

    if key:
        while heap:
            _, idx, item, itr = heap[0]
            yield item, itr
            try:
                item = itr.next()
                heapq.heapreplace(heap, (key(item), idx, item, itr) )
            except StopIteration:
                heapq.heappop(heap)

    else:
        while heap:
            item, idx, itr = heap[0]
            yield item, itr
            try:
                heapq.heapreplace(heap, (itr.next(), idx, itr))
            except StopIteration:
                heapq.heappop(heap)

class _BZ2CompressorFileFacade(object):
    """
    A file facade for the bz2.BZ2Compressor.
    """
    def __init__(self, file):
        self.file                = file
        self.compressor          = bz2.BZ2Compressor()
        self.compressed_bytes_in = 0
    
    def write(self, data):
        compressed = self.compressor.compress(data)
        if len(compressed) > 0:
            self.file.write(compressed)
        self.compressed_bytes_in += len(data)
    
    def flush(self):
        compressed = self.compressor.flush()
        if len(compressed) > 0:
            self.file.write(compressed)
