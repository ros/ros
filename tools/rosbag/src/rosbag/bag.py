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

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)

import bz2
from cStringIO import StringIO
import heapq
import numpy
import os
import re
import struct
import time

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
    #ZLIB = 'zlib'

class Bag(object):
    """
    Bag objects serialize messages to and from disk using the bag format.
    """
    def __init__(self, f, mode='r', compression=Compression.NONE, chunk_threshold=768 * 1024):
        """
        Open a bag file.  The mode can be 'r', 'w', or 'a' for reading (default),
        writing or appending.  The file will be created if it doesn't exist
        when opened for writing or appending; it will be truncated when opened
        for writing.  Simultaneous reading and writing is allowed when in writing
        or appending mode.        
        @param f: filename of bag to open or a stream to read from
        @type  f: str or file
        @param mode: mode, either 'r', 'w', or 'a'
        @type  mode: str
        @param compression: compression mode, see Compression
        @type  compression: str
        @param chunk_threshold: size in bytes of chunks
        @type  chunk_threshold: int
        @raise ValueError: if any argument is invalid
        @raise ROSBagException: if an error occurs opening file
        @raise ROSBagFormatException: if bag format is corrupted
        """
        self._file            = None
        self._filename        = None
        self._version         = None
        
        allowed_compressions = [Compression.NONE, Compression.BZ2] #, Compression.ZLIB]
        if compression not in allowed_compressions:
            raise ValueError('compression must be one of: %s' % ', '.join(allowed_compressions))  
        self._compression = compression      

        if chunk_threshold < 0:
            raise ValueError('chunk_threshold must be greater than or equal to zero')        
        self._chunk_threshold = chunk_threshold

        self._reader          = None
        
        self._topic_infos     = {}    # topic -> TopicInfo
        self._file_header_pos = None

        self._index_data_pos  = 0     # (1.2+)
        self._topic_indexes   = {}    # topic -> IndexEntry[] (1.2+)

        self._topic_count     = 0     # (2.0)
        self._chunk_count     = 0     # (2.0)
        self._chunk_infos     = []    # ChunkInfo[] (2.0)
        self._chunk_headers   = {}    # chunk_pos -> ChunkHeader (2.0)
        
        self._buffer                   = StringIO()        

        self._chunk_open               = False
        self._curr_chunk_info          = None
        self._curr_chunk_data_pos      = None
        self._curr_chunk_topic_indexes = {}

        self._curr_compression = Compression.NONE
        self._output_file      = self._file
        
        self._open(f, mode)
    
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
        if not self._file:
            raise ValueError('I/O operation on closed bag')
        
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
        allowed_compressions = [Compression.NONE, Compression.BZ2] #, Compression.ZLIB]
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
        
    chunk_threshold = property(_get_chunk_threshold, _set_chunk_threshold)

    def _read_message(self, entry, raw):
        self.flush()
        return self._reader.read_message(entry, raw)

    def getIndex(self):
        """
        Get the index.
        @return: the index
        @rtype: dict of topic -> (U{roslib.rostime.Time}, position), where position is dependent on the bag format
        """
        if not self._topic_indexes:
            raise ROSBagException('getIndex not supported on unindexed bag files')

        return self._reader.get_index()

    def readMessages(self, topics=None, start_time=None, end_time=None, topic_filter=None, raw=False):
        """
        Read the messages from the bag file.
        @param topic: list of topics [optional]
        @type  topic: list(str)
        @param start_time: earliest timestamp of message to return [optional]
        @type  start_time: U{roslib.rostime.Time}
        @param end_time: latest timestamp of message to return [optional]
        @type  end_time: U{roslib.rostime.Time}
        @param topic_filter: function to filter topics to include [optional]
        @type  topic_filter: function taking (topic, datatype, md5sum, msg_def) and returning bool
        @param raw: if True, then generate tuples of (datatype, data, md5sum, position, pytype)
        @type  raw: bool
        @return: generator of (topic, message, timestamp) tuples for each message in the bag file
        @rtype: generator of tuples of (topic, message, timestamp)
        """
        self.flush()
        return self._reader.read_messages(topics, start_time, end_time, topic_filter, raw)

    def flush(self):
        """
        Write the open chunk to disk so subsequent reads will read all messages.
        @raise ValueError: if bag is closed 
        """
        if not self._file:
            raise ValueError('I/O operation on closed bag')

        if self._chunk_open:
            self._stop_writing_chunk()

    def write(self, topic, msg, t=None, raw=False):
        """
        Write a message to the bag.  Messages must be written in chronological order for a given topic.
        @param topic: name of topic
        @type  topic: str
        @param msg: message to add to bag, or tuple (if raw)
        @type  msg: Message or tuple of raw message data
        @param t: ROS time of message publication
        @type  t: U{roslib.rostime.Time}
        @param raw: if True, msg is in raw format, i.e. (msg_type, serialized_bytes, md5sum, pytype)
        @type  raw: bool
        @raise ValueError: if arguments are invalid or bag is closed
        @raise ROSBagException: if message isn't in chronological order
        """
        if not self._file:
            raise ValueError('I/O operation on closed bag')
        
        if not topic:
            raise ValueError('topic is invalid')
        if not msg:
            raise ValueError('msg is invalid')

        if t is None:
            t = roslib.rostime.Time.from_sec(time.time())

        # Seek to end (in case previous operation was a read)
        self._file.seek(0, os.SEEK_END)

        # Open a chunk, if needed
        if not self._chunk_open:
            self._start_writing_chunk(t)

        # Write message definition record, if necessary
        if topic not in self._topic_infos:
            if raw:
                msg_type, serialized_bytes, md5sum, pytype = msg
    
                if pytype is None:
                    try:
                        pytype = roslib.message.get_message_class(msg_type)
                    except Exception:
                        pytype = None
                if pytype is None:
                    raise ROSBagException('cannot locate message class and no message class provided for [%s]' % msg_type)
    
                if pytype._md5sum != md5sum:
                    print >> sys.stderr, 'WARNING: md5sum of loaded type [%s] does not match that specified' % msg_type
                    #raise ROSBagException('md5sum of loaded type does not match that of data being recorded')
            
                topic_info = _TopicInfo(topic, msg_type, md5sum, pytype._full_text)
            else:
                topic_info = _TopicInfo(topic, msg.__class__._type, msg.__class__._md5sum, msg._full_text)
                
            self._write_message_definition_record(topic_info)
            self._topic_infos[topic] = topic_info

        # Update the current chunk's topic index
        index_entry = _IndexEntry200(t, self._curr_chunk_info.pos, self._get_chunk_offset())

        topic_index = self._curr_chunk_topic_indexes.get(topic)
        if topic_index is None:
            self._curr_chunk_topic_indexes[topic] = [index_entry]
            self._curr_chunk_info.topic_counts[topic] = 1
        else:
            last_message_time = topic_index[-1].time
            if t < last_message_time:
                raise ROSBagException('received messages not in chronological order on topic %s (%s received after %s)' % (topic, str(t), str(last_message_time)))
            topic_index.append(index_entry)

            self._curr_chunk_info.topic_counts[topic] += 1

        # Update the chunk start/end times
        if t > self._curr_chunk_info.end_time:
            self._curr_chunk_info.end_time = t
        elif t < self._curr_chunk_info.start_time:
            self._curr_chunk_info.start_time = t

        if not raw:
            # Serialize the message to the buffer
            self._buffer.seek(0)
            self._buffer.truncate(0)
            msg.serialize(self._buffer)
            serialized_bytes = self._buffer.getvalue()

        # Write message data record
        self._write_message_data_record(topic, t, serialized_bytes)
        
        # Check if we want to stop this chunk
        chunk_size = self._get_chunk_offset()
        if chunk_size > self._chunk_threshold:
            self._stop_writing_chunk()

    def close(self):
        """
        Close the bag file.  Closing an already closed bag does nothing.
        """
        if self._file:
            if self._mode in 'wa':
                self._stop_writing()
            
            self._close_file()

    def __str__(self):
        s = ''
        if self._filename:
            s += 'bag:          %s (%s)\n' % (self._filename, self._mode)
        s += 'version:      %d.%d\n' % (self._version / 100, self._version % 100)

        if not self._topic_indexes:
            return s.rstrip()
        
        # Show start and end times
        start_stamp = min([index[ 0].time.to_sec() for index in self._topic_indexes.values()])
        end_stamp   = max([index[-1].time.to_sec() for index in self._topic_indexes.values()])
        s += 'start:        %s (%.2f)\n' % (_time_to_str(start_stamp), start_stamp)
        s += 'end:          %s (%.2f)\n' % (_time_to_str(end_stamp),   end_stamp)

        # Show length
        duration = end_stamp - start_stamp
        dur_secs = duration % 60
        dur_mins = int(duration / 60)
        dur_hrs  = int(dur_mins / 60)
        if dur_hrs > 0:
            dur_mins = dur_mins % 60
            s += 'length:       %dhr %d:%ds (%ds)\n' % (dur_hrs, dur_mins, dur_secs, duration)
        elif dur_mins > 0:
            s += 'length:       %d:%ds (%ds)\n' % (dur_mins, dur_secs, duration)
        else:
            s += 'length:       %.1fs\n' % duration

        s += 'messages:     %d\n' % (sum([len(index) for index in self._topic_indexes.values()]))

        # Calculate total size of message chunks
        total_uncompressed_size = sum((h.uncompressed_size for h in self._chunk_headers.values()))
        total_compressed_size   = sum((h.compressed_size   for h in self._chunk_headers.values()))
        s += 'uncompressed: %9s\n' % _human_readable_size(total_uncompressed_size)
        if total_compressed_size != total_uncompressed_size:
            s += 'compressed:   %9s (%.2f%%)\n' % (_human_readable_size(total_compressed_size), (100.0 * total_compressed_size) / total_uncompressed_size)

        # Calculate message sizes (not including first, as it's a MSG_DEF record, and last, as we don't have a succeeding offset)
        #entries = [entry for entry, _ in _mergesort(self._topic_indexes.values(), key=lambda entry: entry.time)]
        #entry_sizes = {}
        #for i in range(len(entries) - 1):
        #    offset0 = entries[i].offset
        #    offset1 = entries[i + 1].offset            
        #    if offset1 > offset0:
        #        entry_sizes[entries[i]] = offset1 - offset0

        # Show topics
        s += 'topics:'
        topics = sorted(self._topic_infos.keys())        
        max_topic_len = max([len(topic.lstrip('/')) for topic in topics])
        max_datatype_len = max([len(self._topic_infos[topic].datatype) for topic in topics])
        for i, topic in enumerate(topics):
            indent = (7 if i == 0 else 14)
            
            index      = self._topic_indexes[topic]
            topic_info = self._topic_infos[topic]
            stamps     = numpy.array([entry.time.to_sec() for entry in index])
            msg_count  = len(stamps)

            s += '%s%-*s (%-*s) %7d msgs' % (' ' * indent, max_topic_len, topic, max_datatype_len, topic_info.datatype, msg_count)
            
            if msg_count > 1:
                spacing = stamps[1:] - stamps[:-1]
                s += ' @ %5.1f Hz' % (1.0 / numpy.median(spacing))

                #sizes = []
                #for entry in index:
                #    if entry in entry_sizes:
                #        sizes.append(entry_sizes[entry])
                #s += ' %d to %d bytes' % (min(sizes), max(sizes))

            s += '\n'

        # Show unique datatypes
        datatypes = set()
        datatype_infos = []
        for topic_info in self._topic_infos.values():
            if topic_info.datatype in datatypes:
                continue
            datatype_infos.append((topic_info.datatype, topic_info.md5sum, topic_info.msg_def))
            datatypes.add(topic_info.datatype)
        
        s += 'types:'
        for i, (datatype, md5sum, msg_def) in enumerate(sorted(datatype_infos)):
            indent = (8 if i == 0 else 14)

            s += '%s%-*s [%s]\n' % (' ' * indent, max_datatype_len, datatype, md5sum)

#            string def = content.definition.c_str();
#            if (def.length() > 0) {
#                printf("    definition: |\n");
#
#                size_t oldind = 0;
#                size_t ind = def.find_first_of('\n', 0);
#                while (ind != def.npos) {
#                    printf("      %s\n", def.substr(oldind, ind - oldind).c_str());
#                    oldind = ind + 1;
#                    ind = def.find_first_of('\n', oldind);
#                }
#                ind = def.length();
#
#                printf("      %s\n", def.substr(oldind, ind - oldind).c_str());
#            }
#            else {
#                printf("    definition: NONE\n");
#            }

        return s.rstrip()

    ### Implementation ###

    def _open(self, f, mode):
        if not f:
            raise ValueError('filename (or stream) is invalid')

        if   mode == 'r': self._open_read(f)
        elif mode == 'w': self._open_write(f)
        elif mode == 'a': self._open_append(f)
        else:
            raise ValueError('mode "%s" is invalid' % mode)

    def _open_read(self, f):
        if isinstance(f, file):
            self._file = f
        else:
            self._file     = open(f, 'rb')
            self._filename = f        

        self._mode = 'r'

        try:
            self._version = self._read_version()
        except:
            self._version = None
            self._close_file()
            raise

        try:
            self._create_reader()
            self._reader.start_reading()
        except:
            self._close_file()
            raise

    def _open_write(self, f):
        if isinstance(f, file):
            self._file = f
        else:
            self._file     = open(f, 'wb')
            self._filename = f
            
        self._mode = 'w'

        try:
            self._start_writing()
        except:
            self._close_file()
            raise

    def _open_append(self, f):
        if isinstance(f, file):
            self._file = f
        else:        
            try:
                # Test if the file already exists.
                open(f, 'r').close()

                # File exists, Open in read with update mode.
                self._file = open(f, 'r+b')
            except:
                # File doesn't exist. Open in write mode.
                self._file = open(f, 'w+b')
        
            self._filename = f

        self._mode = 'a'

        try:
            self._version = self._read_version()
        except:
            self._version = None
            self._close_file()
            raise

        if self._version != 200:
            self._file.close()
            raise ROSBagException('bag version %d is unsupported for appending' % self._version)

        try:
            self._start_appending()
        except:
            self._close_file()
            raise
        
    def _close_file(self):
        self._file.close()
        self._file = None

    def _create_reader(self):
        """
        @raise ROSBagException: if the bag version is unsupported
        """
        if self._version == 200:
            self._reader = _BagReader200(self)
        elif self._version == 102:
            # Get the op code of the first record.  If it's FILE_HEADER, then we have an indexed 1.2 bag.
            first_record_pos = self._file.tell()
            header = _read_record_header(self._file)
            op = _read_uint8_field(header, 'op')
            self._file.seek(first_record_pos)

            if op == _OP_FILE_HEADER:
                self._reader = _BagReader102_Indexed(self)
            else:
                self._reader = _BagReader102_Unindexed(self)
        elif self._version == 101:
            self._reader = _BagReader101(self)
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

    def _start_appending(self):
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
        if self._curr_compression != Compression.NONE:
            self._output_file.flush()
        
        # Create the compressor
        if compression == Compression.BZ2:
            self._output_file = _BZ2CompressorFileFacade(self._file)
        elif compression == Compression.NONE:
            self._output_file = self._file
        else:
            raise ROSBagException('unsupported compression type: %s' % compression)

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

_message_types = {}   # md5sum -> type

_OP_MSG_DEF     = 0x01
_OP_MSG_DATA    = 0x02
_OP_FILE_HEADER = 0x03
_OP_INDEX_DATA  = 0x04
_OP_CHUNK       = 0x05
_OP_CHUNK_INFO  = 0x06

_VERSION             = '#ROSBAG V2.0'
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

class _IndexEntry200(object):
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
        
    def start_reading(self): pass
    def read_messages(self, topics, start_time, end_time, topic_filter, raw): pass

    def get_entries(self, topics, start_time, end_time, topic_filter):
        for entry, _ in _mergesort(self.get_indexes(topics, topic_filter), key=lambda entry: entry.time):
            if start_time and entry.time < start_time:
                continue
            if end_time and entry.time > end_time:
                return
            yield entry

    def get_indexes(self, topics, topic_filter):
        if topics is None and topic_filter is None:
            return self.bag._topic_indexes.values()

        if topics is None:
            topics = self.bag._topic_indexes.keys()
    
        if topic_filter is not None:
            filtered_topics = []
            for topic in topics:
                if topic not in self.bag._topic_infos:
                    continue
                topic_info = self.bag._topic_infos[topic]
                if topic_filter(topic, topic_info.datatype, topic_info.md5sum, topic_info.msg_def):
                    filtered_topics.append(topic)
            topics = filtered_topics

        return [index for topic, index in self.bag._topic_indexes.items() if topic in topics]
    
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
        
        message_type = _message_types.get(md5sum)
        if message_type is None:            
            try:
                message_type = roslib.genpy.generate_dynamic(datatype, msg_def)[datatype]
            except roslib.genpy.MsgGenerationException, ex:
                raise ROSBagException('Error generating datatype %s: %s' % (datatype, str(ex)))

            _message_types[md5sum] = message_type

        return message_type

class _BagReader101(_BagReader):
    """
    Support class for reading v1.1 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)

    def start_reading(self):
        pass

    def read_messages(self, topics, start_time, end_time, topic_filter, raw):
        f = self.bag._file
        
        while True:
            bag_pos = f.tell()
            
            # Read topic/md5/type string headers
            topic = f.readline().rstrip()
            if not topic:
                return

            md5sum = f.readline().rstrip()
            datatype = f.readline().rstrip()
            
            # Migration for rostools->roslib rename
            if datatype in ['rostools/Header', 'rostools/Log', 'rostools/Time']:
                datatype = datatype.replace('rostools', 'roslib')
          
            # Read timestamp
            data = f.read(12)
            if len(data) != 12:
                raise ROSBagFormatException('Bag file appears to be corrupt (1)')
            (time_sec, time_nsec, length) = struct.unpack("<LLL", data)
            t = rospy.Time(time_sec, time_nsec)
        
            # Read message
            data = f.read(length)
            if len(data) != length:
                raise ROSBagFormatException('Bag file appears to be corrupt (2)')
          
            try:
                pytype = _message_types[md5sum]
            except KeyError:
                try:
                    pytype = roslib.message.get_message_class(datatype)
                except Exception:
                    pytype = None
          
                if pytype is None:
                    raise ROSBagException('Cannot deserialize messages of type [%s]: cannot locate message class' % datatype)
                else:
                    if pytype._md5sum != md5sum:
                        (package, type) = datatype.split('/')
                    if roslib.gentools.compute_md5_v1(roslib.gentools.get_file_dependencies(roslib.msgs.msg_file(package,type))) == md5sum:
                        print 'In V1.1 Logfile, found old md5sum for type [%s].  Allowing implicit migration to new md5sum.' % datatype
                    else:
                        raise ROSBagException('Cannot deserialize messages of type [%s]: md5sum is outdated in V1.1 bagfile' % datatype)
                _message_types[md5sum] = pytype
          
            if raw:
                msg = datatype, data, pytype._md5sum, bag_pos, pytype
            else:    
                msg = pytype()
                msg.deserialize(data)
            
            yield topic, msg, t

class _BagReader102_Unindexed(_BagReader):
    """
    Support class for reading unindexed v1.2 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)

    def read_messages(self, topics, start_time, end_time, topic_filter, raw):
        f = self.bag._file

        while True:
            # Read MSG_DEF records
            while True:
                position = f.tell()
                
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

            # Get the timestamp
            secs  = _read_uint32_field(header, 'secs')
            nsecs = _read_uint32_field(header, 'nsecs')
            t = roslib.rostime.Time(secs, nsecs)
   
            # Read the message content
            data = _read_record_data(f)
            
            if raw:
                msg = (topic_info.datatype, data, topic_info.md5sum, position, msg_type)
            else:
                # Deserialize the message
                msg = msg_type()
                msg.deserialize(data)

            yield (topic, msg, t)

class _BagReader102_Indexed(_BagReader):
    """
    Support class for reading indexed v1.2 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)

    def read_messages(self, topics, start_time, end_time, topic_filter, raw):
        for entry in self.get_entries(topics, start_time, end_time, topic_filter):
            yield self.read_message(entry, raw)

    def read_message(self, entry, raw):
        topic, msg = self.read_message_data_record(entry, raw)
        return (topic, msg, entry.time)
    
    def get_index(self):
        index = {}
        for topic, entries in self._topic_indexes:
            index[topic] = [(e.time, e.offset) for e in entries]
        return index

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
    
    def read_message_data_record(self, entry, raw):
        f = self.bag._file

        # Seek to the message position
        f.seek(entry.offset)

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
        data = _read_record_data(f)
        
        if raw:
            msg = topic_info.datatype, data, topic_info.md5sum, entry.offset, msg_type
        else:            
            # Deserialize the message
            msg = msg_type()
            msg.deserialize(data)
        
        return (topic, msg)

class _BagReader200(_BagReader):
    """
    Support class for reading v2.0 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)
        
        self.decompressed_chunk_pos = None
        self.decompressed_chunk     = None
        self.decompressed_chunk_io  = None

    def read_messages(self, topics, start_time, end_time, topic_filter, raw):
        for entry in self.get_entries(topics, start_time, end_time, topic_filter):
            yield self.read_message(entry, raw)

    def read_message(self, entry, raw):
        topic, msg = self.read_message_data_record(entry, raw)
        return (topic, msg, entry.time)

    def get_index(self):
        index = {}
        for topic, entries in self._topic_indexes:
            index[topic] = [(e.time, (e.chunk_pos, e.offset)) for e in entries]

        return index

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
            
            topic_index.append(_IndexEntry200(time, self.bag._curr_chunk_info.pos, offset))

        return (topic, topic_index)

    def read_message_data_record(self, entry, raw):
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
        data = _read_record_data(f)
        
        # Deserialize the message
        if raw:
            msg = topic_info.datatype, data, topic_info.md5sum, (chunk_pos, offset), msg_type
        else:
            msg = msg_type()
            msg.deserialize(data)
        
        return (topic, msg)

def _time_to_str(secs):
    secs_frac = secs - int(secs) 
    secs_frac_str = ('%.2f' % secs_frac)[1:]

    return time.strftime('%b %d %Y %H:%M:%S', time.localtime(secs)) + secs_frac_str

def _human_readable_size(size):
    multiple = 1024.0
    for suffix in ['KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB', 'YB']:
        size /= multiple
        if size < multiple:
            return '%.1f %s' % (size, suffix)

    raise ValueError('number too large')

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
