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

"""
The rosbag Python API.

Provides serialization of bag files.
"""

from __future__ import print_function

import bisect
import bz2
import heapq
import os
import re
import struct
import sys
import threading
import time
import yaml

try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import StringIO  # Python 3.x

import genmsg
import genpy
import genpy.dynamic
import genpy.message

import roslib.names # still needed for roslib.names.canonicalize_name()
import rospy

class ROSBagException(Exception):
    """
    Base class for exceptions in rosbag.
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return self.value

class ROSBagFormatException(ROSBagException):
    """
    Exceptions for errors relating to the bag file format.
    """
    def __init__(self, value):
        ROSBagException.__init__(self, value)

class ROSBagUnindexedException(ROSBagException):
    """
    Exception for unindexed bags.
    """
    def __init__(self):
        ROSBagException.__init__(self, 'Unindexed bag')

class Compression:
    """
    Allowable compression types
    """
    NONE = 'none'
    BZ2  = 'bz2'

class Bag(object):
    """
    Bag serialize messages to and from a single file on disk using the bag format.
    """
    def __init__(self, f, mode='r', compression=Compression.NONE, chunk_threshold=768 * 1024, allow_unindexed=False, options=None, skip_index=False):
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
        @param compression: compression mode, see U{rosbag.Compression} for valid modes
        @type  compression: str
        @param chunk_threshold: minimum number of uncompressed bytes per chunk
        @type  chunk_threshold: int
        @param allow_unindexed: if True, allow opening unindexed bags
        @type  allow_unindexed: bool
        @param options: the bag options (currently: compression and chunk_threshold)
        @type  options: dict
        @param skip_index: if True, don't read the connection index records on open [2.0+]
        @type  skip_index: bool
        @raise ValueError: if any argument is invalid
        @raise ROSBagException: if an error occurs opening file
        @raise ROSBagFormatException: if bag format is corrupted
        """
        if options is not None:
            if type(options) is not dict:
                raise ValueError('options must be of type dict')                
            if 'compression' in options:
                compression = options['compression']
            if 'chunk_threshold' in options:
                chunk_threshold = options['chunk_threshold']

        self._file     = None
        self._filename = None
        self._version  = None

        allowed_compressions = [Compression.NONE, Compression.BZ2]
        if compression not in allowed_compressions:
            raise ValueError('compression must be one of: %s' % ', '.join(allowed_compressions))  
        self._compression = compression      

        if chunk_threshold < 0:
            raise ValueError('chunk_threshold must be greater than or equal to zero')        
        self._chunk_threshold = chunk_threshold

        self._skip_index = skip_index

        self._reader          = None

        self._file_header_pos = None
        self._index_data_pos  = 0       # (1.2+)

        self._clear_index()

        self._buffer = StringIO()        

        self._curr_compression = Compression.NONE
        
        self._open(f, mode, allow_unindexed)

        self._output_file = self._file

    def __iter__(self):
        return self.read_messages()

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    @property
    def options(self):
        """Get the options."""
        return { 'compression' : self._compression, 'chunk_threshold' : self._chunk_threshold }
    
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
        """Get the compression method to use for writing."""
        return self._compression
    
    def _set_compression(self, compression):
        """Set the compression method to use for writing."""
        allowed_compressions = [Compression.NONE, Compression.BZ2]
        if compression not in allowed_compressions:
            raise ValueError('compression must be one of: %s' % ', '.join(allowed_compressions))        
        
        self.flush()
        self._compression = compression
        
    compression = property(_get_compression, _set_compression)
    
    # chunk_threshold
    
    def _get_chunk_threshold(self):
        """Get the chunk threshold to use for writing."""
        return self._chunk_threshold
    
    def _set_chunk_threshold(self, chunk_threshold):
        """Set the chunk threshold to use for writing."""
        if chunk_threshold < 0:
            raise ValueError('chunk_threshold must be greater than or equal to zero')

        self.flush()
        self._chunk_threshold = chunk_threshold
        
    chunk_threshold = property(_get_chunk_threshold, _set_chunk_threshold)

    def read_messages(self, topics=None, start_time=None, end_time=None, connection_filter=None, raw=False):
        """
        Read messages from the bag, optionally filtered by topic, timestamp and connection details.
        @param topics: list of topics or a single topic [optional]
        @type  topics: list(str) or str
        @param start_time: earliest timestamp of message to return [optional]
        @type  start_time: U{genpy.Time}
        @param end_time: latest timestamp of message to return [optional]
        @type  end_time: U{genpy.Time}
        @param connection_filter: function to filter connections to include [optional]
        @type  connection_filter: function taking (topic, datatype, md5sum, msg_def, header) and returning bool
        @param raw: if True, then generate tuples of (datatype, (data, md5sum, position), pytype)
        @type  raw: bool
        @return: generator of (topic, message, timestamp) tuples for each message in the bag file
        @rtype:  generator of tuples of (str, U{genpy.Message}, U{genpy.Time}) [not raw] or (str, (str, str, str, tuple, class), U{genpy.Time}) [raw]
        """
        self.flush()

        if topics and type(topics) is str:
            topics = [topics]
        
        return self._reader.read_messages(topics, start_time, end_time, connection_filter, raw)

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
        Write a message to the bag.
        @param topic: name of topic
        @type  topic: str
        @param msg: message to add to bag, or tuple (if raw)
        @type  msg: Message or tuple of raw message data
        @param t: ROS time of message publication, if None specifed, use current time [optional]
        @type  t: U{genpy.Time}
        @param raw: if True, msg is in raw format, i.e. (msg_type, serialized_bytes, md5sum, pytype)
        @type  raw: bool
        @raise ValueError: if arguments are invalid or bag is closed
        """
        if not self._file:
            raise ValueError('I/O operation on closed bag')

        if not topic:
            raise ValueError('topic is invalid')
        if not msg:
            raise ValueError('msg is invalid')

        if t is None:
            t = genpy.Time.from_sec(time.time())

        # Seek to end (in case previous operation was a read)
        self._file.seek(0, os.SEEK_END)

        # Open a chunk, if needed
        if not self._chunk_open:
            self._start_writing_chunk(t)

        # Unpack raw
        if raw:
            if len(msg) == 5:
                msg_type, serialized_bytes, md5sum, pos, pytype = msg
            elif len(msg) == 4:
                msg_type, serialized_bytes, md5sum, pytype = msg
            else:
                raise ValueError('msg must be of length 4 or 5')

        # Write connection record, if necessary (currently using a connection per topic; ignoring message connection header)
        if topic in self._topic_connections:
            connection_info = self._topic_connections[topic]
            conn_id = connection_info.id
        else:
            conn_id = len(self._connections)

            if raw:
                if pytype is None:
                    try:
                        pytype = genpy.message.get_message_class(msg_type)
                    except Exception:
                        pytype = None
                if pytype is None:
                    raise ROSBagException('cannot locate message class and no message class provided for [%s]' % msg_type)
    
                if pytype._md5sum != md5sum:
                    print('WARNING: md5sum of loaded type [%s] does not match that specified' % msg_type, file=sys.stderr)
                    #raise ROSBagException('md5sum of loaded type does not match that of data being recorded')
            
                header = { 'topic' : topic, 'type' : msg_type, 'md5sum' : md5sum, 'message_definition' : pytype._full_text }
            else:
                header = { 'topic' : topic, 'type' : msg.__class__._type, 'md5sum' : msg.__class__._md5sum, 'message_definition' : msg._full_text }

            connection_info = _ConnectionInfo(conn_id, topic, header)

            self._write_connection_record(connection_info)

            self._connections[conn_id] = connection_info
            self._topic_connections[topic] = connection_info

        # Create an index entry
        index_entry = _IndexEntry200(t, self._curr_chunk_info.pos, self._get_chunk_offset())

        # Update the indexes and current chunk info 
        if conn_id not in self._curr_chunk_connection_indexes:
            # This is the first message on this connection in the chunk
            self._curr_chunk_connection_indexes[conn_id] = [index_entry]
            self._curr_chunk_info.connection_counts[conn_id] = 1
        else:
            curr_chunk_connection_index = self._curr_chunk_connection_indexes[conn_id]
            if index_entry >= curr_chunk_connection_index[-1]:
                # Test if we're writing chronologically.  Can skip binary search if so.
                curr_chunk_connection_index.append(index_entry)
            else:
                bisect.insort_right(curr_chunk_connection_index, index_entry)

            self._curr_chunk_info.connection_counts[conn_id] += 1

        if conn_id not in self._connection_indexes:
            self._connection_indexes[conn_id] = [index_entry]
        else:
            bisect.insort_right(self._connection_indexes[conn_id], index_entry)

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
        self._write_message_data_record(conn_id, t, serialized_bytes)
        
        # Check if we want to stop this chunk
        chunk_size = self._get_chunk_offset()
        if chunk_size > self._chunk_threshold:
            self._stop_writing_chunk()

    def reindex(self):
        """
        Reindexes the bag file.  Yields position of each chunk for progress.
        """
        self._clear_index()
        return self._reader.reindex()

    def close(self):
        """
        Close the bag file.  Closing an already closed bag does nothing.
        """
        if self._file:
            if self._mode in 'wa':
                self._stop_writing()
            
            self._close_file()

    def __str__(self):
        rows = []

        try:
            if self._filename:
                rows.append(('path', self._filename))

            if self._version == 102 and type(self._reader) == _BagReader102_Unindexed:
                rows.append(('version', '1.2 (unindexed)'))
            else:
                rows.append(('version', '%d.%d' % (self._version / 100, self._version % 100)))

            if not self._connection_indexes and not self._chunks:
                rows.append(('size', _human_readable_size(self.size)))
            else:
                if self._chunks:
                    start_stamp = self._chunks[ 0].start_time.to_sec()
                    end_stamp   = self._chunks[-1].end_time.to_sec()
                else:
                    start_stamp = min([index[ 0].time.to_sec() for index in self._connection_indexes.itervalues()])
                    end_stamp   = max([index[-1].time.to_sec() for index in self._connection_indexes.itervalues()])
    
                # Show duration
                duration = end_stamp - start_stamp
                dur_secs = duration % 60
                dur_mins = int(duration / 60)
                dur_hrs  = int(dur_mins / 60)
                if dur_hrs > 0:
                    dur_mins = dur_mins % 60
                    duration_str = '%dhr %d:%02ds (%ds)' % (dur_hrs, dur_mins, dur_secs, duration)
                elif dur_mins > 0:
                    duration_str = '%d:%02ds (%ds)' % (dur_mins, dur_secs, duration)
                else:
                    duration_str = '%.1fs' % duration   

                rows.append(('duration', duration_str))
        
                # Show start and end times
                rows.append(('start', '%s (%.2f)' % (_time_to_str(start_stamp), start_stamp)))
                rows.append(('end',   '%s (%.2f)' % (_time_to_str(end_stamp),   end_stamp)))
    
                rows.append(('size', _human_readable_size(self.size)))

                if self._chunks:
                    num_messages = 0
                    for c in self._chunks:
                        for counts in c.connection_counts.itervalues():
                            num_messages += counts
                else:
                    num_messages = sum([len(index) for index in self._connection_indexes.itervalues()])
                rows.append(('messages', str(num_messages)))

                # Show compression information
                if len(self._chunk_headers) == 0:
                    rows.append(('compression', 'none'))
                else:
                    compression_counts       = {}
                    compression_uncompressed = {}
                    compression_compressed   = {}
                    for chunk_header in self._chunk_headers.itervalues():
                        if chunk_header.compression not in compression_counts:
                            compression_counts[chunk_header.compression] = 1
                            compression_uncompressed[chunk_header.compression] = chunk_header.uncompressed_size
                            compression_compressed[chunk_header.compression] = chunk_header.compressed_size
                        else:
                            compression_counts[chunk_header.compression] += 1
                            compression_uncompressed[chunk_header.compression] += chunk_header.uncompressed_size
                            compression_compressed[chunk_header.compression] += chunk_header.compressed_size
    
                    chunk_count = len(self._chunk_headers)
    
                    compressions = []
                    for count, compression in reversed(sorted([(v, k) for k, v in compression_counts.items()])):
                        if compression != Compression.NONE:
                            fraction = (100.0 * compression_compressed[compression]) / compression_uncompressed[compression]
                            compressions.append('%s [%d/%d chunks; %.2f%%]' % (compression, count, chunk_count, fraction))
                        else:
                            compressions.append('%s [%d/%d chunks]' % (compression, count, chunk_count))
                    rows.append(('compression', ', '.join(compressions)))
    
                    all_uncompressed = (sum([count for c, count in compression_counts.items() if c != Compression.NONE]) == 0)
                    if not all_uncompressed:    
                        total_uncompressed_size = sum((h.uncompressed_size for h in self._chunk_headers.itervalues()))
                        total_compressed_size   = sum((h.compressed_size   for h in self._chunk_headers.itervalues()))
                        
                        total_uncompressed_size_str = _human_readable_size(total_uncompressed_size)
                        total_compressed_size_str   = _human_readable_size(total_compressed_size)
                        total_size_str_length = max([len(total_uncompressed_size_str), len(total_compressed_size_str)])

                        if duration > 0:
                            uncompressed_rate_str = _human_readable_size(total_uncompressed_size / duration)
                            compressed_rate_str   = _human_readable_size(total_compressed_size / duration)
                            rate_str_length = max([len(uncompressed_rate_str), len(compressed_rate_str)])

                            rows.append(('uncompressed', '%*s @ %*s/s' %
                                         (total_size_str_length, total_uncompressed_size_str, rate_str_length, uncompressed_rate_str)))
                            rows.append(('compressed',   '%*s @ %*s/s (%.2f%%)' %
                                         (total_size_str_length, total_compressed_size_str,   rate_str_length, compressed_rate_str, (100.0 * total_compressed_size) / total_uncompressed_size)))
                        else:
                            rows.append(('uncompressed', '%*s' % (total_size_str_length, total_uncompressed_size_str)))
                            rows.append(('compressed',   '%*s' % (total_size_str_length, total_compressed_size_str)))

                datatypes = set()
                datatype_infos = []
                for connection in self._connections.itervalues():
                    if connection.datatype in datatypes:
                        continue
                    datatype_infos.append((connection.datatype, connection.md5sum, connection.msg_def))
                    datatypes.add(connection.datatype)
                    
                topics = sorted(set([c.topic for c in self._get_connections()]))
                topic_datatypes    = {}
                topic_conn_counts  = {}
                topic_msg_counts   = {}
                topic_freqs_median = {}
                for topic in topics:
                    connections = list(self._get_connections(topic))

                    topic_datatypes[topic] = connections[0].datatype
                    topic_conn_counts[topic] = len(connections)

                    msg_count = 0
                    for connection in connections:
                        for chunk in self._chunks:
                            msg_count += chunk.connection_counts.get(connection.id, 0)
                    topic_msg_counts[topic] = msg_count

                    if self._connection_indexes_read:
                        stamps = [entry.time.to_sec() for entry in self._get_entries(connections)]
                        if len(stamps) > 1:
                            periods = [s1 - s0 for s1, s0 in zip(stamps[1:], stamps[:-1])]
                            med_period = _median(periods)
                            if med_period > 0.0:
                                topic_freqs_median[topic] = 1.0 / med_period

                topics = sorted(topic_datatypes.iterkeys())
                max_topic_len       = max([len(topic) for topic in topics])
                max_datatype_len    = max([len(datatype) for datatype in datatypes])
                max_msg_count_len   = max([len('%d' % msg_count) for msg_count in topic_msg_counts.itervalues()])
                max_freq_median_len = max([len(_human_readable_frequency(freq)) for freq in topic_freqs_median.itervalues()]) if len(topic_freqs_median) > 0 else 0

                # Show datatypes       
                for i, (datatype, md5sum, msg_def) in enumerate(sorted(datatype_infos)):
                    s = '%-*s [%s]' % (max_datatype_len, datatype, md5sum)
                    if i == 0:
                        rows.append(('types', s))
                    else:
                        rows.append(('', s))
                    
                # Show topics
                for i, topic in enumerate(topics):
                    topic_msg_count = topic_msg_counts[topic]
                    
                    s = '%-*s   %*d %s' % (max_topic_len, topic, max_msg_count_len, topic_msg_count, 'msgs' if topic_msg_count > 1 else 'msg ')
                    if topic in topic_freqs_median:
                        s += ' @ %*s' % (max_freq_median_len, _human_readable_frequency(topic_freqs_median[topic]))
                    else:
                        s += '   %*s' % (max_freq_median_len, '')

                    s += ' : %-*s' % (max_datatype_len, topic_datatypes[topic])
                    if topic_conn_counts[topic] > 1:
                        s += ' (%d connections)' % topic_conn_counts[topic]
        
                    if i == 0:
                        rows.append(('topics', s))
                    else:
                        rows.append(('', s))
        
        except Exception as ex:
            raise

        first_column_width = max([len(field) for field, _ in rows]) + 1

        s = ''
        for (field, value) in rows:
            if field:
                s += '%-*s %s\n' % (first_column_width, field + ':', value)
            else:
                s += '%-*s %s\n' % (first_column_width, '', value)

        return s.rstrip()

    def _get_yaml_info(self, key=None):
        s = ''

        try:
            if self._filename:
                s += 'path: %s\n' % self._filename

            if self._version == 102 and type(self._reader) == _BagReader102_Unindexed:
                s += 'version: 1.2 (unindexed)\n'
            else:
                s += 'version: %d.%d\n' % (self._version / 100, self._version % 100)

            if not self._connection_indexes and not self._chunks:
                s += 'size: %d\n' % self.size
                s += 'indexed: False\n'
            else:
                if self._chunks:
                    start_stamp = self._chunks[ 0].start_time.to_sec()
                    end_stamp   = self._chunks[-1].end_time.to_sec()
                else:
                    start_stamp = min([index[ 0].time.to_sec() for index in self._connection_indexes.itervalues()])
                    end_stamp   = max([index[-1].time.to_sec() for index in self._connection_indexes.itervalues()])
                
                duration = end_stamp - start_stamp
                s += 'duration: %.6f\n' % duration
                s += 'start: %.6f\n' % start_stamp
                s += 'end: %.6f\n' % end_stamp
                s += 'size: %d\n' % self.size
                if self._chunks:
                    num_messages = 0
                    for c in self._chunks:
                        for counts in c.connection_counts.itervalues():
                            num_messages += counts
                else:
                    num_messages = sum([len(index) for index in self._connection_indexes.itervalues()])
                s += 'messages: %d\n' % num_messages
                s += 'indexed: True\n'

                # Show compression information
                if len(self._chunk_headers) == 0:
                    s += 'compression: none\n'
                else:
                    compression_counts       = {}
                    compression_uncompressed = {}
                    compression_compressed   = {}
                    for chunk_header in self._chunk_headers.itervalues():
                        if chunk_header.compression not in compression_counts:
                            compression_counts[chunk_header.compression] = 1
                            compression_uncompressed[chunk_header.compression] = chunk_header.uncompressed_size
                            compression_compressed[chunk_header.compression] = chunk_header.compressed_size
                        else:
                            compression_counts[chunk_header.compression] += 1
                            compression_uncompressed[chunk_header.compression] += chunk_header.uncompressed_size
                            compression_compressed[chunk_header.compression] += chunk_header.compressed_size
    
                    chunk_count = len(self._chunk_headers)
    
                    main_compression_count, main_compression = list(reversed(sorted([(v, k) for k, v in compression_counts.items()])))[0]
                    s += 'compression: %s\n' % str(main_compression)
    
                    all_uncompressed = (sum([count for c, count in compression_counts.items() if c != Compression.NONE]) == 0)
                    if not all_uncompressed:    
                        s += 'uncompressed: %d\n' % sum((h.uncompressed_size for h in self._chunk_headers.itervalues()))
                        s += 'compressed: %d\n' % sum((h.compressed_size for h in self._chunk_headers.itervalues()))

                datatypes = set()
                datatype_infos = []
                for connection in self._connections.itervalues():
                    if connection.datatype in datatypes:
                        continue
                    datatype_infos.append((connection.datatype, connection.md5sum, connection.msg_def))
                    datatypes.add(connection.datatype)
                    
                topics = sorted(set([c.topic for c in self._get_connections()]))
                topic_datatypes    = {}
                topic_conn_counts  = {}
                topic_msg_counts   = {}
                topic_freqs_median = {}
                for topic in topics:
                    connections = list(self._get_connections(topic))

                    topic_datatypes[topic] = connections[0].datatype
                    topic_conn_counts[topic] = len(connections)

                    msg_count = 0
                    for connection in connections:
                        for chunk in self._chunks:
                            msg_count += chunk.connection_counts.get(connection.id, 0)
                    topic_msg_counts[topic] = msg_count

                    if self._connection_indexes_read:
                        stamps = [entry.time.to_sec() for entry in self._get_entries(connections)]
                        if len(stamps) > 1:
                            periods = [s1 - s0 for s1, s0 in zip(stamps[1:], stamps[:-1])]
                            med_period = _median(periods)
                            if med_period > 0.0:
                                topic_freqs_median[topic] = 1.0 / med_period

                topics = sorted(topic_datatypes.iterkeys())
                max_topic_len       = max([len(topic) for topic in topics])
                max_datatype_len    = max([len(datatype) for datatype in datatypes])
                max_msg_count_len   = max([len('%d' % msg_count) for msg_count in topic_msg_counts.itervalues()])
                max_freq_median_len = max([len(_human_readable_frequency(freq)) for freq in topic_freqs_median.itervalues()]) if len(topic_freqs_median) > 0 else 0

                # Show datatypes       
                s += 'types:\n'
                for i, (datatype, md5sum, msg_def) in enumerate(sorted(datatype_infos)):
                    s += '    - type: %s\n' % datatype
                    s += '      md5: %s\n' % md5sum
                    
                # Show topics
                s += 'topics:\n'
                for i, topic in enumerate(topics):
                    topic_msg_count = topic_msg_counts[topic]
                    
                    s += '    - topic: %s\n' % topic
                    s += '      type: %s\n' % topic_datatypes[topic]
                    s += '      messages: %d\n' % topic_msg_count
                        
                    if topic_conn_counts[topic] > 1:
                        s += '      connections: %d\n' % topic_conn_counts[topic]

                    if topic in topic_freqs_median:
                        s += '      frequency: %.4f\n' % topic_freqs_median[topic]

            if not key:
                return s
            
            class DictObject(object):
                def __init__(self, d):
                    for a, b in d.items():
                        if isinstance(b, (list, tuple)):
                           setattr(self, a, [DictObject(x) if isinstance(x, dict) else x for x in b])
                        else:
                           setattr(self, a, DictObject(b) if isinstance(b, dict) else b)

            obj = DictObject(yaml.load(s))
            try:
                val = eval('obj.' + key)
            except Exception as ex:
                print('Error getting key "%s"' % key, file=sys.stderr)
                return None

            def print_yaml(val, indent=0):
                indent_str = '  ' * indent

                if type(val) is list:
                    s = ''
                    for item in val:
                        s += '%s- %s\n' % (indent_str, print_yaml(item, indent + 1))
                    return s
                elif type(val) is DictObject:
                    s = ''
                    for i, (k, v) in enumerate(val.__dict__.items()):
                        if i != 0:
                            s += indent_str
                        s += '%s: %s' % (k, str(v))
                        if i < len(val.__dict__) - 1:
                            s += '\n'
                    return s
                else:
                    return indent_str + str(val)

            return print_yaml(val)

        except Exception as ex:
            raise

    ### Internal API ###

    @property
    def _has_compressed_chunks(self):
        if not self._chunk_headers:
            return False

        return any((h.compression != Compression.NONE for h in self._chunk_headers.itervalues()))

    @property
    def _uncompressed_size(self):
        if not self._chunk_headers:
            return self.size

        return sum((h.uncompressed_size for h in self._chunk_headers.itervalues()))

    def _read_message(self, position, raw=False):
        """
        Read the message from the given position in the file.
        """
        self.flush()
        return self._reader.seek_and_read_message_data_record(position, raw)

    # Index accessing

    def _get_connections(self, topics=None, connection_filter=None):
        """
        Yield the connections, optionally filtering by topic and/or connection information.
        """
        if topics:
            if type(topics) is str:
                topics = set([roslib.names.canonicalize_name(topics)])
            else:
                topics = set([roslib.names.canonicalize_name(t) for t in topics])

        for c in self._connections.itervalues():
            if topics and c.topic not in topics and roslib.names.canonicalize_name(c.topic) not in topics:
                continue
            if connection_filter and not connection_filter(c.topic, c.datatype, c.md5sum, c.msg_def, c.header):
                continue
            yield c

    def _get_entries(self, connections=None, start_time=None, end_time=None):
        """
        Yield index entries on the given connections in the given time range.
        """
        for entry, _ in _mergesort(self._get_indexes(connections), key=lambda entry: entry.time):
            if start_time and entry.time < start_time:
                continue
            if end_time and entry.time > end_time:
                return
            yield entry

    def _get_entries_reverse(self, connections=None, start_time=None, end_time=None):
        """
        Yield index entries on the given connections in the given time range in reverse order.
        """
        for entry, _ in _mergesort((reversed(index) for index in self._get_indexes(connections)), key=lambda entry: -entry.time.to_sec()):
            if end_time and entry.time > end_time:
                continue
            if start_time and entry.time < start_time:
                return
            yield entry

    def _get_entry(self, t, connections=None):
        """
        Return the first index entry on/before the given time on the given connections
        """
        indexes = self._get_indexes(connections)

        entry = _IndexEntry(t)

        first_entry = None

        for index in indexes:
            i = bisect.bisect_right(index, entry) - 1
            if i >= 0:
                index_entry = index[i]
                if first_entry is None or index_entry > first_entry:
                    first_entry = index_entry

        return first_entry
    
    def _get_entry_after(self, t, connections=None):
        """
        Return the first index entry after the given time on the given connections
        """
        indexes = self._get_indexes(connections)

        entry = _IndexEntry(t)

        first_entry = None

        for index in indexes:
            i = bisect.bisect_right(index, entry) 
            if i <= len(index) - 1:
                index_entry = index[i]
                if first_entry is None or index_entry < first_entry:
                    first_entry = index_entry

        return first_entry

    def _get_indexes(self, connections):
        """
        Get the indexes for the given connections.
        """
        if not self._connection_indexes_read:
            self._reader._read_connection_index_records()

        if connections is None:
            return self._connection_indexes.itervalues()
        else:
            return (self._connection_indexes[c.id] for c in connections)

    ### Implementation ###

    def _clear_index(self):
        self._connection_indexes_read = False
        self._connection_indexes      = {}    # id    -> IndexEntry[] (1.2+)

        self._topic_connections  = {}    # topic -> connection_id
        self._connections        = {}    # id -> ConnectionInfo
        self._connection_count   = 0     # (2.0)
        self._chunk_count        = 0     # (2.0)
        self._chunks             = []    # ChunkInfo[] (2.0)
        self._chunk_headers      = {}    # chunk_pos -> ChunkHeader (2.0)

        self._chunk_open                    = False
        self._curr_chunk_info               = None
        self._curr_chunk_data_pos           = None
        self._curr_chunk_connection_indexes = {}
    
    def _open(self, f, mode, allow_unindexed):
        if not f:
            raise ValueError('filename (or stream) is invalid')

        try:
            if   mode == 'r': self._open_read(f, allow_unindexed)
            elif mode == 'w': self._open_write(f)
            elif mode == 'a': self._open_append(f, allow_unindexed)
            else:
                raise ValueError('mode "%s" is invalid' % mode)
        except struct.error:
            raise ROSBagFormatException('error with bag')

    def _open_read(self, f, allow_unindexed):
        if isinstance(f, file):
            self._file     = f
            self._filename = None
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
        except ROSBagUnindexedException as ex:
            if not allow_unindexed:
                self._close_file()
                raise
        except:
            self._close_file()
            raise

    def _open_write(self, f):
        if isinstance(f, file):
            self._file     = f
            self._filename = None
        else:
            self._file     = open(f, 'w+b')
            self._filename = f

        self._mode = 'w'

        self._version = 200

        try:
            self._create_reader()
            self._start_writing()
        except:
            self._close_file()
            raise

    def _open_append(self, f, allow_unindexed):
        if isinstance(f, file):
            self._file     = f
            self._filename = None
        else:        
            try:
                # Test if the file already exists
                open(f, 'r').close()

                # File exists: open in read with update mode
                self._file = open(f, 'r+b')
            except IOError:
                # File doesn't exist: open in write mode
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
        except ROSBagUnindexedException:
            if not allow_unindexed:
                self._close_file()
                raise
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
        major_version, minor_version = self._version / 100, self._version % 100
        if major_version == 2:
            self._reader = _BagReader200(self)
        elif major_version == 1:
            if minor_version == 1:
                raise ROSBagException('unsupported bag version %d. Please convert bag to newer format.' % self._version)
            else:
                # Get the op code of the first record.  If it's FILE_HEADER, then we have an indexed 1.2 bag.
                first_record_pos = self._file.tell()
                header = _read_header(self._file)
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
            raise ROSBagException('This does not appear to be a bag file')
        
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

        # Write connection infos
        for connection_info in self._connections.itervalues():
            self._write_connection_record(connection_info)

        # Write chunk infos
        for chunk_info in self._chunks:
            self._write_chunk_info_record(chunk_info)

        # Re-write the file header
        self._file.seek(self._file_header_pos)
        self._write_file_header_record(self._index_data_pos, len(self._connections), len(self._chunks))

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
        self._chunks.append(self._curr_chunk_info)

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

        # Write out the connection indexes and clear them
        self._file.seek(end_of_chunk_pos)
        for connection_id, entries in self._curr_chunk_connection_indexes.items():
            self._write_connection_index_record(connection_id, entries)
        self._curr_chunk_connection_indexes.clear()

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

    def _write_file_header_record(self, index_pos, connection_count, chunk_count):
        header = {
            'op':          _pack_uint8(_OP_FILE_HEADER),
            'index_pos':   _pack_uint64(index_pos),
            'conn_count':  _pack_uint32(connection_count),
            'chunk_count': _pack_uint32(chunk_count)
        }
        _write_record(self._file, header, padded_size=_FILE_HEADER_LENGTH)

    def _write_connection_record(self, connection_info):
        header = {
            'op':    _pack_uint8(_OP_CONNECTION),
            'topic': connection_info.topic,
            'conn':  _pack_uint32(connection_info.id)
        }
        
        _write_header(self._output_file, header)
        
        _write_header(self._output_file, connection_info.header)

    def _write_message_data_record(self, connection_id, t, serialized_bytes):
        header = {
            'op':   _pack_uint8(_OP_MSG_DATA),
            'conn': _pack_uint32(connection_id),
            'time': _pack_time(t)
        }
        _write_record(self._output_file, header, serialized_bytes)

    def _write_chunk_header(self, chunk_header):
        header = {
            'op':          _pack_uint8(_OP_CHUNK),
            'compression': chunk_header.compression,
            'size':        _pack_uint32(chunk_header.uncompressed_size)
        }
        _write_header(self._file, header)

        self._file.write(_pack_uint32(chunk_header.compressed_size))

    def _write_connection_index_record(self, connection_id, entries):        
        header = {
            'op':    _pack_uint8(_OP_INDEX_DATA),
            'conn':  _pack_uint32(connection_id),
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
            'count':      _pack_uint32(len(chunk_info.connection_counts))
        }
        
        buffer = self._buffer
        buffer.seek(0)
        buffer.truncate(0)
        for connection_id, count in chunk_info.connection_counts.items():
            buffer.write(_pack_uint32(connection_id))
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
_OP_CONNECTION  = 0x07

_OP_CODES = {
    _OP_MSG_DEF:     'MSG_DEF',
    _OP_MSG_DATA:    'MSG_DATA',
    _OP_FILE_HEADER: 'FILE_HEADER',
    _OP_INDEX_DATA:  'INDEX_DATA',
    _OP_CHUNK:       'CHUNK',
    _OP_CHUNK_INFO:  'CHUNK_INFO',
    _OP_CONNECTION:  'CONNECTION'
}

_VERSION             = '#ROSBAG V2.0'
_FILE_HEADER_LENGTH  = 4096
_INDEX_VERSION       = 1
_CHUNK_INDEX_VERSION = 1

class _ConnectionInfo(object):
    def __init__(self, id, topic, header):
        try:
            datatype, md5sum, msg_def = header['type'], header['md5sum'], header['message_definition']
        except KeyError as ex:
            raise ROSBagFormatException('connection header field %s not found' % str(ex))

        self.id       = id
        self.topic    = topic
        self.datatype = datatype
        self.md5sum   = md5sum
        self.msg_def  = msg_def
        self.header   = header

    def __str__(self):
        return '%d on %s: %s' % (self.id, self.topic, str(self.header))

class _ChunkInfo(object):
    def __init__(self, pos, start_time, end_time):
        self.pos        = pos
        self.start_time = start_time
        self.end_time   = end_time
        
        self.connection_counts = {}

    def __str__(self):
        s  = 'chunk_pos:   %d\n' % self.pos
        s += 'start_time:  %s\n' % str(self.start_time)
        s += 'end_time:    %s\n' % str(self.end_time)
        s += 'connections: %d\n' % len(self.connection_counts)
        s += '\n'.join(['  - [%4d] %d' % (connection_id, count) for connection_id, count in self.connection_counts.items()])
        return s

class _ChunkHeader(object):
    def __init__(self, compression, compressed_size, uncompressed_size, data_pos=0):
        self.compression       = compression
        self.compressed_size   = compressed_size
        self.uncompressed_size = uncompressed_size
        self.data_pos          = data_pos

    def __str__(self):
        if self.uncompressed_size > 0:
            ratio = 100 * (float(self.compressed_size) / self.uncompressed_size)
            return 'compression: %s, size: %d, uncompressed: %d (%.2f%%)' % (self.compression, self.compressed_size, self.uncompressed_size, ratio)
        else:
            return 'compression: %s, size: %d, uncompressed: %d' % (self.compression, self.compressed_size, self.uncompressed_size)

class _IndexEntry(object):
    def __init__(self, time):
        self.time = time

    def __cmp__(self, other):
        return self.time.__cmp__(other.time)

class _IndexEntry102(_IndexEntry):
    def __init__(self, time, offset):
        self.time   = time
        self.offset = offset
        
    @property
    def position(self):
        return self.offset
        
    def __str__(self):
        return '%d.%d: %d' % (self.time.secs, self.time.nsecs, self.offset)

class _IndexEntry200(_IndexEntry):
    def __init__(self, time, chunk_pos, offset):
        self.time      = time
        self.chunk_pos = chunk_pos
        self.offset    = offset

    @property
    def position(self):
        return (self.chunk_pos, self.offset)

    def __str__(self):
        return '%d.%d: %d+%d' % (self.time.secs, self.time.nsecs, self.chunk_pos, self.offset)
    
def _get_message_type(info):
    message_type = _message_types.get(info.md5sum)
    if message_type is None:
        try:
            message_type = genpy.dynamic.generate_dynamic(info.datatype, info.msg_def)[info.datatype]
            if (message_type._md5sum != info.md5sum):
                print('WARNING: For type [%s] stored md5sum [%s] does not match message definition [%s].\n  Try: "rosrun rosbag fix_msg_defs.py old_bag new_bag."'%(info.datatype, info.md5sum, message_type._md5sum), file=sys.stderr)
        except genmsg.InvalidMsgSpec:
            message_type = genpy.dynamic.generate_dynamic(info.datatype, "")[info.datatype]
            print('WARNING: For type [%s] stored md5sum [%s] has invalid message definition."'%(info.datatype, info.md5sum), file=sys.stderr)
        except genmsg.MsgGenerationException as ex:
            raise ROSBagException('Error generating datatype %s: %s' % (info.datatype, str(ex)))

        _message_types[info.md5sum] = message_type

    return message_type

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

def _skip_record(f):
    _skip_sized(f)  # skip header
    _skip_sized(f)  # skip data

def _skip_sized(f):
    size = _read_uint32(f)
    f.seek(size, os.SEEK_CUR)

def _read_sized(f):
    try:
        size = _read_uint32(f)
    except struct.error as ex:
        raise ROSBagFormatException('error unpacking uint32: %s' % str(ex))
    return _read(f, size)

def _write_sized(f, v):
    f.write(_pack_uint32(len(v)))
    f.write(v)

def _read_field(header, field, unpack_fn):
    if field not in header:
        raise ROSBagFormatException('expected "%s" field in record' % field)
    
    try:
        value = unpack_fn(header[field])
    except Exception as ex:
        raise ROSBagFormatException('error reading field "%s": %s' % (field, str(ex)))
    
    return value

def _read_str_field   (header, field): return _read_field(header, field, lambda v: v)
def _read_uint8_field (header, field): return _read_field(header, field, _unpack_uint8)
def _read_uint32_field(header, field): return _read_field(header, field, _unpack_uint32)
def _read_uint64_field(header, field): return _read_field(header, field, _unpack_uint64)
def _read_time_field  (header, field): return _read_field(header, field, _unpack_time)

def _write_record(f, header, data='', padded_size=None):
    header_str = _write_header(f, header)

    if padded_size is not None:
        header_len = len(header_str)
        if header_len < padded_size:
            data = ' ' * (padded_size - header_len)
        else:
            data = ''

    _write_sized(f, data)

def _write_header(f, header):
    header_str = ''.join([_pack_uint32(len(k) + 1 + len(v)) + k + '=' + v for k, v in header.items()])
    _write_sized(f, header_str)
    return header_str

def _read_header(f, req_op=None):
    bag_pos = f.tell()

    # Read header
    try:
        header = _read_sized(f)
    except ROSBagException as ex:
        raise ROSBagFormatException('Error reading header: %s' % str(ex))

    # Parse header into a dict
    header_dict = {}
    while header != '':
        # Read size
        if len(header) < 4:
            raise ROSBagFormatException('Error reading header field')           
        (size,) = struct.unpack('<L', header[:4])                          # @todo reindex: catch struct.error
        header = header[4:]

        # Read bytes
        if len(header) < size:
            raise ROSBagFormatException('Error reading header field: expected %d bytes, read %d' % (size, len(header)))
        (name, sep, value) = header[:size].partition('=')
        if sep == '':
            raise ROSBagFormatException('Error reading header field')

        header_dict[name] = value                                          # @todo reindex: raise exception on empty name
        
        header = header[size:]

    # Check the op code of the header, if supplied
    if req_op is not None:
        op = _read_uint8_field(header_dict, 'op')
        if req_op != op:
            raise ROSBagFormatException('Expected op code: %s, got %s' % (_OP_CODES[req_op], _OP_CODES[op]))

    return header_dict

def _peek_next_header_op(f):
    pos = f.tell()
    header = _read_header(f)
    op = _read_uint8_field(header, 'op')
    f.seek(pos)
    return op

def _read_record_data(f):
    try:
        record_data = _read_sized(f)
    except ROSBagException as ex:
        raise ROSBagFormatException('Error reading record data: %s' % str(ex))

    return record_data

class _BagReader(object):
    def __init__(self, bag):
        self.bag = bag
        
    def start_reading(self):
        raise NotImplementedError()

    def read_messages(self, topics, start_time, end_time, connection_filter, raw):
        raise NotImplementedError()

    def reindex(self):
        raise NotImplementedError()

class _BagReader102_Unindexed(_BagReader):
    """
    Support class for reading unindexed v1.2 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)
        
    def start_reading(self):
        self.bag._file_header_pos = self.bag._file.tell()

    def reindex(self):
        """Generates all bag index information by rereading the message records."""
        f = self.bag._file
        
        total_bytes = self.bag.size
        
        # Re-read the file header to get to the start of the first message
        self.bag._file.seek(self.bag._file_header_pos)

        offset = f.tell()

        # Read message definition and data records
        while offset < total_bytes:
            yield offset
            
            op = _peek_next_header_op(f)

            if op == _OP_MSG_DEF:
                connection_info = self.read_message_definition_record()
    
                if connection_info.topic not in self.bag._topic_connections:
                    self.bag._topic_connections[connection_info.topic] = connection_info.id
                    self.bag._connections[connection_info.id]          = connection_info
                    self.bag._connection_indexes[connection_info.id]   = []

            elif op == _OP_MSG_DATA:
                # Read the topic and timestamp from the header
                header = _read_header(f)
                
                topic = _read_str_field(header, 'topic')
                secs  = _read_uint32_field(header, 'sec')
                nsecs = _read_uint32_field(header, 'nsec')
                t = genpy.Time(secs, nsecs)

                if topic not in self.bag._topic_connections:
                    datatype = _read_str_field(header, 'type')
                    self._create_connection_info_for_datatype(topic, datatype)

                connection_id = self.bag._topic_connections[topic]
                info = self.bag._connections[connection_id]

                # Skip over the message content
                _skip_sized(f)

                # Insert the message entry (in order) into the connection index
                bisect.insort_right(self.bag._connection_indexes[connection_id], _IndexEntry102(t, offset))
            
            offset = f.tell()

    def read_messages(self, topics, start_time, end_time, topic_filter, raw):
        f = self.bag._file

        f.seek(self.bag._file_header_pos)

        while True:
            # Read MSG_DEF records
            while True:
                position = f.tell()
                
                try:
                    header = _read_header(f)
                except Exception:
                    return

                op = _read_uint8_field(header, 'op')
                if op != _OP_MSG_DEF:
                    break

                connection_info = self.read_message_definition_record(header)
                
                if connection_info.topic not in self.bag._topic_connections:
                    self.bag._topic_connections[connection_info.topic] = connection_info.id

                self.bag._connections[connection_info.id] = connection_info

            # Check that we have a MSG_DATA record
            if op != _OP_MSG_DATA:
                raise ROSBagFormatException('Expecting OP_MSG_DATA, got %d' % op)

            topic = _read_str_field(header, 'topic')
            
            if topic not in self.bag._topic_connections:
                datatype = _read_str_field(header, 'type')
                self._create_connection_info_for_datatype(topic, datatype)

            connection_id = self.bag._topic_connections[topic]
            info = self.bag._connections[connection_id]
    
            # Get the message type
            try:
                msg_type = _get_message_type(info)
            except KeyError:
                raise ROSBagException('Cannot deserialize messages of type [%s].  Message was not preceeded in bagfile by definition' % info.datatype)

            # Get the timestamp
            secs  = _read_uint32_field(header, 'sec')
            nsecs = _read_uint32_field(header, 'nsec')
            t = genpy.Time(secs, nsecs)

            # Read the message content
            data = _read_record_data(f)
            
            if raw:
                msg = (info.datatype, data, info.md5sum, position, msg_type)
            else:
                # Deserialize the message
                msg = msg_type()
                msg.deserialize(data)

            yield (topic, msg, t)

        self.bag._connection_indexes_read = True

    def _create_connection_info_for_datatype(self, topic, datatype):
        for c in self.bag._connections.itervalues():
            if c.datatype == datatype:
                connection_id     = len(self.bag._connections)
                connection_header = { 'topic' : topic, 'type' : c.header['type'], 'md5sum' : c.header['md5sum'], 'message_definition' : c.header['message_definition'] }
                connection_info   = _ConnectionInfo(connection_id, topic, connection_header)

                self.bag._topic_connections[topic]          = connection_id
                self.bag._connections[connection_id]        = connection_info
                self.bag._connection_indexes[connection_id] = []
                return

        raise ROSBagFormatException('Topic %s of datatype %s not preceded by message definition' % (topic, datatype))

    def read_message_definition_record(self, header=None):
        if not header:
            header = _read_header(self.bag._file, _OP_MSG_DEF)

        topic    = _read_str_field(header, 'topic')
        datatype = _read_str_field(header, 'type')
        md5sum   = _read_str_field(header, 'md5')
        msg_def  = _read_str_field(header, 'def')

        _skip_sized(self.bag._file)  # skip the record data

        connection_header = { 'topic' : topic, 'type' : datatype, 'md5sum' : md5sum, 'message_definition' : msg_def }

        id = len(self.bag._connections)

        return _ConnectionInfo(id, topic, connection_header)

class _BagReader102_Indexed(_BagReader102_Unindexed):
    """
    Support class for reading indexed v1.2 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)

    def read_messages(self, topics, start_time, end_time, connection_filter, raw):
        connections = self.bag._get_connections(topics, connection_filter)
        for entry in self.bag._get_entries(connections, start_time, end_time):
            yield self.seek_and_read_message_data_record(entry.offset, raw)

    def reindex(self):
        """Generates all bag index information by rereading the message records."""
        f = self.bag._file
        
        total_bytes = self.bag.size
        
        # Re-read the file header to get to the start of the first message
        self.bag._file.seek(self.bag._file_header_pos)
        self.read_file_header_record()

        offset = f.tell()

        # Read message definition and data records
        while offset < total_bytes:
            yield offset
            
            op = _peek_next_header_op(f)

            if op == _OP_MSG_DEF:
                connection_info = self.read_message_definition_record()
    
                if connection_info.topic not in self.bag._topic_connections:
                    self.bag._topic_connections[connection_info.topic] = connection_info.id
                    self.bag._connections[connection_info.id] = connection_info
                    self.bag._connection_indexes[connection_info.id] = []

            elif op == _OP_MSG_DATA:
                # Read the topic and timestamp from the header
                header = _read_header(f)
                
                topic = _read_str_field(header, 'topic')
                secs  = _read_uint32_field(header, 'sec')
                nsecs = _read_uint32_field(header, 'nsec')
                t = genpy.Time(secs, nsecs)

                if topic not in self.bag._topic_connections:
                    datatype = _read_str_field(header, 'type')
                    self._create_connection_info_for_datatype(topic, datatype)

                connection_id = self.bag._topic_connections[topic]
                info = self.bag._connections[connection_id]

                # Skip over the message content
                _skip_sized(f)

                # Insert the message entry (in order) into the connection index
                bisect.insort_right(self.bag._connection_indexes[connection_id], _IndexEntry102(t, offset))

            elif op == _OP_INDEX_DATA:
                _skip_record(f)

            offset = f.tell()

    def start_reading(self):
        try:
            # Read the file header
            self.read_file_header_record()
            
            total_bytes = self.bag.size
    
            # Check if the index position has been written, i.e. the bag was closed successfully
            if self.bag._index_data_pos == 0:
                raise ROSBagUnindexedException()
    
            # Seek to the beginning of the topic index records
            self.bag._file.seek(self.bag._index_data_pos)

            # Read the topic indexes
            topic_indexes = {}
            while True:
                pos = self.bag._file.tell()
                if pos >= total_bytes:
                    break

                topic, index = self.read_topic_index_record()

                topic_indexes[topic] = index

            # Read the message definition records (one for each topic)
            for topic, index in topic_indexes.items():
                self.bag._file.seek(index[0].offset)
    
                connection_info = self.read_message_definition_record()
    
                if connection_info.topic not in self.bag._topic_connections:
                    self.bag._topic_connections[connection_info.topic] = connection_info.id
                self.bag._connections[connection_info.id] = connection_info
    
                self.bag._connection_indexes[connection_info.id] = index

            self.bag._connection_indexes_read = True

        except Exception as ex:
            raise ROSBagUnindexedException()

    def read_file_header_record(self):
        self.bag._file_header_pos = self.bag._file.tell()

        header = _read_header(self.bag._file, _OP_FILE_HEADER)

        self.bag._index_data_pos = _read_uint64_field(header, 'index_pos')

        _skip_sized(self.bag._file)  # skip the record data, i.e. padding

    def read_topic_index_record(self):
        f = self.bag._file

        header = _read_header(f, _OP_INDEX_DATA)

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

    def seek_and_read_message_data_record(self, position, raw):
        f = self.bag._file

        # Seek to the message position
        f.seek(position)

        # Skip any MSG_DEF records
        while True:
            header = _read_header(f)
            op = _read_uint8_field(header, 'op')
            if op != _OP_MSG_DEF:
                break
            _skip_sized(f)

        # Check that we have a MSG_DATA record
        if op != _OP_MSG_DATA:
            raise ROSBagFormatException('Expecting OP_MSG_DATA, got %d' % op)
        
        topic = _read_str_field(header, 'topic')

        connection_id = self.bag._topic_connections[topic]
        info = self.bag._connections[connection_id]

        # Get the message type
        try:
            msg_type = _get_message_type(info)
        except KeyError:
            raise ROSBagException('Cannot deserialize messages of type [%s].  Message was not preceeded in bagfile by definition' % info.datatype)

        # Get the timestamp
        secs  = _read_uint32_field(header, 'sec')
        nsecs = _read_uint32_field(header, 'nsec')
        t = genpy.Time(secs, nsecs)

        # Read the message content
        data = _read_record_data(f)
        
        if raw:
            msg = info.datatype, data, info.md5sum, position, msg_type
        else:
            # Deserialize the message
            msg = msg_type()
            msg.deserialize(data)
        
        return (topic, msg, t)

class _BagReader200(_BagReader):
    """
    Support class for reading v2.0 bag files.
    """
    def __init__(self, bag):
        _BagReader.__init__(self, bag)
        
        self.decompressed_chunk_pos = None
        self.decompressed_chunk     = None
        self.decompressed_chunk_io  = None

    def reindex(self):
        """
        Generates all bag index information by rereading the chunks.
        Assumes the file header has been read.
        """
        f = self.bag._file

        f.seek(0, os.SEEK_END)
        total_bytes = f.tell()

        # Read any connection records from after the chunk section.
        # This is to workaround a bug in rosbag record --split (fixed in r10390)
        # where connection records weren't always being written inside the chunk.
        self._read_terminal_connection_records()

        # Re-read the file header to get to the start of the first chunk
        self.bag._file.seek(self.bag._file_header_pos)
        self.read_file_header_record()

        trunc_pos = None

        while True:
            chunk_pos = f.tell()
            if chunk_pos >= total_bytes:
                break
            
            yield chunk_pos

            try:
                self._reindex_read_chunk(f, chunk_pos, total_bytes)
            except Exception as ex:
                break
            
            trunc_pos = f.tell()

        if trunc_pos and trunc_pos < total_bytes:
            f.truncate(trunc_pos)

    def _reindex_read_chunk(self, f, chunk_pos, total_bytes):
        # Read the chunk header
        chunk_header = self.read_chunk_header()

        # If the chunk header size is 0, then the chunk wasn't correctly terminated - we're done
        if chunk_header.compressed_size == 0:
            raise ROSBagException('unterminated chunk at %d' % chunk_pos)

        if chunk_header.compression == Compression.NONE:
            chunk_file = f
        else:
            # Read the compressed chunk
            compressed_chunk = _read(f, chunk_header.compressed_size)

            # Decompress it
            if chunk_header.compression == Compression.BZ2:
                self.decompressed_chunk = bz2.decompress(compressed_chunk)
            else:
                raise ROSBagException('unsupported compression type: %s' % chunk_header.compression)

            if self.decompressed_chunk_io:
                self.decompressed_chunk_io.close()
            self.decompressed_chunk_io = StringIO(self.decompressed_chunk)

            chunk_file = self.decompressed_chunk_io

        # Read chunk connection and message records
        self.bag._curr_chunk_info = None

        if chunk_header.compression == Compression.NONE:
            offset = chunk_file.tell() - chunk_pos
        else:
            offset = chunk_file.tell()

        expected_index_length = 0

        while offset < chunk_header.uncompressed_size:
            op = _peek_next_header_op(chunk_file)

            if op == _OP_CONNECTION:
                connection_info = self.read_connection_record(chunk_file)

                if connection_info.id not in self.bag._connections:
                    self.bag._connections[connection_info.id] = connection_info
                if connection_info.id not in self.bag._connection_indexes:
                    self.bag._connection_indexes[connection_info.id] = []

            elif op == _OP_MSG_DATA:
                # Read the connection id and timestamp from the header
                header = _read_header(chunk_file)

                connection_id = _read_uint32_field(header, 'conn')
                t             = _read_time_field  (header, 'time')

                # Update the chunk info with this timestamp
                if not self.bag._curr_chunk_info:
                    self.bag._curr_chunk_info = _ChunkInfo(chunk_pos, t, t)
                else:
                    if t > self.bag._curr_chunk_info.end_time:
                        self.bag._curr_chunk_info.end_time = t
                    elif t < self.bag._curr_chunk_info.start_time:
                        self.bag._curr_chunk_info.start_time = t
                if connection_id in self.bag._curr_chunk_info.connection_counts:
                    self.bag._curr_chunk_info.connection_counts[connection_id] += 1
                else:
                    self.bag._curr_chunk_info.connection_counts[connection_id] = 1

                # Skip over the message content
                _skip_sized(chunk_file)

                # Insert the message entry (in order) into the connection index
                if connection_id not in self.bag._connection_indexes:
                    raise ROSBagException('connection id (id=%d) in chunk at position %d not preceded by connection record' % (connection_id, chunk_pos))
                bisect.insort_right(self.bag._connection_indexes[connection_id], _IndexEntry200(t, chunk_pos, offset)) 

                expected_index_length += 1

            else:
                # Unknown record type so skip
                _skip_record(chunk_file)

            if chunk_header.compression == Compression.NONE:
                offset = chunk_file.tell() - chunk_pos
            else:
                offset = chunk_file.tell()

        # Skip over index records, connection records and chunk info records
        next_op = _peek_next_header_op(f)
        
        total_index_length = 0
        
        while next_op != _OP_CHUNK:
            if next_op == _OP_INDEX_DATA:
                # Bug workaround: C Turtle bags (pre-1.1.15) were written with an incorrect data length
                _, index = self.read_connection_index_record()
                total_index_length += len(index)
            else:
                _skip_record(f)

            if f.tell() >= total_bytes:
                if total_index_length != expected_index_length:
                    raise ROSBagException('index shorter than expected (%d vs %d)' % (total_index_length, expected_index_length))
                break

            next_op = _peek_next_header_op(f)

        # Chunk was read correctly - store info
        self.bag._chunk_headers[chunk_pos] = chunk_header
        self.bag._chunks.append(self.bag._curr_chunk_info)

    def _read_terminal_connection_records(self):
        b, f, r = self.bag, self.bag._file, self.bag._reader

        # Seek to the first record after FILE_HEADER
        f.seek(b._file_header_pos)
        r.read_file_header_record()

        # Advance to the first CONNECTION
        if self._advance_to_next_record(_OP_CONNECTION):
            # Read the CONNECTION records
            while True:
                connection_info = r.read_connection_record(f)

                b._connections[connection_info.id] = connection_info
                b._connection_indexes[connection_info.id] = []

                next_op = _peek_next_header_op(f)
                if next_op != _OP_CONNECTION:
                    break

    def _advance_to_next_record(self, op):
        b, f = self.bag, self.bag._file

        try:
            while True:
                next_op = _peek_next_header_op(f)
                if next_op == op:
                    break

                if next_op == _OP_INDEX_DATA:
                    # Workaround the possible invalid data length in INDEX_DATA records

                    # read_connection_index_record() requires _curr_chunk_info is set
                    if b._curr_chunk_info is None:
                        b._curr_chunk_info = _ChunkInfo(0, rospy.Time(0, 1), rospy.Time(0, 1))

                    b._reader.read_connection_index_record()
                else:
                    _skip_record(f)

            return True

        except Exception as ex:
            return False

    def start_reading(self):
        try:
            # Read the file header
            self.read_file_header_record()
    
            # Check if the index position has been written, i.e. the bag was closed successfully
            if self.bag._index_data_pos == 0:
                raise ROSBagUnindexedException()
    
            # Seek to the end of the chunks
            self.bag._file.seek(self.bag._index_data_pos)

            # Read the connection records
            self.bag._connection_indexes = {}
            for i in range(self.bag._connection_count):
                connection_info = self.read_connection_record(self.bag._file)
                self.bag._connections[connection_info.id] = connection_info
                self.bag._connection_indexes[connection_info.id] = []

            # Read the chunk info records
            self.bag._chunks = [self.read_chunk_info_record() for i in range(self.bag._chunk_count)]
    
            # Read the chunk headers
            self.bag._chunk_headers = {}
            for chunk_info in self.bag._chunks:
                self.bag._file.seek(chunk_info.pos)
                self.bag._chunk_headers[chunk_info.pos] = self.read_chunk_header()

            if not self.bag._skip_index:
                self._read_connection_index_records()

        except Exception as ex:
            raise ROSBagUnindexedException()

    def _read_connection_index_records(self):
        for chunk_info in self.bag._chunks:
            self.bag._file.seek(chunk_info.pos)
            _skip_record(self.bag._file)

            self.bag._curr_chunk_info = chunk_info
            for i in range(len(chunk_info.connection_counts)):
                connection_id, index = self.read_connection_index_record()
                self.bag._connection_indexes[connection_id].extend(index)

        # Remove any connections with no entries
        # This is a workaround for a bug where connection records were being written for
        # connections which had no messages in the bag
        orphan_connection_ids = [id for id, index in self.bag._connection_indexes.iteritems() if not index]
        for id in orphan_connection_ids:
            del self.bag._connections[id]
            del self.bag._connection_indexes[id]

        self.bag._connection_indexes_read = True

    def read_messages(self, topics, start_time, end_time, connection_filter, raw):
        connections = self.bag._get_connections(topics, connection_filter)
        for entry in self.bag._get_entries(connections, start_time, end_time):
            yield self.seek_and_read_message_data_record((entry.chunk_pos, entry.offset), raw)

    ###

    def read_file_header_record(self):
        self.bag._file_header_pos = self.bag._file.tell()

        header = _read_header(self.bag._file, _OP_FILE_HEADER)

        self.bag._index_data_pos   = _read_uint64_field(header, 'index_pos')
        self.bag._chunk_count      = _read_uint32_field(header, 'chunk_count')
        self.bag._connection_count = _read_uint32_field(header, 'conn_count')

        _skip_sized(self.bag._file)  # skip over the record data, i.e. padding

    def read_connection_record(self, f):
        header = _read_header(f, _OP_CONNECTION)

        conn_id = _read_uint32_field(header, 'conn')
        topic   = _read_str_field   (header, 'topic')

        connection_header = _read_header(f)

        return _ConnectionInfo(conn_id, topic, connection_header)

    def read_chunk_info_record(self):
        f = self.bag._file
        
        header = _read_header(f, _OP_CHUNK_INFO)

        chunk_info_version = _read_uint32_field(header, 'ver')
        
        if chunk_info_version == 1:
            chunk_pos        = _read_uint64_field(header, 'chunk_pos')
            start_time       = _read_time_field  (header, 'start_time')
            end_time         = _read_time_field  (header, 'end_time')
            connection_count = _read_uint32_field(header, 'count') 

            chunk_info = _ChunkInfo(chunk_pos, start_time, end_time)

            _read_uint32(f)   # skip the record data size

            for i in range(connection_count):
                connection_id = _read_uint32(f)
                count         = _read_uint32(f)
    
                chunk_info.connection_counts[connection_id] = count
                
            return chunk_info
        else:
            raise ROSBagFormatException('Unknown chunk info record version: %d' % chunk_info_version)

    def read_chunk_header(self):
        header = _read_header(self.bag._file, _OP_CHUNK)

        compression       = _read_str_field   (header, 'compression')
        uncompressed_size = _read_uint32_field(header, 'size')

        compressed_size = _read_uint32(self.bag._file)  # read the record data size
        
        data_pos = self.bag._file.tell()

        return _ChunkHeader(compression, compressed_size, uncompressed_size, data_pos)

    def read_connection_index_record(self):
        f = self.bag._file

        header = _read_header(f, _OP_INDEX_DATA)
        
        index_version = _read_uint32_field(header, 'ver')
        connection_id = _read_uint32_field(header, 'conn')
        count         = _read_uint32_field(header, 'count')
        
        if index_version != 1:
            raise ROSBagFormatException('expecting index version 1, got %d' % index_version)
    
        record_size = _read_uint32(f) # skip the record data size

        index = []
                
        for i in range(count):
            time   = _read_time  (f)
            offset = _read_uint32(f)
            
            index.append(_IndexEntry200(time, self.bag._curr_chunk_info.pos, offset))

        return (connection_id, index)

    def seek_and_read_message_data_record(self, position, raw):
        chunk_pos, offset = position

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

        # Skip any CONNECTION records
        while True:
            header = _read_header(f)
            op = _read_uint8_field(header, 'op')
            if op != _OP_CONNECTION:
                break
            _skip_sized(f)

        # Check that we have a MSG_DATA record
        if op != _OP_MSG_DATA:
            raise ROSBagFormatException('Expecting OP_MSG_DATA, got %d' % op)

        connection_id = _read_uint32_field(header, 'conn')
        t             = _read_time_field  (header, 'time')

        # Get the message type
        connection_info = self.bag._connections[connection_id]
        try:
            msg_type = _get_message_type(connection_info)
        except KeyError:
            raise ROSBagException('Cannot deserialize messages of type [%s].  Message was not preceded in bag by definition' % connection_info.datatype)

        # Read the message content
        data = _read_record_data(f)
        
        # Deserialize the message
        if raw:
            msg = connection_info.datatype, data, connection_info.md5sum, (chunk_pos, offset), msg_type
        else:
            msg = msg_type()
            msg.deserialize(data)
        
        return (connection_info.topic, msg, t)

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

    return '-'

def _human_readable_frequency(freq):
    multiple = 1000.0
    for suffix in ['Hz', 'kHz', 'MHz', 'GHz', 'THz', 'PHz', 'EHz', 'ZHz', 'YHz']:
        if freq < multiple:
            return '%.1f %s' % (freq, suffix)
        freq /= multiple

    return '-'

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

def _median(values):
    values_len = len(values)
    if values_len == 0:
        return float('nan')

    sorted_values = sorted(values)
    if values_len % 2 == 1:
        return sorted_values[(values_len + 1) / 2 - 1]

    lower = sorted_values[values_len / 2 - 1]
    upper = sorted_values[values_len / 2]

    return float(lower + upper) / 2
