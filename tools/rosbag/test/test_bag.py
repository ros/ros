#!/usr/bin/env python
#
# test_bag.py

from __future__ import with_statement

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)

import heapq
import os
import shutil
import sys
import time
import unittest

import rosbag
from rosbag import bag
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import ColorRGBA

class TestRosbag(unittest.TestCase):
    def setUp(self):
        pass
    
    def test_opening_stream_works(self):
        f = open('/tmp/test_opening_stream_works.bag', 'w')
        b = rosbag.Bag(f, 'w')
        for i in range(10):
            msg = Int32()
            msg.data = i
            b.write('/int', msg)
        b.close()
        
        f = open('/tmp/test_opening_stream_works.bag', 'r')
        b = rosbag.Bag(f, 'r')
        self.assert_(len(list(b.read_messages())) == 10)
        b.close()

    def test_invalid_bag_arguments_fails(self):
        f = '/tmp/test_invalid_bad_arguments_fails.bag'
        
        def fn1(): rosbag.Bag('')
        def fn2(): rosbag.Bag(None)
        def fn3(): rosbag.Bag(f, 'z')        
        def fn4(): rosbag.Bag(f, 'r', compression='foobar')
        def fn5(): rosbag.Bag(f, 'r', chunk_threshold=-1000)
        for fn in [fn1, fn2, fn3, fn4, fn5]:
            self.failUnlessRaises(ValueError, fn)

    def test_io_on_close_fails(self):
        def fn():
            b = rosbag.Bag('/tmp/test_io_close_fails.bag', 'w')
            b.close()
            size = b.size()
        self.failUnlessRaises(ValueError, fn)
        
    def test_write_invalid_message_fails(self):
        def fn():
            b = rosbag.Bag('/tmp/test_write_invalid_message_fails.bag', 'w')
            b.write(None, None, None)
        self.failUnlessRaises(ValueError, fn)

    def test_simple_write_uncompressed_works(self):
        b = rosbag.Bag('/tmp/test_simple_write_uncompressed_works.bag', 'w')
        msg_count = 0
        for i in range(5, 0, -1):
            msg = Int32()
            msg.data = i
            t = roslib.rostime.Time.from_sec(i)
            b.write('/ints' + str(i), msg, t)
            msg_count += 1
        b.close()

        msgs = list(rosbag.Bag('/tmp/test_simple_write_uncompressed_works.bag').read_messages())
        
        self.assert_(len(msgs) == msg_count, 'not all messages written: expected %d, got %d' % (msg_count, len(msgs)))

        for (_, _, t1), (_, _, t2) in zip(msgs, msgs[1:]):
            self.assert_(t1 < t2, 'messages returned unordered: got timestamp %s before %s' % (str(t1), str(t2)))

    def test_writing_nonchronological_works(self):
        b = rosbag.Bag('/tmp/test_writing_nonchronological_works.bag', 'w')
        msg_count = 0
        for i in range(5, 0, -1):
            msg = Int32()
            msg.data = i
            t = roslib.rostime.Time.from_sec(i)
            b.write('/ints', msg, t)
            msg_count += 1
        b.close()

        msgs = list(rosbag.Bag('/tmp/test_writing_nonchronological_works.bag').read_messages())
        
        self.assert_(len(msgs) == msg_count, 'not all messages written: expected %d, got %d' % (msg_count, len(msgs)))

        for (_, _, t1), (_, _, t2) in zip(msgs, msgs[1:]):
            self.assert_(t1 < t2, 'messages returned unordered: got timestamp %s before %s' % (str(t1), str(t2)))

    def test_large_write_works(self):
        for compression in [rosbag.Compression.NONE, rosbag.Compression.BZ2]:
            b = rosbag.Bag('/tmp/test_large_write_works.bag', 'w', compression=compression)
            msg_count = 0
            for i in range(10000):
                msg = Int32()
                msg.data = i
                t = roslib.rostime.Time.from_sec(i)
                b.write('/ints', msg, t)
                msg_count += 1
            b.close()

            msgs = list(rosbag.Bag('/tmp/test_large_write_works.bag').read_messages())

            self.assertEquals(len(msgs), msg_count, 'not all messages written: expected %d, got %d' % (msg_count, len(msgs)))

            for (_, _, t1), (_, _, t2) in zip(msgs, msgs[1:]):
                self.assert_(t1 < t2, 'messages returned unordered: got timestamp %s before %s' % (str(t1), str(t2)))

    def test_get_messages_time_range_works(self):
        b = rosbag.Bag('/tmp/test_get_messages_time_range_works.bag', 'w')
        for i in range(30):
            msg = Int32()
            msg.data = i
            t = roslib.rostime.Time.from_sec(i)
            b.write('/ints', msg, t)
        b.close()
        
        start_time = roslib.rostime.Time.from_sec(3)
        end_time = roslib.rostime.Time.from_sec(7)
        msgs = list(rosbag.Bag('/tmp/test_get_messages_time_range_works.bag').read_messages(topics='/ints', start_time=start_time, end_time=end_time))

        self.assertEquals(len(msgs), 5)
        
    def test_get_messages_filter_works(self):
        b = rosbag.Bag('/tmp/test_get_messages_filter_works.bag', 'w')
        for i in range(30):
            msg = Int32()
            msg.data = i
            t = roslib.rostime.Time.from_sec(i)
            b.write('/ints' + str(i), msg, t)
        b.close()

        def filter(topic, datatype, md5sum, msg_def, header):
            return '5' in topic and datatype == Int32._type and md5sum == Int32._md5sum and msg_def == Int32._full_text

        self.assertEquals(len(list(rosbag.Bag('/tmp/test_get_messages_filter_works.bag').read_messages(connection_filter=filter))), 3)

    def test_rosbag_filter(self):
        inbag_filename  = '/tmp/test_rosbag_filter__1.bag'
        outbag_filename = '/tmp/test_rosbag_filter__2.bag'
        
        b = rosbag.Bag(inbag_filename, 'w')
        for i in range(30):
            msg = Int32()
            msg.data = i
            t = roslib.rostime.Time.from_sec(i)
            b.write('/ints' + str(i), msg, t)
        b.close()

        expression = "(int(t.secs) == m.data) and (topic == '/ints' + str(m.data)) and (m.data >= 15 and m.data < 20)"

        os.system('rosbag filter %s %s "%s"' % (inbag_filename, outbag_filename, expression))

        msgs = list(rosbag.Bag(outbag_filename).read_messages())

        self.assertEquals(len(msgs), 5)

    def test_reindex_works(self):
        fn = '/tmp/test_reindex_works.bag'
        
        chunk_threshold = 1024

        with rosbag.Bag(fn, 'w', chunk_threshold=chunk_threshold) as b:
            for i in range(100):
                for j in range(5):
                    msg = Int32()
                    msg.data = i
                    b.write('/topic%d' % j, msg)
            file_header_pos = b._file_header_pos

        start_index = 4117 + chunk_threshold * 2 + chunk_threshold / 2

        trunc_filename   = '%s.trunc%s'   % os.path.splitext(fn)
        reindex_filename = '%s.reindex%s' % os.path.splitext(fn)

        for trunc_index in range(start_index, start_index + chunk_threshold):
            shutil.copy(fn, trunc_filename)
            
            with open(trunc_filename, 'r+b') as f:
                f.seek(file_header_pos)
                header = {
                    'op':          bag._pack_uint8(bag._OP_FILE_HEADER),
                    'index_pos':   bag._pack_uint64(0),
                    'conn_count':  bag._pack_uint32(0),
                    'chunk_count': bag._pack_uint32(0)
                }
                bag._write_record(f, header, padded_size=bag._FILE_HEADER_LENGTH)
                f.truncate(trunc_index)

            shutil.copy(trunc_filename, reindex_filename)
       
            try:
                b = rosbag.Bag(reindex_filename, 'a', allow_unindexed=True)
            except Exception, ex:
                pass
            for done in b.reindex():
                pass
            b.close()

            msgs = list(rosbag.Bag(reindex_filename, 'r'))
        
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, 'TestRosbag', TestRosbag, sys.argv)
