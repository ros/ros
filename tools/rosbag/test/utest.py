# utest.py

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)

import heapq
import sys
import time
import unittest

import rosbag
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import ColorRGBA

class TestRosbag(unittest.TestCase):
    def setUp(self):
        pass

    def test_write_non_chronological_fails(self):
        def fn():
            b = rosbag.Bag('test.bag', 'w')
            for i in range(5, 0, -1):
                msg = Int32()
                msg.data = i
                b.write('/ints', msg, roslib.rostime.Time.from_sec(i))
            b.close()

        self.failUnlessRaises(rosbag.ROSBagException, fn)
        
    def test_simple_write_uncompressed(self):
        b = rosbag.Bag('test.bag', 'w')
        msg_count = 0
        for i in range(5, 0, -1):
            msg = Int32()
            msg.data = i
            t = roslib.rostime.Time.from_sec(i)
            b.write('/ints' + str(i), msg, t)
            msg_count += 1
        b.close()

        msgs = list(rosbag.Bag('test.bag').getMessages())
        
        self.assert_(len(msgs) == msg_count, 'not all messages written: expected %d, got %d' % (msg_count, len(msgs)))

        for (_, _, t1), (_, _, t2) in zip(msgs, msgs[1:]):
            self.assert_(t1 < t2, 'messages returned unordered: got timestamp %s before %s' % (str(t1), str(t2)))

    def test_large_write_compressed(self):
        b = rosbag.Bag('large_write.bag', 'w')
        msg_count = 0
        for i in range(10000):
            msg = Int32()
            msg.data = i
            t = roslib.rostime.Time.from_sec(i)
            b.write('/ints', msg, t)
            msg_count += 1
        b.close()

        msgs = list(rosbag.Bag('large_write.bag').getMessages())

        self.assert_(len(msgs) == msg_count, 'not all messages written: expected %d, got %d' % (msg_count, len(msgs)))

        for (_, _, t1), (_, _, t2) in zip(msgs, msgs[1:]):
            self.assert_(t1 < t2, 'messages returned unordered: got timestamp %s before %s' % (str(t1), str(t2)))

    #def test_simple_write(self):
        #self.failIf(timeout_t < time.time(), "timeout exceeded")
        #self.assert_(self.callback_invoked[0], "callback not invoked")

def main():
    sys.exit()

if __name__ == '__main__':
    #main()

    import rostest
    rostest.run(PKG, 'TestRosbag', TestRosbag, sys.argv)
