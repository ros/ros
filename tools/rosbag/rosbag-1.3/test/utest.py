# print.py

import roslib; roslib.load_manifest('rosbag')

import heapq
import time

import rosbag
from std_msgs.msg import Int32
from std_msgs.msg import ColorRGBA

def dump(f):
    b = rosbag.Bag()
    try:
        b.open(f, 'r')
        i = 0
        for message in b.getMessages():
            print message
            i += 1
    except Exception, ex:
        print str(ex)
    finally:
        b.close()
        
    b.reader.dump()

def main():
    import sys

    w = rosbag.Bag('ints.bag', 'w', compression=rosbag.Compression.BZ2)
    for i in range(9, 0, -1):
        msg = Int32()
        msg.data = i
        w.write('/int' + str(i), msg, roslib.rostime.Time.from_sec(i))
    w.close()

    for (topic, msg, t) in rosbag.Bag('ints.bag').getMessages():
        print topic, msg, t

    #print w.getMessages()
    #w.close()

    #dump('color.bag')

    #for f in sys.argv[1:]:
    #    dump(f)

if __name__ == '__main__':
    main()
    #import cProfile
    #cProfile.run('main()')
