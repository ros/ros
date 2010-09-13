#!/usr/bin/env python
import roslib; roslib.load_manifest('rosh')

# this changes the behavior of rospy/roslaunch/etc... on SIGINT to just be a nrmal KeyboardInterrupt
roslib.set_interactive(True)

import rosh
rosh.rosh_init()

# import all symbols after initialization
from rosh import *

