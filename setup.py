#!/usr/bin/env python

from setuptools import setup

import os
import rospkg.stack
import sys

version = '0.0.0'
try:
    version = rospkg.stack.parse_stack_file(os.path.join(os.path.dirname(__file__), 'stack.xml')).version
except Exception, e:
    pass

sys.path.insert(0, 'src')

PKG = 'ros_comm'
gen = ['msg', 'srv']
packages = []
package_dir = {}
if 'CATKIN_BINARY_DIR' in os.environ:
     build_d = os.environ['CATKIN_BINARY_DIR']
     for t in gen:
         p = os.path.join(build_d, 'gen', 'py', PKG, t)
         if os.path.isdir(p):
             # e.g. std_msgs.msg = build/gen/py/std_msgs/msg
             package_dir["%s.%s"%(PKG, t)] = p
             packages.append("%s.%s"%(PKG, t))

setup(name=PKG,
      version=version,
      packages=packages + ['rosbag',
                'rosgraph',
                'rosgraph.impl',
                'roslaunch',
                'rosmaster', 
                'rosmsg',
                'rosnode',
                'rosparam',
                'rospy',
                'rospy.impl',
                'rosservice',
                'rostopic',
                'rostest',
                'roswtf',
                'message_filters',
                'test_ros',
                'test_rosbag',
                ],
      package_dir = dict(package_dir, **{
          'rosbag':'tools/rosbag/src/rosbag',
          'rosgraph':'tools/rosgraph/src/rosgraph',
          'rosgraph.impl':'tools/rosgraph/src/rosgraph/impl',
          'roslaunch': 'tools/roslaunch/src/roslaunch',
          'rosmaster': 'tools/rosmaster/src/rosmaster',
          'rosmsg': 'tools/rosmsg/src/rosmsg',
          'rosnode': 'tools/rosnode/src/rosnode',
          'rosparam': 'tools/rosparam/src/rosparam',
          'rospy': 'clients/rospy/src/rospy',
          'rospy.impl': 'clients/rospy/src/rospy/impl',
          'rosservice': 'tools/rosservice/src/rosservice',
          'rostopic': 'tools/rostopic/src/rostopic',
          'rostest': 'tools/rostest/src/rostest',
          'roswtf': 'utilities/roswtf/src/roswtf',
          'message_filters': 'utilities/message_filters/src/message_filters',
          'test_ros': 'test/test_ros/src/test_ros',
          'test_rosbag': 'test/test_rosbag/src/test_rosbag',
                     }),
      install_requires=['rospkg', 'genmsg', 'genpy', 'roslib'],
      scripts = [
          'tools/rosbag/scripts/rosbag',
          'tools/rosgraph/scripts/rosgraph',

          'tools/roslaunch/scripts/roslaunch',
          'tools/roslaunch/scripts/roscore',
          'tools/roslaunch/scripts/roslaunch-logs',
          'tools/roslaunch/scripts/roslaunch-deps',

          'tools/rosmaster/scripts/rosmaster',

          'tools/rosmsg/scripts/rosmsg',
          'tools/rosmsg/scripts/rossrv',
          'tools/rosmsg/scripts/rosmsg-proto',
          
          'tools/rosnode/scripts/rosnode',
          'tools/rosparam/scripts/rosparam',
          'tools/rosservice/scripts/rosservice',
          'tools/rostest/scripts/rostest',
          'tools/rostopic/scripts/rostopic',

          'utilities/roswtf/scripts/roswtf',
                 ],
      author = "Maintained by Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/ros_comm",
      download_url = "http://pr.willowgarage.com/downloads/ros_comm/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "ros_comm Python libraries and tools",
      long_description = """\
Python libraries and toolchain for ROS communications-related packages, including core rospy client libraries and graph introspection tools (rostopic, rosnode, rosservice, rosparam).
""",
      license = "BSD"
      )

