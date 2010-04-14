# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
from __future__ import with_statement
PKG = 'roslib2'
import roslib; roslib.load_manifest(PKG)

import os
import sys
import unittest
import yaml

import roslib.packages
import rostest

def load_distros():
    """
    Load distro files as dicts
    """
    d = roslib.packages.get_pkg_dir(PKG)
    distros = {}
    for release_name in ['latest', 'boxturtle']:
        with open(os.path.join(d, 'test', '%s.rosdistro'%release_name)) as f:
            distros[release_name] = yaml.load(f.read())
    return distros

def load_Distros():
    """
    Load distro files as Distro instances
    """
    from roslib2.distro import Distro
    d = roslib.packages.get_pkg_dir(PKG)
    distros = {}
    for release_name in ['latest', 'boxturtle']:
        p = os.path.join(d, 'test', '%s.rosdistro'%release_name)
        distros[release_name] = Distro(p)
    return distros

boxturtle_ros_rules = {'dev-svn': 'https://code.ros.org/svn/ros/stacks/ros/tags/rc',
                       'distro-svn': 'https://code.ros.org/svn/ros/stacks/ros/tags/$RELEASE_NAME',
                       'release-svn': 'https://code.ros.org/svn/ros/stacks/ros/tags/$STACK_NAME-$STACK_VERSION',
                       'source-tarball': 'http://ros.org/download/stacks/$STACK_NAME/$STACK_NAME-$STACK_VERSION.tar.bz2'}
boxturtle_rospkg_rules = {'dev-svn': 'https://code.ros.org/svn/ros-pkg/stacks/$STACK_NAME/branches/$STACK_NAME-1.0',
                          'distro-svn': 'https://code.ros.org/svn/ros-pkg/stacks/$STACK_NAME/tags/$RELEASE_NAME',
                          'release-svn': 'https://code.ros.org/svn/ros-pkg/stacks/$STACK_NAME/tags/$STACK_NAME-$STACK_VERSION',
                          'source-tarball': 'http://ros.org/download/stacks/$STACK_NAME/$STACK_NAME-$STACK_VERSION.tar.bz2'}
boxturtle_wgrospkg_rules = {'dev-svn': 'https://code.ros.org/svn/wg-ros-pkg/stacks/$STACK_NAME/branches/$STACK_NAME-1.0',
                            'distro-svn': 'https://code.ros.org/svn/wg-ros-pkg/stacks/$STACK_NAME/tags/$RELEASE_NAME',
                            'release-svn': 'https://code.ros.org/svn/wg-ros-pkg/stacks/$STACK_NAME/tags/$STACK_NAME-$STACK_VERSION',
                            'source-tarball': 'http://ros.org/download/stacks/$STACK_NAME/$STACK_NAME-$STACK_VERSION.tar.bz2'}

wg_unbranched_rules = {'dev-svn': 'https://code.ros.org/svn/wg-ros-pkg/stacks/$STACK_NAME/trunk',
                       'distro-svn': 'https://code.ros.org/svn/wg-ros-pkg/stacks/$STACK_NAME/tags/$RELEASE_NAME',
                       'release-svn': 'https://code.ros.org/svn/wg-ros-pkg/stacks/$STACK_NAME/tags/$STACK_NAME-$STACK_VERSION',
                       'source-tarball': 'http://ros.org/download/stacks/$STACK_NAME/$STACK_NAME-$STACK_VERSION.tar.bz2'}

boxturtle_versions = {
  'common': '1.0.3',
  'common_msgs': '1.0.0',
  'geometry': '1.0.2',
  'navigation': '1.0.4',
  'pr2_common': '1.0.2',
  'pr2_mechanism': '1.0.2',
  'pr2_navigation': '0.1.1',
  'robot_model': '1.0.1',
  'ros':'1.0.1',
  'ros_experimental': '0.1.0',
  'simulator_gazebo': '1.0.3',
  'simulator_stage':  '1.0.0',
  'visualization': '1.0.1',
  'wg_common': '0.1.2',
  'wg_pr2_apps': '0.1.1',
  'wifi_drivers':'0.1.3',
  }

class DistroTest(unittest.TestCase):

    def test_Distro(self):
        # TODO: better unit tests. For now this is mostly a tripwire
        from roslib2.distro import Distro, DistroStack, Variant
        distros = load_Distros()

        r = 'boxturtle'
        v = '6'
        boxturtle = distros['boxturtle']
        
        self.assertEquals(r, boxturtle.release_name)
        self.assertEquals(v, boxturtle.version)        

        # make sure ros got assigned and is correct
        ros = DistroStack('ros', boxturtle_ros_rules, boxturtle_versions['ros'], r, v)
        self.assertEquals(ros, boxturtle.ros)
        self.assertEquals(ros, boxturtle.stacks['ros'])        

        # make sure we loaded the stacks
        latest = distros['boxturtle']        
        stack_names = ['common', 'common_msgs', 'navigation', 'geometry']
        for s in stack_names:
            val = DistroStack(s, boxturtle_rospkg_rules, boxturtle_versions[s], r, v)
            self.assertEquals(val, boxturtle.stacks[s])

        self.assertEquals(['base', 'pr2'], sorted(boxturtle.variants.keys()))
        #TODO: much more to test
        
    def test_get_rules(self):
        distros = load_distros()
        # boxturtle tests
        boxturtle = distros['boxturtle']
        from roslib2.distro import get_rules

        
        self.assertEquals(boxturtle_ros_rules, get_rules(boxturtle, 'ros'))

        for s in ['common', 'navigation', 'simulator_stage', 'visualization', 'visualization_common']:
            self.assertEquals(boxturtle_rospkg_rules, get_rules(boxturtle, s))
            
        for s in ['arm_navigation', 'motion_planners', 'pr2_calibration', 'pr2_ethercat_drivers']:
            self.assertEquals(wg_unbranched_rules, get_rules(boxturtle, s))
        
    def test_load_distro_stacks(self):
        from roslib2.distro import load_distro_stacks, DistroStack

        distros = load_distros()

        v = '6'
        r = 'boxturtle'
        boxturtle = distros[r]
        
        ros_version = boxturtle_versions['ros']
        self.assertEquals({}, load_distro_stacks(boxturtle, [], r, '5'))

        # - test with overrides
        ros = DistroStack('ros', boxturtle_ros_rules, ros_version, 'foxturtle', '55')
        self.assertEquals({'ros': ros}, load_distro_stacks(boxturtle, ['ros'], 'foxturtle', '55'))

        # - test with targetted rule change
        rules = boxturtle_rospkg_rules.copy()
        rules['dev-svn'] = 'https://madeup/stuff/$STACK_NAME/trunk'
        test_stack = DistroStack('test_rule_override', rules, '0.1.3', r, v)
        self.assertEquals({'test_rule_override': test_stack}, load_distro_stacks(boxturtle, ['test_rule_override']))

        # - test with actual distro values
        ros = DistroStack('ros', boxturtle_ros_rules, ros_version, r, v)
        self.assertEquals({'ros': ros}, load_distro_stacks(boxturtle, ['ros']))

        # test with the base stuff
        stack_names = ['ros', 'common', 'common_msgs', 'navigation', 'geometry']
        val = {'ros': ros}
        for s in [x for x in stack_names if x != 'ros']:
            val[s] = DistroStack(s, boxturtle_rospkg_rules, boxturtle_versions[s], r, v)

        self.assertEquals(val.keys(), load_distro_stacks(boxturtle, stack_names, r, v).keys())
        loaded = load_distro_stacks(boxturtle, stack_names, r, v)
        # iterate compare first for easy test diagnosis
        for k, item in val.iteritems():
            self.assertEquals(item, loaded[k], "failed on [%s]: %s"%(k, loaded[k]))
        # failsafe to ensure no extra items
        self.assertEquals(val, loaded)

        # add in some pr2 stacks which have different ways of setting rules
        pr2_stack_names = ['pr2_common', 'pr2_mechanism']
        for s in pr2_stack_names:
            val[s] = DistroStack(s, boxturtle_wgrospkg_rules, boxturtle_versions[s], r, v)
        stack_names = stack_names + pr2_stack_names
        self.assertEquals(val, load_distro_stacks(boxturtle, stack_names, r, v))

    def test_get_variants(self):
        import roslib2.distro
        from roslib2.distro import get_variants

        distros = load_distros()
        # boxturtle tests
        boxturtle = distros['boxturtle']
        self.assertEquals(['base', 'pr2'], get_variants(boxturtle, 'ros'))
        self.assertEquals(['base', 'pr2'], get_variants(boxturtle, 'navigation'))        
        self.assertEquals(['pr2'], get_variants(boxturtle, 'pr2_mechanism'))        
        self.assertEquals([], get_variants(boxturtle, 'arm_navigation'))
        self.assertEquals([], get_variants(boxturtle, 'fake'))        

        # latest tests
        latest = distros['latest']
        self.assertEquals(['base', 'pr2', 'pr2all'], get_variants(latest, 'ros'))
        self.assertEquals(['base', 'pr2','pr2all'], get_variants(latest, 'navigation'))        
        self.assertEquals(['pr2','pr2all'], get_variants(latest, 'pr2_mechanism'))        
        self.assertEquals(['pr2all'], get_variants(latest, 'arm_navigation'))        
        self.assertEquals([], get_variants(latest, 'fake'))
        
if __name__ == '__main__':
  rostest.unitrun('roslib2', 'test_distro', DistroTest, coverage_packages=['roslib2.distro'])

