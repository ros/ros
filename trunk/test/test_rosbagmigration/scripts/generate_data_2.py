#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#


import roslib; roslib.load_manifest('test_rosbagmigration')

import rospy
from test_rosbagmigration.msg import *

import rosrecord

def generate_data():
    bag = rosrecord.Rebagger("test/migrated_explicit_gen2.bag")
    m = MigratedExplicit(None, 17, 58.2, "aldfkja", 82)
    bag.add("migrated_explicit", m, roslib.rostime.Time())
    bag.close()

    bag = rosrecord.Rebagger("test/migrated_implicit_gen2.bag")
    m = MigratedImplicit(None, 34, 16.32, "kljene", MigratedExplicit(None, 17, 58.2, "aldfkja", 82))
    bag.add("migrated_implicit", m, roslib.rostime.Time())
    bag.close()

    bag = rosrecord.Rebagger("test/migrated_mixed_gen2.bag")
    m = MigratedMixed(None, MigratedImplicit(None, 34, 16.32, "kljene", MigratedExplicit(None, 17, 58.2, "aldfkja", 82)))
    bag.add("migrated_mixed", m, roslib.rostime.Time())
    bag.close()

    bag = rosrecord.Rebagger("test/partially_migrated_gen2.bag")
    m = PartiallyMigrated(40, MigratedExplicit(None, 17, 58.2, "aldfkja", 82))
    bag.add("partially_migrated", m, roslib.rostime.Time())
    bag.close()

    bag = rosrecord.Rebagger("test/renamed_gen2.bag")
    m = Renamed2(2.17, [8, 2, 5])
    bag.add("renamed", m, roslib.rostime.Time())
    bag.close()

    bag = rosrecord.Rebagger("test/convergent_gen2.bag")
    m = Convergent(1.2, 3.4, 5.6, 7.8, SimpleMigrated(11), SimpleMigrated(22), SimpleMigrated(33), SimpleMigrated(44))
    bag.add("convergent", m, roslib.rostime.Time())
    bag.close()

    bag = rosrecord.Rebagger("test/converged_gen2.bag")
    m = Converged([1.2, 3.4, 5.6, 7.8], [SimpleMigrated(11), SimpleMigrated(22), SimpleMigrated(33), SimpleMigrated(44)])
    bag.add("converged", m, roslib.rostime.Time())
    bag.close()

    bag = rosrecord.Rebagger("test/constants_gen2.bag")
    m = Constants(Constants.CONSTANT)
    bag.add("constants", m, roslib.rostime.Time())
    bag.close()
        
if __name__ == '__main__':
    generate_data()
