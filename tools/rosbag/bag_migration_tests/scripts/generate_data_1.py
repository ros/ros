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

import rospy
from test_rosbag.msg import *

import genpy
import rosbag

def generate_data():
    bag = rosbag.Bag("test/migrated_explicit_gen1.bag", "w")
    m = MigratedExplicit(None, 17, 58.2, "aldfkja")
    bag.write("migrated_explicit", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/migrated_implicit_gen1.bag", "w")
    m = MigratedImplicit(None, 34, 16.32, "kljene", MigratedExplicit(None, 17, 58.2, "aldfkja"))
    bag.write("migrated_implicit", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/migrated_mixed_gen1.bag", "w")
    m = MigratedMixed(None, MigratedImplicit(None, 34, 16.32, "kljene", MigratedExplicit(None, 17, 58.2, "aldfkja")))
    bag.write("migrated_mixed", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/migrated_addsub_gen1.bag", "w")
    m = MigratedAddSub(Simple(42))
    bag.write("migrated_addsub", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/unmigrated_gen1.bag", "w")
    m = Unmigrated(12, "uuiasjs")
    bag.write("unmigrated", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/subunmigrated_gen1.bag", "w")
    m = SubUnmigrated(92, Unmigrated(12, "uuiasjs"))
    bag.write("subunmigrated", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/partially_migrated_gen1.bag", "w")
    m = PartiallyMigrated(40, MigratedExplicit(None, 17, 58.2, "aldfkja"))
    bag.write("partially_migrated", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/renamed_gen1.bag", "w")
    m = Renamed1(2.17, [8, 2, 5])
    bag.write("renamed", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/convergent_gen1.bag", "w")
    m = Convergent(1.2, 3.4, 5.6, 7.8, SimpleMigrated(11), SimpleMigrated(22), SimpleMigrated(33), SimpleMigrated(44))
    bag.write("convergent", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/converged_gen1.bag", "w")
    m = Converged([1.2, 3.4, 5.6, 7.8], [SimpleMigrated(11), SimpleMigrated(22), SimpleMigrated(33), SimpleMigrated(44)])
    bag.write("converged", m, genpy.Time())
    bag.close()

    bag = rosbag.Bag("test/constants_gen1.bag", "w")
    m = Constants(Constants.CONSTANT)
    bag.write("constants", m, genpy.Time())
    bag.close()
        
if __name__ == '__main__':
    generate_data()
