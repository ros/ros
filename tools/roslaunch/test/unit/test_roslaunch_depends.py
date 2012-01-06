# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

import os
import sys

import rospkg

def test_roslaunch_deps():
    from roslaunch.depends import roslaunch_deps, RoslaunchDeps
    roslaunch_d = rospkg.RosPack().get_path('roslaunch')

    min_deps = RoslaunchDeps(nodes=[('test_ros', 'talker.py')], pkgs=['test_ros'])
    include_deps = RoslaunchDeps(nodes=[('test_ros', 'talker.py'), ('test_ros', 'listener.py')], pkgs=['test_ros'])
    example_deps = RoslaunchDeps(nodes=[('test_ros', 'listener.py'), ('test_ros', 'talker.py')], pkgs=['test_ros'],
                                 includes=os.path.join(roslaunch_d, 'example-include.launch'))

    example_file_deps = {
        os.path.join(roslaunch_d, 'example.launch') : example_deps,

        os.path.join(roslaunch_d, 'example-include.launch') : min_deps,
        }
    example_min_file_deps = {
        os.path.join(roslaunch_d, 'example-min.launch') : min_deps,
        }
    r_missing = {'roslaunch': set(['test_ros'])}
    tests = [
        ([os.path.join(roslaunch_d, 'example-min.launch')], ('roslaunch', example_min_file_deps, r_missing)),

        ([os.path.join(roslaunch_d, 'example.launch')], ('roslaunch', example_file_deps, r_missing)),

        ]
    for files, results in tests:
        for v in [True, False]:
            base_pkg, file_deps, missing = roslaunch_deps(files, verbose=v)
            assert base_pkg == results[0]
            assert file_deps == results[1], "%s vs %s"%(file_deps, results[1])
            assert missing == results[2], "%s vs %s"%(missing, results[2])
            
