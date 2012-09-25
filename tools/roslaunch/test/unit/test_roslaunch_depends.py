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

class Foo: pass

def test_RoslaunchDeps():
    from roslaunch.depends import RoslaunchDeps
    min_deps = RoslaunchDeps(nodes=[('rospy', 'talker.py')], pkgs=['rospy'])
    assert min_deps == min_deps
    assert min_deps == RoslaunchDeps(nodes=[('rospy', 'talker.py')], pkgs=['rospy'])
    assert not min_deps.__eq__(Foo())
    assert Foo() != min_deps
    assert min_deps != RoslaunchDeps(nodes=[('rospy', 'talker.py')])
    assert min_deps != RoslaunchDeps(pkgs=['rospy'])

    assert 'talker.py' in repr(min_deps)
    assert 'talker.py' in str(min_deps)
    
import cStringIO
from contextlib import contextmanager
@contextmanager
def fakestdout():
    realstdout = sys.stdout
    fakestdout = cStringIO.StringIO()
    sys.stdout = fakestdout
    yield fakestdout
    sys.stdout = realstdout

def test_roslaunch_deps_main():
    from roslaunch.depends import roslaunch_deps_main
    roslaunch_d = rospkg.RosPack().get_path('roslaunch')
    rosmaster_d = rospkg.RosPack().get_path('rosmaster')
    f = os.path.join(roslaunch_d, 'resources', 'example.launch')
    rosmaster_f = os.path.join(rosmaster_d, 'test', 'rosmaster.test')
    invalid_f = os.path.join(roslaunch_d, 'test', 'xml', 'invalid-xml.xml')
    not_f = os.path.join(roslaunch_d, 'test', 'xml', 'not-launch.xml')
    
    with fakestdout() as b:
        roslaunch_deps_main(['roslaunch-deps', f])
        s = b.getvalue()
        assert s.strip() == 'rospy', "buffer value [%s]"%(s)
    with fakestdout() as b:
        roslaunch_deps_main(['roslaunch-deps', f, '--warn'])
        s = b.getvalue()
        assert s.strip() == """Dependencies:
rospy

Missing declarations:
roslaunch/manifest.xml:
  <depend package="rospy" />""", s

    # tripwire, don't really care about exact verbose output
    with fakestdout() as b:
        roslaunch_deps_main(['roslaunch-deps', f, '--verbose'])

    # try with no file
    with fakestdout() as b:
        try:
            roslaunch_deps_main(['roslaunch-deps'])
            assert False, "should have failed"
        except SystemExit:
            pass

    # try with non-existent file
    with fakestdout() as b:
        try:
            roslaunch_deps_main(['roslaunch-deps', 'fakefile', '--verbose'])
            assert False, "should have failed"
        except SystemExit:
            pass

    # try with bad file
    with fakestdout() as b:
        try:
            roslaunch_deps_main(['roslaunch-deps', invalid_f, '--verbose'])
            assert False, "should have failed: %s"%b.getvalue()
        except SystemExit:
            pass
    with fakestdout() as b:
        try:
            roslaunch_deps_main(['roslaunch-deps', not_f, '--verbose'])
            assert False, "should have failed: %s"%b.getvalue()
        except SystemExit:
            pass

    # try with files from different pacakges
    with fakestdout() as b:
        try:
            roslaunch_deps_main(['roslaunch-deps', f, rosmaster_f, '--verbose'])
            assert False, "should have failed"
        except SystemExit:
            pass
    
def test_roslaunch_deps():
    from roslaunch.depends import roslaunch_deps, RoslaunchDeps
    example_d = os.path.join(rospkg.RosPack().get_path('roslaunch'), 'resources')

    min_deps = RoslaunchDeps(nodes=[('rospy', 'talker.py')], pkgs=['rospy'])
    include_deps = RoslaunchDeps(nodes=[('rospy', 'talker.py'), ('rospy', 'listener.py')], pkgs=['rospy'])
    example_deps = RoslaunchDeps(nodes=[('rospy', 'talker.py'), ('rospy', 'listener.py')], pkgs=['rospy'],
                                 includes=[os.path.join(example_d, 'example-include.launch')])

    example_file_deps = {
        os.path.join(example_d, 'example.launch') : example_deps,

        os.path.join(example_d, 'example-include.launch') : include_deps,
        }
    example_min_file_deps = {
        os.path.join(example_d, 'example-min.launch') : min_deps,
        }
    r_missing = {'roslaunch': set(['rospy'])}
    tests = [
        ([os.path.join(example_d, 'example-min.launch')], ('roslaunch', example_min_file_deps, r_missing)),

        ([os.path.join(example_d, 'example.launch')], ('roslaunch', example_file_deps, r_missing)),

        ]
    for files, results in tests:
        for v in [True, False]:
            base_pkg, file_deps, missing = roslaunch_deps(files, verbose=v)
            assert base_pkg == results[0]
            assert file_deps == results[1], "\n%s vs \n%s"%(file_deps, results[1])
            assert missing == results[2], "\n%s vs \n%s"%(missing, results[2])
            
