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

import os
import unittest


class RoslibPackagesTest(unittest.TestCase):

    def test_find_node(self):
        import roslib.packages
        d = roslib.packages.get_pkg_dir('roslib')
        p = os.path.join(d, 'test', 'fake_node.py')
        self.assertEquals([p], roslib.packages.find_node('roslib', 'fake_node.py'))

        self.assertEquals([], roslib.packages.find_node('roslib', 'not_a_node'))

    def test_get_pkg_dir(self):
        import roslib.packages
        import roslib.rospack
        path = os.path.normpath(roslib.rospack.rospackexec(['find', 'roslib']))
        self.assertEquals(path, roslib.packages.get_pkg_dir('roslib'))
        try:
            self.assertEquals(path, roslib.packages.get_pkg_dir('fake_roslib'))
            self.fail('should have raised')
        except roslib.packages.InvalidROSPkgException:
            pass

    def test_get_dir_pkg(self):
        import roslib.packages
        path = get_roslib_path()

        res = roslib.packages.get_dir_pkg(path)
        res = (os.path.realpath(res[0]), res[1])
        self.assertEquals((path, 'roslib'), res)
        res = roslib.packages.get_dir_pkg(os.path.join(path, 'test'))
        res = (os.path.realpath(res[0]), res[1])
        self.assertEquals((path, 'roslib'), res)

        # must fail on parent of roslib
        self.assertEquals((None, None), roslib.packages.get_dir_pkg(os.path.dirname(path)))


def get_roslib_path():
    return os.path.realpath(os.path.abspath(os.path.join(get_test_path(), '..')))


def get_test_path():
    return os.path.abspath(os.path.dirname(__file__))
