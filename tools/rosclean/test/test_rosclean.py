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

def test__get_check_dirs():
    # just a tripwire, no way to assert the actual values w/o identical reimplementation
    from rosclean import _get_check_dirs
    vals = _get_check_dirs()
    for path, desc in vals:
        assert os.path.isdir(path)
    
def test_get_human_readable_disk_usage():
    from rosclean import get_human_readable_disk_usage
    val = get_human_readable_disk_usage(get_test_path())
    assert val
    
def get_test_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__)))

def test_get_disk_usage():
    from rosclean import get_disk_usage
    val = get_disk_usage(get_test_path())
    assert val > 0

def test_cmd():
    from rosclean import rosclean_main
    try:
        rosclean_main(['rosclean', 'fake'])
        assert False, "should have raised sys exit"
    except SystemExit:
        pass

    # should run cleanly
    try:
        rosclean_main(['rosclean', 'check'])
    except SystemExit:
        assert False, "failed with sys exit"
