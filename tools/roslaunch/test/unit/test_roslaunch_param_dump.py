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

    
import cStringIO
from contextlib import contextmanager
@contextmanager
def fakestdout():
    realstdout = sys.stdout
    fakestdout = cStringIO.StringIO()
    sys.stdout = fakestdout
    yield fakestdout
    sys.stdout = realstdout

import rospkg
import logging

SAMPLE1 = """/rosparam_load/dict1/head: 1
/rosparam_load/dict1/knees: 3
/rosparam_load/dict1/shoulders: 2
/rosparam_load/dict1/toes: 4
/rosparam_load/integer1: 1
/rosparam_load/integer2: 2
/rosparam_load/list1: [head, shoulders, knees, toes]
/rosparam_load/list2: [1, 1, 2, 3, 5, 8]
/rosparam_load/preformattedtext: 'This is the first line

  This is the second line

  Line breaks are preserved

  Indentation is stripped

  '
/rosparam_load/robots/child/grandchildparam: a grandchild namespace param
/rosparam_load/robots/childparam: a child namespace parameter
/rosparam_load/string1: bar
/rosparam_load/string2: '10'"""

SAMPLE2 = """/load_ns/subns/dict1/head: 1
/load_ns/subns/dict1/knees: 3
/load_ns/subns/dict1/shoulders: 2
/load_ns/subns/dict1/toes: 4
/load_ns/subns/integer1: 1
/load_ns/subns/integer2: 2
/load_ns/subns/list1: [head, shoulders, knees, toes]
/load_ns/subns/list2: [1, 1, 2, 3, 5, 8]
/load_ns/subns/preformattedtext: 'This is the first line

  This is the second line

  Line breaks are preserved

  Indentation is stripped

  '
/load_ns/subns/robots/child/grandchildparam: a grandchild namespace param
/load_ns/subns/robots/childparam: a child namespace parameter
/load_ns/subns/string1: bar
/load_ns/subns/string2: '10'"""


def test_dump_params():
    # normal entrypoint has logging configured
    logger = logging.getLogger('roslaunch').setLevel(logging.CRITICAL)
    from roslaunch.param_dump import dump_params
    roslaunch_d = rospkg.RosPack().get_path('roslaunch')
    test_d = os.path.join(roslaunch_d, 'test', 'xml')
    node_rosparam_f = os.path.join(test_d, 'test-node-rosparam-load.xml')
    with fakestdout() as b:
        assert dump_params([node_rosparam_f])
        s = b.getvalue().strip()
        # remove float vals as serialization is not stable
        s = '\n'.join([x for x in s.split('\n') if not 'float' in x])
        assert str(s) == str(SAMPLE1), "[%s]\nvs\n[%s]"%(s, SAMPLE1)
    node_rosparam_f = os.path.join(test_d, 'test-node-rosparam-load-ns.xml')
    with fakestdout() as b:
        assert dump_params([node_rosparam_f])
        s = b.getvalue().strip()
        # remove float vals as serialization is not stable
        s = '\n'.join([x for x in s.split('\n') if not 'float' in x])
        assert str(s) == str(SAMPLE2), "[%s]\nvs\n[%s]"%(s, SAMPLE2)
        
    invalid_f = os.path.join(test_d, 'invalid-xml.xml')
    with fakestdout() as b:
        assert not dump_params([invalid_f])

