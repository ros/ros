# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

def test_vars():
    import rosgraph.rosenv
    assert 'ROS_MASTER_URI' == rosgraph.rosenv.ROS_MASTER_URI
    assert rosgraph.rosenv.ROS_IP == 'ROS_IP'
    assert rosgraph.rosenv.ROS_HOSTNAME == 'ROS_HOSTNAME'
    assert rosgraph.rosenv.ROS_NAMESPACE == 'ROS_NAMESPACE'
    
def test_get_master_uri():
    from rosgraph.rosenv import get_master_uri
    val = get_master_uri()
    if 'ROS_MASTER_URI' in os.environ:
        assert val == os.environ['ROS_MASTER_URI']

    # environment override
    val = get_master_uri(env=dict(ROS_MASTER_URI='foo'))
    assert val == 'foo'

    # argv override precedence, first arg wins
    val = get_master_uri(env=dict(ROS_MASTER_URI='foo'), argv=['__master:=bar', '__master:=bar2'])
    assert val == 'bar'

    # empty env
    assert None == get_master_uri(env={})
    
    # invalid argv
    try:
        val = get_master_uri(argv=['__master:='])
        assert False, "should have failed"
    except ValueError:
        pass
    # invalid argv
    try:
        val = get_master_uri(argv=['__master:=foo:=bar'])
        assert False, "should have failed"
    except ValueError:
        pass

    
    

    
    
