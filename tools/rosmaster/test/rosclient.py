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
# Revision $Id: test_embed_msg.py 1986 2008-08-26 23:57:56Z sfkwc $

import unittest
import xmlrpclib

import rosgraph

class TestRosClient(unittest.TestCase):

    def setUp(self):
        self.last_code = None
        self.last_msg = None
        self.last_val = None
        self.master = xmlrpclib.ServerProxy(rosgraph.get_master_uri())
    
    def tearDown(self):
        self.master = None
        
    ## unit test assertion that fails if status code is not 1 and otherwise returns the value parameter
    ## @param args [int, str, val]: returnv value from ROS API call
    ## @return val value parameter from args (arg[2] for master/slave API)
    def apiSuccess(self, args):
        self.assert_(len(args) == 3, "invalid API return value triplet: %s"%str(args))
        self.last_code, self.last_msg, self.last_val = args
        assert self.last_code == 1, "status code is not 1: %s"%self.last_msg
        return self.last_val

    ## unit test assertions that fails if status code is not 0 and otherwise returns true
    ## @param args [int, str, val]: returnv value from ROS API call
    ## @return True if status code is 0
    def apiFail(self, args):
        self.assert_(len(args) == 3, "invalid API return value triplet: %s"%str(args))
        self.last_code, self.last_msg, self.last_val = args
        assert self.last_code == 0, "Call should have failed with status code 0: %s"%self.last_msg

    ## unit test assertion that fails if status code is not -1 and otherwise returns true
    ## @return True if status code is -1
    def apiError(self, args, msg=None):
        self.assert_(len(args) == 3, "invalid API return value triplet: %s"%str(args))
        self.last_code, self.last_msg, self.last_val = args
        if msg:
            assert self.last_code == -1, "%s (return msg was %s)"%(msg, self.last_msg)
        else:
            assert self.last_code == -1, "Call should have returned error -1 code: %s"%self.last_msg            
    
