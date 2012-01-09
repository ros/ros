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

PKG = 'test_roslaunch'

import os, sys, unittest

import xmlrpclib
import rostest
import rospkg

import rosgraph
master = rosgraph.Master('params_basic')
def get_param(*args):
    return master.getParam(*args)
    
## Test Roslaunch 'param' tags
class TestParamsBasic(unittest.TestCase):

    ## test primitive values
    def test_values(self):
        ## Test roslaunch string params
        self.assertEquals(get_param('stringempty'), '')
        print get_param('stringbar')
        self.assertEquals(get_param('stringbar'), 'bar')
        self.assertEquals(get_param('str10'), '10')
        self.assertEquals(get_param('string10'), '10')        
        self.assertEquals(get_param('stringentity'), '<stringentity/>')        
        ## Test roslaunch integer params
        self.assertEquals(get_param("integerneg1"), -1)
        self.assertEquals(get_param("integer0"), 0)
        self.assertEquals(get_param("integer1"), 1)
        self.assertEquals(get_param("integernoop1"), 1)
        self.assertEquals(get_param("integer12345"), 12345)
        ## Test roslaunch float params
        self.assertEquals(get_param("floatpi"),3.14159)
        self.assertEquals(get_param("floatnooppi"),3.14159)
        self.assertEquals(get_param("float3"),3.0)
        self.assertEquals(get_param("floatneg1"),-1.0)
        ## Test roslaunch boolean params
        for p in ['true', 'TRUE', 'True']:
            self.assertTrue(get_param(p), "[%s] is not false: %s"%(p, get_param(p)))
        for p in ['false', "FALSE", 'False']:
            self.assertFalse(get_param(p), "[%s] is not false: %s"%(p, get_param(p)))
            
    ## Test roslaunch ns attribute (namespace) on params
    def test_ns(self):
        self.assertEquals(get_param("/wg/childparam"),"wg")
        self.assertEquals(get_param("/wg2/childparam"),"wg2")
        self.assertEquals(get_param("/wg3/childparam"),"wg3")
        self.assertEquals(get_param("/wg/wg4/childparam"),"wg4")
        self.assertEquals(get_param("/wg/wg4/wg5/childparam"),"wg5")          
        ## Test roslaunch <group> tag with and without ns attribute
        self.assertEquals(get_param("/wga/wg/childparam"),"wg")
        self.assertEquals(get_param("/wga/wg2/childparam"),"wg2")
        self.assertEquals(get_param("/wga/wg3/childparam"),"wg3")
        self.assertEquals(get_param("/wga/wg/wg4/childparam"),"wg4")
        self.assertEquals(get_param("/wga/wg/wg4/wg5/childparam"),"wg5")          
        # test second-level group
        self.assertEquals(get_param("/wga/wgb/wg/childparam"),"bwg")
        self.assertEquals(get_param("/wga/wgb/wg2/childparam"),"bwg2")
        self.assertEquals(get_param("/wga/wgb/wg3/childparam"),"bwg3")
        self.assertEquals(get_param("/wga/wgb/wg/wg4/childparam"),"bwg4")
        self.assertEquals(get_param("/wga/wgb/wg/wg4/wg5/childparam"),"bwg5")
        # test unscoped group
        self.assertEquals(get_param("/wgc/childparam"),"wg")
        self.assertEquals(get_param("/wgc2/childparam"),"wg2")
        self.assertEquals(get_param("/wgc3/childparam"),"wg3")
        self.assertEquals(get_param("/wgc/wg4/childparam"),"wg4")
        self.assertEquals(get_param("/wgc/wg4/wg5/childparam"),"wg5")          
        
    ## test 'command' attribute
    def test_commandandfile(self):
        dir = rospkg.RosPack().get_path('roslaunch')
        with open(os.path.join(dir, 'resources', 'example.launch'), 'r') as f:
            data = f.read()
        test_file = data
        self.assertEquals(get_param("commandoutput"),test_file)
        self.assertEquals(get_param("textfile"),test_file)
        ## test 'binfile' attribute
        bindata = get_param("binaryfile")
        self.assertTrue(isinstance(bindata, xmlrpclib.Binary))
        self.assertEquals(bindata.data,test_file)
    
if __name__ == '__main__':
    rostest.rosrun(PKG, sys.argv[0], TestParamsBasic, sys.argv)
    
