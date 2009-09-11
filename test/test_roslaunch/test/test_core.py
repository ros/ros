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
# Revision $Id: tcpros.py 1636 2008-07-28 20:58:29Z sfkwc $

PKG = 'test_roslaunch'
NAME = 'test_core'

import roslib; roslib.load_manifest(PKG)

import os, sys, unittest

_lastmsg = None
def printlog_cb(msg):
    global _lastmsg 
    _lastmsg = msg
def printlog_cb_exception(msg):
    raise Exception(msg)
    
## Test roslaunch.core
class TestCore(unittest.TestCase):
    def test_child_mode(self):
        from roslaunch.core import is_child_mode, set_child_mode
        self.failIf(is_child_mode())
        set_child_mode(True)
        self.assert_(is_child_mode())        
        set_child_mode(False)
        self.failIf(is_child_mode())

    def test_printlog(self):
        from roslaunch.core import add_printlog_handler, add_printerrlog_handler, printlog, printlog_bold, printerrlog
        add_printlog_handler(printlog_cb)
        add_printlog_handler(printlog_cb_exception)        
        add_printerrlog_handler(printlog_cb)
        add_printerrlog_handler(printlog_cb_exception)        
            
        #can't really test functionality, just make sure it doesn't crash
        global _lastmsg
        _lastmsg = None
        printlog('foo')
        self.assertEquals('foo', _lastmsg)
        printlog_bold('bar')
        self.assertEquals('bar', _lastmsg)        
        printerrlog('baz')
        self.assertEquals('baz', _lastmsg)        
        
    def test_Master(self):
        from roslaunch.core import Master
        old_env = os.environ.get('ROS_MASTER_URI', None)
        try:
            os.environ['ROS_MASTER_URI'] = 'http://foo:789'
            m = Master()
            self.assertEquals(m.type, Master.ZENMASTER)
            self.assertEquals(m.uri, 'http://foo:789')
            self.assertEquals(m.auto, Master.AUTO_NO)
            self.assertEquals(m.log_output, False)
            
            m = Master(Master.ZENMASTER, 'http://foo:1234', Master.AUTO_START)
            self.assertEquals(m.type, Master.ZENMASTER)
            self.assertEquals(m.uri, 'http://foo:1234')
            self.assertEquals(m.auto, Master.AUTO_START)
            self.assertEquals(m.log_output, False)
            
            import xmlrpclib
            self.assert_(isinstance(m.get(), xmlrpclib.ServerProxy))
            m.set_port(567)
            self.assertEquals(m.uri, 'http://foo:567/')
            self.failIf(m.is_running())

            try:
                m = Master(Master.ZENMASTER, 'http://foo:1234', False)
                self.fail("should have failed on invalid auto value")
            except: pass
            try:
                m = Master(Master.ZENMASTER, 'http://foo:1234', 123)
                self.fail("should have failed on invalid auto value")
            except: pass
            
        finally:
            if old_env is None:
                del os.environ['ROS_MASTER_URI']
            else:
                os.environ['ROS_MASTER_URI'] = old_env

    def test_Executable(self):
        from roslaunch.core import Executable, PHASE_SETUP, PHASE_RUN, PHASE_TEARDOWN

        e = Executable('ls', ['-alF'])
        self.assertEquals('ls', e.command)
        self.assertEquals(['-alF'], e.args)
        self.assertEquals(PHASE_RUN, e.phase)
        self.assertEquals('ls -alF', str(e))
        self.assertEquals('ls -alF', repr(e))        

        e = Executable('ls', ['-alF', 'b*'], PHASE_TEARDOWN)
        self.assertEquals('ls', e.command)
        self.assertEquals(['-alF', 'b*'], e.args)
        self.assertEquals(PHASE_TEARDOWN, e.phase)
        self.assertEquals('ls -alF b*', str(e))
        self.assertEquals('ls -alF b*', repr(e))        

    def test_RosbinExecutable(self):
        from roslaunch.core import RosbinExecutable, PHASE_SETUP, PHASE_RUN, PHASE_TEARDOWN

        e = RosbinExecutable('ls', ['-alF'])
        self.assertEquals('ls', e.command)
        self.assertEquals(['-alF'], e.args)
        self.assertEquals(PHASE_RUN, e.phase)
        self.assertEquals('ros/bin/ls -alF', str(e))
        self.assertEquals('ros/bin/ls -alF', repr(e))        

        e = RosbinExecutable('ls', ['-alF', 'b*'], PHASE_TEARDOWN)
        self.assertEquals('ls', e.command)
        self.assertEquals(['-alF', 'b*'], e.args)
        self.assertEquals(PHASE_TEARDOWN, e.phase)
        self.assertEquals('ros/bin/ls -alF b*', str(e))
        self.assertEquals('ros/bin/ls -alF b*', repr(e))        
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_roslaunch', NAME, TestCore, coverage_packages=['roslaunch.core'])
    
