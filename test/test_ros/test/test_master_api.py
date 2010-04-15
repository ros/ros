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

## Integration test for empty services to test serializers
## and transport

PKG = 'test_ros'
NAME = 'test_master'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

import rospy

from test_ros.master import MasterApiTestCase, set_node_name

# Due to the need to have a fresh master for each of these test cases,
# we have to go through the pain of exposing each of the test cases one-by-one

class MasterSimpleApi(MasterApiTestCase):
    def testGetPid(self):
        self._testGetPid()
    def testGetUri(self):
        self._testGetUri()

class MasterRegisterServiceSuccess(MasterApiTestCase):
    def testRegisterServiceSuccess(self):
        self._testRegisterServiceSuccess()
        
class MasterRegisterPublisherSuccess(MasterApiTestCase):        
    def testRegisterPublisherSuccess(self):
        self._testRegisterPublisherSuccess()
class MasterRegisterPublisherTypes(MasterApiTestCase):
    ## #591: this test may change if we make registering '*' unsupported
    def testRegisterPublisherTypes(self):
        self._testRegisterPublisherTypes()

class MasterRegisterSubscriberSimpleSuccess(MasterApiTestCase):     
    def testRegisterSubscriberSimpleSuccess(self):
        self._testRegisterSubscriberSimpleSuccess()


class MasterUnregisterServiceSuccess(MasterApiTestCase):
    def testUnregisterServiceSuccess(self):
        self._testUnregisterServiceSuccess()

class MasterUnregisterPublisherSuccess(MasterApiTestCase):     
    def testUnregisterPublisherSuccess(self):
        self._testUnregisterPublisherSuccess()

class MasterUnregisterSubscriberSuccess(MasterApiTestCase):     
    def testUnregisterSubscriberSuccess(self):
        self._testUnregisterSubscriberSuccess()

class MasterRegisterServiceInvalid(MasterApiTestCase):
    def testRegisterServiceInvalid(self):
        self._testRegisterServiceInvalid()

class MasterRegisterPublisherInvalid(MasterApiTestCase):
    def testRegisterPublisherInvalid(self):
        self._testRegisterPublisherInvalid()

class MasterRegisterSubscriberInvalid(MasterApiTestCase):     
    def testRegisterSubscriberInvalid(self):
        self._testRegisterSubscriberInvalid()


class MasterUnregisterServiceInvalid(MasterApiTestCase):
    def testUnregisterServiceInvalid(self):
        self._testUnregisterServiceInvalid()

class MasterUnregisterPublisherInvalid(MasterApiTestCase):
    def testUnregisterPublisherInvalid(self):
        self._testUnregisterPublisherInvalid()

class MasterUnregisterSubscriberInvalid(MasterApiTestCase):     
    def testUnregisterSubscriberInvalid(self):
        self._testUnregisterSubscriberInvalid()


        
if __name__ == '__main__':
    # this is terribly complicated on the account that we want a fresh master for each test, so we cannot
    # run all the tests as a single test node. instead, we have to have a separate test node per test.
    
    import optparse
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options] topic", prog=NAME)

    # have to redeclare --text/--cov options, which are standard rostest options
    parser.add_option("--text",dest="text_ignore", default=False,
                      action="store_true", help="rostest standard option")
    parser.add_option("--cov",dest="cov_ignore", default=False,
                      action="store_true", help="rostest standard option")
    
    parser.add_option("--simple",dest="simple", default=False,
                      action="store_true", help="MasterSimpleApi")

    parser.add_option("--gtest_output",dest="gtest_output", default='',
                      help="xml output file")

    parser.add_option("--regsrvsuccess", dest="regsrvsuccess", default=False,
                      action="store_true", help="MasterRegisterServiceSuccess")
    parser.add_option("--regpubsuccess", dest="regpubsuccess", default=False,
                      action="store_true", help="MasterRegisterPublisherSuccess")
    parser.add_option("--regpubtypes", dest="regpubtypes", default=False,
                      action="store_true", help="MasterRegisterPublisherTypes")
    parser.add_option("--regsubsimplesuccess", dest="regsubsimplesuccess", default=False,
                      action="store_true", help="MasterRegisterSubscriberSimpleSuccess")


    parser.add_option("--unregsrvsuccess", dest="unregsrvsuccess", default=False,
                      action="store_true", help="MasterUnregisterServiceSuccess")
    parser.add_option("--unregpubsuccess", dest="unregpubsuccess", default=False,
                      action="store_true", help="MasterUnregisterPublisherSuccess")
    parser.add_option("--unregsubsuccess", dest="unregsubsuccess", default=False,
                      action="store_true", help="MasterUnregisterSubscriberSuccess")

    parser.add_option("--regsrvinvalid", dest="regsrvinvalid", default=False,
                      action="store_true", help="MasterRegisterServiceInvalid")
    parser.add_option("--regsubinvalid", dest="regsubinvalid", default=False,
                      action="store_true", help="MasterRegisterSubscriberInvalid")
    parser.add_option("--regpubinvalid", dest="regpubinvalid", default=False,
                      action="store_true", help="MasterRegisterPublisherInvalid")

    parser.add_option("--unregsrvinvalid", dest="unregsrvinvalid", default=False,
                      action="store_true", help="MasterUnregisterServiceInvalid")
    parser.add_option("--unregsubinvalid", dest="unregsubinvalid", default=False,
                      action="store_true", help="MasterUnregisterSubscriberInvalid")
    parser.add_option("--unregpubinvalid", dest="unregpubinvalid", default=False,
                      action="store_true", help="MasterUnregisterPublisherInvalid")


    (options, args) = parser.parse_args()
    if options.simple:
        cls = MasterSimpleApi

    elif options.regsrvsuccess:
        cls = MasterRegisterServiceSuccess
    elif options.regpubsuccess:
        cls = MasterRegisterPublisherSuccess
    elif options.regpubtypes:
        cls = MasterRegisterPublisherTypes
    elif options.regsubsimplesuccess:
        cls = MasterRegisterSubscriberSimpleSuccess

    elif options.unregsrvsuccess:
        cls = MasterUnregisterServiceSuccess
    elif options.unregpubsuccess:
        cls = MasterUnregisterPublisherSuccess
    elif options.unregsubsuccess:
        cls = MasterUnregisterSubscriberSuccess     

    elif options.regsrvinvalid:
        cls = MasterRegisterServiceInvalid
    elif options.regpubinvalid:
        cls = MasterRegisterPublisherInvalid
    elif options.regsubinvalid:
        cls = MasterRegisterSubscriberInvalid

    elif options.unregsrvinvalid:
        cls = MasterUnregisterServiceInvalid
    elif options.unregpubinvalid:
        cls = MasterUnregisterPublisherInvalid
    elif options.unregsubinvalid:
        cls = MasterUnregisterSubscriberInvalid

    if not cls:
        parser.error("you must specify a test to run with an [options] flag")
        
    set_node_name(NAME)
    rospy.init_node(NAME, disable_rostime=True)
    import rostest
    rostest.rosrun(PKG, NAME, cls, sys.argv)
