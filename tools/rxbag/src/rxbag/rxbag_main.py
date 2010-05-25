#!/usr/bin/env python
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

"""
The main entry-point to rxbag.
"""

PKG = 'rxbag'
import roslib; roslib.load_manifest(PKG)
import rospy

import optparse
import sys
import threading
import time

# Ensure wxPython version >= 2.8, and install hotfix for 64-bit Cairo support for wxGTK
import wxversion
WXVER = '2.8'
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    print >> sys.stderr, 'This application requires wxPython version %s' % WXVER
    sys.exit(1)
import wx
import wx.lib.wxcairo
if 'wxGTK' in wx.PlatformInfo:
    # Workaround for 64-bit systems
    import ctypes
    gdkLib = wx.lib.wxcairo._findGDKLib()
    gdkLib.gdk_cairo_create.restype = ctypes.c_void_p

import rxbag_app

def run(options, args):
    app = rxbag_app.RxBagApp(options, args)
    app.MainLoop()

    rospy.signal_shutdown('GUI shutdown')

def rxbag_main():
    # Parse command line for input files and options
    usage = "usage: %prog [options] BAG_FILE1 [BAG_FILE2 ...]"
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('-s', '--start',         dest='start',   default=0.0,   action='store', type='float', help='start SEC seconds into the bag files', metavar='SEC')
    parser.add_option(      '--record',        dest='record',  default=False, action='store_true',          help='record to a bag file')
    parser.add_option('-a', '--all',           dest='all',     default=False, action='store_true',          help='record all topics')
    parser.add_option('-e', '--regex',         dest='regex',   default=False, action="store_true",          help='match topics using regular expressions')
    parser.add_option('-o', '--output-prefix', dest='prefix',  default=None,  action="store",               help='prepend PREFIX to beginning of bag name (name will always end with date stamp)')
    parser.add_option('-O', '--output-name',   dest='name',    default=None,  action="store",               help='record to bag with name NAME.bag')
    parser.add_option('-l', '--limit',         dest='limit',   default=0,     action="store", type='int',   help='only record NUM messages on each topic', metavar='NUM')
    parser.add_option(      '--profile',       dest='profile', default=False, action='store_true',          help='profile and write results to rxbag.prof [advanced]')

    options, args = parser.parse_args(sys.argv[1:])

    if len(args) == 0:
        if options.record:
            if not options.all:
                parser.error('You must specify topics to record when recording (or specify --all).')
        else:
            parser.error('You must specify at least one bag file to view.')
            
    if options.prefix and options.name:
        parser.error('Can\'t set both prefix and name.')

    if options.profile:
        import cProfile
        cProfile.runctx('run(options, args)', globals(), locals(), 'rxbag.prof')
    else:
        run(options, args)
