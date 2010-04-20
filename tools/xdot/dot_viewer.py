#!/usr/bin/env python

# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jonathan Bohren 

import roslib; roslib.load_manifest('xdot')
import rospy

import sys
import xdot
import wx

def main():
  import optparse

  parser = optparse.OptionParser(
    usage='\n\t%prog [file]')
  parser.add_option(
    '-f', '--filter',
    type='choice', choices=('dot', 'neato', 'twopi', 'circo', 'fdp'),
    dest='filter', default='dot',
    help='graphviz filter: dot, neato, twopi, circo, or fdp [default: %default]')

  (options, args) = parser.parse_args(sys.argv[1:])
  if len(args) > 1:
    parser.error('incorrect number of arguments')

  app = wx.App()

  frame = xdot.wxxdot.WxDotFrame()
  frame.set_filter(options.filter)

  """Sample mouse event."""
  #def print_cb(item,event):
  #  print "MOUSE_EVENT: "+str(item)
  #frame.widget.register_select_callback(print_cb)

  frame.Show()
  if len(args) >= 1:
    if args[0] == '-':
      frame.set_dotcode(sys.stdin.read())
    else:
      frame.open_file(args[0])

  app.MainLoop()

if __name__ == '__main__':
    main()
