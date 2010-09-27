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
#
# Revision $Id$

WXVER = '2.8' 
import wxversion 
if wxversion.checkInstalled(WXVER): 
  wxversion.select(WXVER) 
else: 
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER) 
  sys.exit(1) 
import wx

import roslib.scriptutil
import rospy

import rxtools.rxplot

# TODO: poll for rospy.is_shutdown()
def rxplot_main():
    from optparse import OptionParser
    parser = OptionParser(usage="usage: rxplot [options] /topic/field1 [/topic/field2]")
    parser.add_option("-l", "--legend", type="string",
                      dest="legend", default='',
                      help="set the legend")
    parser.add_option("-p", "--period", type="int",
                      dest="period", default=5,
                      help="set period displayed in window")
    parser.add_option("-m", "--marker", type="string",
                      dest="marker", default='None',
                      help="set the line marker, e.g. o, ^, .")
    parser.add_option("-t", "--title", type="string",
                      dest="title", default='RXPlot',
                      help="set the title")
    parser.add_option("-b", "--buffer", type="int",
                      dest="buffer", default=-1,
                      help="set size of buffer in seconds (default of -1 keeps all data)")
    options, topics = parser.parse_args()

    if not topics:
        parser.error("Please specify a topic field")
        
    topic_list = []
    for t in topics:
        # c_topics is the list of topics to plot together
        c_topics = []
        # compute combined topic list, t == '/foo/bar1,/baz/bar2'
        for sub_t in [x for x in t.split(',') if x]:
            # check for shorthand '/foo/field1:field2:field3'
            if ':' in sub_t:
                base = sub_t[:sub_t.find(':')]
                # the first prefix includes a field name, so save then strip it off
                c_topics.append(base)
                if not '/' in base:
                    parser.error("%s must contain a topic and field name"%sub_t)
                base = base[:base.rfind('/')]

                # compute the rest of the field names
                fields = sub_t.split(':')[1:]
                c_topics.extend(["%s/%s"%(base, f) for f in fields if f])
            else:
                c_topics.append(sub_t)

        # #1053: resolve command-line topic names
        c_topics = [roslib.scriptutil.script_resolve_name('rxplot', n) for n in c_topics]

        topic_list.append(c_topics)

    #flatten for printing
    print_topic_list = []
    for l in topic_list:
        if type(l) == list:
            print_topic_list.extend(l)
        else:
            print_topic_list.append(l)

    # constrain the number of things we plot by len(COLORS). this is an artificial
    # performance hack.
    if len(print_topic_list) > len(rxtools.rxplot.COLORS):
        parser.error("maximum number of topics that can be plotted is %d"%len(rxtools.rxplot.COLORS))
    print "plotting topics", ', '.join(print_topic_list)

    rxtools.rxplot.rxplot_app(topic_list, options)
