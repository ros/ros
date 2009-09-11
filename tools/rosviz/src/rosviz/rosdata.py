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
import roslib;roslib.load_manifest('rosviz')

import threading
import rospy
from std_msgs.msg import *
import chartdata

def exemplar():
    xmax   = 800
    ymax   = 20
    labels = ['/axiscam', '/foo/bar']
    xdata  = [ [ (0, 500), (500, 200),  (700, 100)], [ (0, 410), (410, 20), (430, 10), ] ]
    ycurr  = 19
    labels = [x for x in reversed(labels)]
    for x in xdata:
        barcol = colors[:len(xdata)+1]
        print barcol
        ax.broken_barh(x , (ycurr, 1), facecolors=barcol, edgecolors=barcol, **axprops)
        ycurr -= 1


def exemplar2():
    nodes = _nodeData.values()
    ymax  = min(20, len(nodes))
    ycurr = ymax - 1
    for n in nodes:
        labels.insert(0, n.name)
        bytes_sent_total, xdata = n.topic_data()
        if bytes_sent_total > xmax:
            xmax = bytes_sent_total

        barcol = colors[:len(xdata)]
        ax.broken_barh(xdata , (ycurr, 1), facecolors=barcol, edgecolors=barcol, **axprops)
        ycurr -= 1
            
_node_data_lock = threading.Lock()
_node_data = {}
_plots = {}
def update(self, *args):
    if 1:
        return
    ax = axes['node-by-topic']
    try:
        _node_data_lock.acquire()
        for n in _node_data.itervalues():
            name = n.name
            if not name in _plots:
                _plots[name] = ax.broken_bar(n.topic_data(), yrange, label=name, picker=True)
            else:
                _plots[name].set_xdata(n.topic_data())
    finally:
        _node_data_lock.release()
    app.update()

## list of current topics. topics cannot be deleted once they are created
_topics = set()
## list of current nodes
_nodes = set()
## list of current services
_services = set()

import time, itertools
def query_nodes():
    global _topics, nodes, services
    
    caller_id = '/rosviz'
    start = time.time()
    master= _getMaster()
    topicsupdate   = []
    nodesupdate    = []
    servicesupdate = []    
    
    #TODO: do we want a stripped down getNodes() API?
    code, msg, state = master.getSystemState(caller_id)
    if code != 1:
        raise Exception("unable to query system state from master: %s"%msg)
    pubstats, substats, srvstats = state
    for topic, list in itertools.chain(pubstats, substats):
        topicsupdate.append(topic)
        for l in list:
            nodesupdate.append(l)
    for service, list in srvstats:
        servicesupdate.append(topic)
        for l in list:
            nodesupdate.append(l)
    
    # process the data
    topicsupdate    = set(topicsupdate)
    addedtopics     = topicsupdate - topics

    nodesupdate     = set(nodesupdate)
    addednodes      = nodesupdate - nodes
    deletednodes    = nodes - nodesupdate

    servicesupdate  = set(servicesupdate)
    addedservices   = servicesupdate - services
    deletedservices = services - servicesupdate

    _nodes    = nodesupdate
    _topics   = topicsupdate
    _services = servicesupdate    

    #print "ELAPSED[query_nodes]: %ss"%(time.time() - start)
    
NAME = 'rosviz'
## locate ROS master or throw Exception if it cannot be found
## @raise RVException if master cannot be located
def _master():
    import roslib.scriptutil as scriptutil 
    master = scriptutil.getMaster()
    if master is None:
        raise RVException("Cannot locate master")
    return master
    
def init():
    #verify master can be found
    _master()

def rosloop():
    master = _master()
    #TODO: poll master and nodes
        
