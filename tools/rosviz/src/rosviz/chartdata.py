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
# Revision $Id$

## getBusStats(callerId):
##   return [publishStats, subscribeStats, serviceStats]
##      publishStats: [[topic_name, pubConnectionData]...[topic_nameN, pubConnectionDataN]]
##          pubConnectionData: [connectionId, bytesSent, messageDataSent, connected]* . 
##      subscribeStats: [[topic_name, subConnectionData]...[topic_name, subConnectionData]]
##          subConnectionData: [connectionId, bytesReceived, dropEstimate, connected]* . dropEstimate is -1 if no estimate. 
##      serviceStats: not sure yet, probably akin to [numRequests, bytesReceived, bytesSent] 
## getBusInfo(callerId, connId=0):
##   return [[connectionId1, destinationCallerId1, transport1]... [connectionIdN, destinationCallerIdN, transportN]]
##
##   Maps connectionId from getBusStats into display data
##   If connId is non zero, just return [[connId, destCallerId, trans]] for the matching ID
    
class TopicData(object):
    def __init__(self):
        self.pubnodes = {}
        self.subnodes = {}
    def add_publisher(self, data):
        self.pubnodes[data.caller_id] = data
    def add_subscriber(self, data):
        self.subnodes[data.caller_id] = data

class ConnectionData(object):
    def __init__(self, id, caller_id, topic):
        self.id = id
        self.caller_id = caller_id
        self.topic = topic
        
        self.dest_caller_id = ''
        self.transport = ''
        
        self.bytes_sent = 0
        self.message_data_sent = 0
        self.connected = False

    def setInfo(self, dest_caller_id, transport):
        self.dest_caller_id = dest_caller_id
        self.transport    = transport
        
    def update(self, bytes_sent, message_data_sent, connected):
        self.bytes_sent       = bytes_sent
        self.message_data_sent = message_data_sent
        self.connected       = connected

## Data structure for storing statistics for ROS publish/subscribe
## data for a particular topic on a node
class NodeTopicData(object):
    ## @param nodeName:  caller_id of node being tracked
    ## @param topic_name: name of topic being tracked
    def __init__(self, node_name, topic_name):
        self.node_name = node_name
        self.topic_name = topic_name
        self.pub_data = []
        self.sub_data = []
        
    ## @param cdata: data structure to search for ConnectionData instance
    ## @return ConnectionData: connection data with matching ID
    def _get(self, id, cdata):
        d = [x for x in cdata if x.id == id]
        if d:
            return d[0]
        d = ConnectionData(id, self.node_name, self.topic_name)
        cdata.append(d)
        return d

    def _bytes_sent(self):
        return reduce(lambda x, y: x+y, [d.bytes_sent for d in self.pub_data])
    def _message_data_sent(self):
        return max([d.message_data_sent for d in self.pub_data])
    def _bytes_recvd(self):
        return reduce(lambda x, y: x+y, [d.bytes_recvd for d in self.sub_data])
    def _drop_estimate(self):
        return reduce(lambda x, y: x+y, [d.drop_estimate for d in self.sub_data if d.drop_estimate != -1])

    bytes_sent = property(_bytes_sent, None, None, "Numbers of bytes transmitted to subscribers.")
    bytes_recvd = property(_bytes_recvd, None, None, "Bytes received by all topics.")
    drop_estimate = property(_drop_estimate, None, None, "Number of messages dropped or retransmitted.")
    message_data_sent = property(_message_data_sent, None, None, "Total message data (in bytes).")
                      
    ## Update publish data
    ## @param id int: connection ID
    def pub_data(self, id, bytes_sent, message_data_sent, connected):
        d = self._get(id, self.pub_data)
        d.update(bytes_sent, message_data_sent, connected)

    ## Update subscription data
    ## @param id int: connection ID        
    def sub_data(self, id, bytes_recvd, drop_estimate, connected):
        d = self._get(id, self.pub_data)
        d.update(byteRecvd, drop_estimate, connected)

HEALTH_ERROR = -1
HEALTH_FLAKY =  0
HEALTH_GOOD  =  1

## Data structure for storing statistics about a ROS node
class NodeData(object):
    def __init__(self, name):
        self.name = name
        self.topics = {}
        self.health = HEALTH_GOOD
        
    def _bytes_sent(self):
        return sum([t.bytes_sent for t in self.topics.itervalues()])
    def _message_data_sent(self):
        return sum([d.message_data_sent for d in self.topics.itervalues()])
    def _bytes_recvd(self):
        return sum([d.bytes_recvd for d in self.topics.itervalues()])
    def _drop_estimate(self):
        acc = 0 #manual implementation to avoid double calc on drop_estimate
        for t in topics:
            d = t.drop_estimate
            if d != -1:
                acc += d
        return acc

    bytes_sent  = property(_bytes_sent,  None, None, "Numbers of bytes transmitted to subscribers.")
    bytes_recvd = property(_bytes_recvd, None, None, "Bytes received by all topics.")
    drop_estimate = property(_drop_estimate, None, None, "Number of messages dropped or retransmitted.")
    message_data_sent = property(_message_data_sent, None, None, "Total message data (in bytes).")

    def topic_data(self):
        xmin = 0
        bars = []
        bytes_sent_total = float(self.bytes_sent)
        for t in self.topics.itervalues():
            bytes_sent = t.bytes_sent
            bars.append((xmin, bytes_sent))
            xmin += bytes_sent
        return bytes_sent_total, bars
    
    def bloat_data(self):
        pass

    def _topic(self, topic_name):
        if topic_name in self.topics:
            return self.topics[topic_name]
        else:
            t = NodeTopicData(self.name, topic_name)
            self.topics[topic_name] = t
            return t

    #TODO: definitely wrong as it refers to fields not in this class
    def update(self, rpc_response):
        code, msg, data = rpc_response
        if code != 1:
            self.health = HEALTH_ERROR
        else:
            self.health = HEALTH_GOOD            
        pub_stats, sub_stats, serv_stats = data

        for s in pub_stats:
            topic_name, conns = s
            for id, cbytes_sent, cmessage_data_sent, connected in conns:
                t.pub_data(id, cbytes_sent, cmessage_data_sent)
        for s in sub_stats:
            topic_name, conns = s
            t = self._topic(topic_name)
            for id, cbytesrecd, cdropest, connected in conns:
                d = [x for x in self.pub_data if x.id == id]
                if d:
                    d = d[0]
                else:
                    d = ConnectionData(id, self.name, topic_name)
                    t.sub_data.append(d)
        for s in serv_stats:
            pass
