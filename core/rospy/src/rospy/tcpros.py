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
"""
TCPROS connection protocol.
"""
## TCPROS connection protocol.
#  http://pr.willowgarage.com/wiki/ROS/TCPROS
# 
#  TCPROS sets up a TCP/IP server socket on the source machine that
#  waits for incoming connections. When it receives an incoming
#  connection, it parses the header sent along the connection to
#  determine which topic it corresponds to. The format of this header
#  is:
# 
#  HEADER-LENGTH (32-bit int)
#  TOPIC-NAME (string identifier)
#  '\n' (newline character)
#  MSG-MD5SUM
#
#  The MSG-MD5SUM is an md5sum of the full text of the .msg file. As
#  such, it will signal a version change even for compatible changes to
#  the file.

import rospy.tcpros_base
import rospy.tcpros_pubsub
import rospy.tcpros_service

_handler = rospy.tcpros_pubsub.TCPROSHandler()

DEFAULT_BUFF_SIZE = rospy.tcpros_base.DEFAULT_BUFF_SIZE
def init_tcpros():
    server = rospy.tcpros_base.init_tcpros_server()
    server.topic_connection_handler = _handler.topic_connection_handler
    server.service_connection_handler = rospy.tcpros_service.service_connection_handler

def get_tcpros_handler():
    return _handler
