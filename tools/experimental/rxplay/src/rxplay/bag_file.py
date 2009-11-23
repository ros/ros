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

PKG = 'rxplay'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord
import os

## A utility class for reading ROS messages (optionally using an index)
class BagFile:
    def __init__(self, path):
        self.path = path

        self._file     = None
        self._next_msg = None

    ## Iterate through the raw messages
    def raw_messages(self):
        if not self._file and not self._open():
            return
        f, next_msg = self._file, self._next_msg

        while True:
            try:
                pos = f.tell()                   
                topic, msg, t = next_msg(f, True)        
                if msg == None or rospy.is_shutdown():
                    break
                yield pos, topic, msg, t
                
            except rosrecord.ROSRecordException, e:
                rospy.logerr('ROSRecordException: couldn\'t read msg at pos %d - %s' % (pos, e)) 
            except IOError:
                rospy.logerr('IOError: couldn\'t read msg at pos %d' % pos)
            except KeyError:
                rospy.logerr('KeyError: couldn\'t read msg at pos %d' % pos)
                
        self._close()

    ## Returns (datatype, message, timestamp)
    def load_message(self, pos, index=None):
        msgs = list(self.load_messages([pos], index))
        if len(msgs) == 0:
            return (None, None, None)
        
        return msgs[0]

    ## Returns iterator of (datatype, message, timestamp)
    def load_messages(self, positions, index=None):
        if not self._file and not self._open():
            return
        f, next_msg = self._file, self._next_msg

        try:
            for pos in positions:
                f.seek(pos)

                topic, raw_msg, t = next_msg(f, True)
                
                if raw_msg:
                    (datatype, message_data, md5, bag_pos, pytype) = raw_msg

                    msg = pytype()
                    msg.deserialize(message_data)

                    if index:
                        index.datatype_defs_read.add(datatype)

                    yield (datatype, msg, t)

        except rosrecord.ROSRecordException, e:
            rospy.logerr('ROSRecordException: couldn\'t read %d - %s' % (pos, e)) 
        except IOError:
            rospy.logerr('IOError: couldn\'t read %d' % pos)
        except KeyError:
            rospy.logerr('KeyError: couldn\'t read %d' % pos)

    def read_datatype_defs(self, index):
        if not self._file and not self._open():
            return
        f, next_msg = self._file, self._next_msg

        index.datatype_defs_read = set()

        try:
            for datatype, pos in index._data.datatype_first_pos.items():
                f.seek(pos)

                topic, raw_msg, t = next_msg(f, True)

                if raw_msg:
                    (datatype, message_data, md5, bag_pos, pytype) = raw_msg

                    msg = pytype()
                    msg.deserialize(message_data)

                    index.datatype_defs_read.add(datatype)

        except rosrecord.ROSRecordException, e:
            rospy.logerr('ROSRecordException: couldn\'t read %d - %s' % (pos, e)) 
        except IOError:
            rospy.logerr('IOError: couldn\'t read %d' % pos)
        except KeyError:
            rospy.logerr('KeyError: couldn\'t read %d' % pos)

    def _open(self):
        # TODO: error handling
        self._file, v = rosrecord.open_log_file(self.path)
        
        self._next_msg = {
            rosrecord.HEADER_V1_1 : rosrecord.next_msg_v1_1,
            rosrecord.HEADER_V1_2 : rosrecord.next_msg_v1_2
        }[v]
        
        return True
        
    def _close(self):
        if not self._file:
            return

        self._file.close()
        self._file     = None
        self._next_msg = None
