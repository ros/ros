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

import rospy
import random
import genmsg.msgs
import genpy.dynamic

def get_sub_defs(msg_fqn, msg_txt):
    def_dict = {}
    defs = msg_txt.split("\n" + "="*80 + "\n")
    def_dict[msg_fqn] = defs[0]
    for d in defs[1:]:
        lines = d.splitlines()
        if not lines[0].startswith("MSG: "):
            raise Exception("Invalid sub definition!")
        type = lines[0][5:].strip()
        def_txt = "\n".join(lines[1:])
        def_dict[type] = def_txt
    return def_dict

class RandomMsgGen(object):
    def randstr(self, length=0):
        if length == 0:
            length = self.rand.randint(3,10)
        return ''.join([chr(self.rand.randint(ord('a'), ord('z'))) for i in xrange(length)])

    def __init__(self, seed, num_topics, duration):
        self.message_defs = {}
        self.message_dict = {}
        self.topic_dict = {}
        self.duration = duration
        self.output = []

        self.rand = random.Random(seed)

        for i in xrange(num_topics):
            msg_pkg = self.randstr()
            msg_name = self.randstr()
            msg_fqn = "%s/%s"%(msg_pkg,msg_name)
            msg_fields = []
            
            msg_def = ""
            msg_sub_defs = {}
            
            for j in xrange(self.rand.randint(3,5)):
                field_name = self.randstr()
                field_type = self.rand.choice(genmsg.msgs.BUILTIN_TYPES + self.message_defs.keys())
                field_array = self.rand.choice(5*[""]+["[]","[%d]"%self.rand.randint(1,10)])

                if (field_type not in genmsg.msgs.BUILTIN_TYPES):
                    tmp = get_sub_defs(field_type, self.message_defs[field_type])
                    for (sm_type, sm_def) in tmp.iteritems():
                        msg_sub_defs[sm_type] = sm_def

                msg_def = msg_def + "%s%s %s\n"%(field_type, field_array, field_name)

            for (t,d) in msg_sub_defs.iteritems():
                msg_def = msg_def + "\n" + "="*80 + "\n"
                msg_def = msg_def + "MSG: %s\n"%(t)
                msg_def = msg_def + d

            self.message_defs[msg_fqn] = msg_def

            topic_name = self.randstr()

            self.message_dict[msg_fqn] = genpy.dynamic.generate_dynamic(msg_fqn, msg_def)[msg_fqn] 
            self.topic_dict[topic_name] = self.message_dict[msg_fqn]

        time = 0.0
        while time < duration:
            topic = self.rand.choice(self.topic_dict.keys())
            msg_inst = self.rand_value(self.topic_dict[topic]._type)
            self.output.append((topic, msg_inst, time))
            time = time + self.rand.random()*.01

    def topics(self):
        return self.topic_dict.iteritems()

    def messages(self):
        for m in self.output:
            yield m

    def message_count(self):
        return len(self.output)
    
    def rand_value(self, field_type):
        if field_type == 'bool':
            return self.rand.randint(0,1)
        elif field_type == 'byte':
            return self.rand.randint(0,2**7-1)
        elif field_type == 'int8':
            return self.rand.randint(-2**7,2**7-1)
        elif field_type == 'int16':
            return self.rand.randint(-2**15,2**15-1)
        elif field_type == 'int32':
            return self.rand.randint(-2**31,2**31-1)
        elif field_type == 'int64':
            return self.rand.randint(-2**63,2**63-1)
        elif field_type == 'char':
            return self.rand.randint(0,2**8-1)
        elif field_type == 'uint8':
            return self.rand.randint(0,2**8-1)
        elif field_type == 'uint16':
            return self.rand.randint(0,2**16-1)
        elif field_type == 'uint32':
            return self.rand.randint(0,2**32-1)
        elif field_type == 'uint64':
            return self.rand.randint(0,2**64-1)
        elif field_type == 'float32':
            return self.rand.random()
        elif field_type == 'float64':
            return self.rand.random()
        elif field_type == 'string':
            return self.randstr(100)
        elif field_type == 'duration':
            return rospy.Duration.from_sec(self.rand.random())
        elif field_type == 'time':
            return rospy.Time.from_sec(self.rand.random()*1000)
        elif field_type.endswith(']'): # array type
            base_type, is_array, array_len = genmsg.msgs.parse_type(field_type)
            
            if array_len is None:
                array_len = self.rand.randint(1,100)

            # Make this more efficient rather than depend on recursion inside the array check
            if field_type == 'bool':
                return [ self.rand.randint(0,1) for i in xrange(0,array_len) ]
            elif field_type == 'byte':
                return [ self.rand.randint(-2**7,2**7-1) for i in xrange(0,array_len) ]
            elif field_type == 'int8':
                return [ self.rand.randint(-2**7,2**7-1) for i in xrange(0,array_len) ]
            elif field_type == 'int16':
                return [ self.rand.randint(-2**15,2**15-1) for i in xrange(0,array_len) ]
            elif field_type == 'int32':
                return [ self.rand.randint(-2**31,2**31-1) for i in xrange(0,array_len) ]
            elif field_type == 'int64':
                return [ self.rand.randint(-2**63,2**63-1) for i in xrange(0,array_len) ]
            elif field_type == 'char':
                return [ self.rand.randint(0,2**8-1) for i in xrange(0,array_len) ]
            elif field_type == 'uint8':
                return [ self.rand.randint(0,2**8-1) for i in xrange(0,array_len) ]
            elif field_type == 'uint16':
                return [ self.rand.randint(0,2**16-1) for i in xrange(0,array_len) ]
            elif field_type == 'uint32':
                return [ self.rand.randint(0,2**32-1) for i in xrange(0,array_len) ]
            elif field_type == 'uint64':
                return [ self.rand.randint(0,2**64-1) for i in xrange(0,array_len) ]
            elif field_type == 'float32':
                return [ self.rand.random() for i in xrange(0,array_len) ]
            elif field_type == 'float64':
                return [ self.rand.random() for i in xrange(0,array_len) ]
            elif field_type == 'string':
                return [ self.randstr(100) for i in xrange(0,array_len) ]
            elif field_type == 'duration':
                return [ rospy.Duration.from_sec(self.rand.random()) for i in xrange(0,array_len) ]
            elif field_type == 'time':
                return [ rospy.Time.from_sec(self.rand.random()*1000) for i in xrange(0,array_len) ]
            else:
                return [ self.rand_value(base_type) for i in xrange(0,array_len) ]

        else:
            msg_class = self.message_dict[field_type]
            msg_inst = msg_class()
            for s in msg_inst.__slots__:
                ind = msg_inst.__slots__.index(s)
                msg_inst.__setattr__(s,self.rand_value(msg_inst._slot_types[ind]))
            return msg_inst
