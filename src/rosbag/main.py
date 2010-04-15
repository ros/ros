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

import sys
import UserDict

from record_cmd import record_cmd
from play_cmd   import play_cmd
from info_cmd   import info_cmd
from check_cmd  import check_cmd
from fix_cmd    import fix_cmd
from filter_cmd import filter_cmd

class RosbagCmds(UserDict.UserDict):
    def __init__(self):
        UserDict.UserDict.__init__(self)

        self['help'] = self.help_cmd

    def get_valid_cmds(self):
        str = "Valid commands:\n"
        for k in sorted(self.keys()):
            str += "    * %s\n" % k

        return str

    def help_cmd(self,argv):
        argv = [a for a in argv if a != '-h' and a != '--help']

        if len(argv) == 0:
            print 'Usage: rosbag <command> [options] [args]'
            print ''
            print self.get_valid_cmds()
            print 'For more information please visit the rosbag wiki page: http://code.ros.org/wiki/rosbag'
            print ''
        else:
            cmd = argv[0]
            if cmd in self:
                self[cmd](['-h'])
            else:
                print >> sys.stderr, 'Invalid command: %s' % cmd
                print >> sys.stderr, ''
                print >> sys.stderr, self.get_valid_cmds()

def rosbagmain(argv=None):
    cmds = RosbagCmds()
    cmds['record'] = record_cmd
    cmds['play']   = play_cmd
    cmds['info']   = info_cmd
    cmds['check']  = check_cmd
    cmds['fix']    = fix_cmd
    cmds['filter'] = filter_cmd

    if argv is None:
        argv = sys.argv

    if '-h' in argv or '--help' in argv:
        argv = [a for a in argv if a != '-h' and a != '--help']
        argv.insert(1, 'help')

    if len(argv) > 1:
        cmd = argv[1]
    else:
        cmd = 'help'

    if cmd in cmds:
        cmds[cmd](argv[2:])
    else:
        cmds['help']([cmd])
