# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
# Revision $Id: rlutil.py 9911 2010-06-02 21:43:42Z kwc $

import os
import sys
import platform
import subprocess

import roslib.rosenv
from roslib.scriptutil import ask_and_call

import roslib.exceptions
class ROSCleanException(roslib.exceptions.ROSLibException): pass

def _usage():
    print """Usage: rosclean <command>

Commands:
\trosclean check\tCheck usage of log files
\trosclean purge\tRemove log files
"""
    sys.exit(os.EX_USAGE)
    
def _get_check_dirs():
    home_dir = roslib.rosenv.get_ros_home()
    log_dir = roslib.rosenv.get_log_dir()
    dirs = [ (log_dir, 'ROS node logs'),
             (os.path.join(home_dir, 'rosmake'), 'rosmake logs')]
    return [x for x in dirs if os.path.isdir(x[0])]
    
def _rosclean_cmd_check(argv):
    dirs = _get_check_dirs()
    for d, label in dirs:
        desc = get_human_readable_disk_usage(d)
        print "%s %s"%(desc, label)

def get_human_readable_disk_usage(d):
    """
    Get human-readable disk usage for directory
    @param d: directory path
    @type  d: str
    @return: human-readable disk usage (du -h)
    @rtype: str
    """
    # only implemented on Linux for now. Should work on OS X but need to verify first (du is not identical)
    if platform.system() == 'Linux':
        try:
            return subprocess.Popen(['du', '-sh', d], stdout=subprocess.PIPE).communicate()[0].split()[0]
        except:
            raise ROSCleanException("rosclean is not supported on this platform")
    else:
        raise ROSCleanException("rosclean is not supported on this platform")
    
def get_disk_usage(d):
    """
    Get disk usage in bytes for directory
    @param d: directory path
    @type  d: str
    @return: disk usage in bytes (du -b)
    @rtype: int
    @raise ROSCleanException: if get_disk_usage() cannot be used on this platform
    """
    # only implemented on Linux for now. Should work on OS X but need to verify first (du is not identical)
    if platform.system() == 'Linux':
        try:
            return subprocess.Popen(['du', '-sb', d], stdout=subprocess.PIPE).communicate()[0].split()[0]
        except:
            raise ROSCleanException("rosclean is not supported on this platform")
    else:
        raise ROSCleanException("rosclean is not supported on this platform")

def _rosclean_cmd_purge(argv):
    dirs = _get_check_dirs()

    for d, label in dirs:
        print "Purging %s.\nPLEASE BE CAREFUL TO VERIFY THE COMMAND BELOW!"%label
        cmds = [['rm', '-rf', d]]
        try:
            ask_and_call(cmds)
        except:
            print >> sys.stderr, "FAILED to execute command"

def rosclean_main(argv=None):
    if argv == None:
        argv = sys.argv
    if len(argv) < 2:
        _usage()
    command = argv[1]
    if command == 'check':
        _rosclean_cmd_check(argv)
    elif command == 'purge':
        _rosclean_cmd_purge(argv)
    else:
        _usage()

if __name__ == '__main__':
    rosclean_main()
