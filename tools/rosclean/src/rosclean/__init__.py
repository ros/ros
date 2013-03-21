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

from __future__ import print_function

__version__ = '1.7.0'

import argparse
import os
import sys
import platform
import subprocess

import rospkg

class CleanupException(Exception): pass

def _ask_and_call(cmds, cwd=None):
    """
    Pretty print cmds, ask if they should be run, and if so, runs
    them using _call().

    :param cmds: a list of commands executed one after another, ``list``
    :param cwd: (optional) set cwd of command that is executed, ``str``
    :returns: ``True`` if cmds were run.
    """
    # Pretty-print a string version of the commands
    def quote(s):
        return '"%s"'%s if ' ' in s else s
    sys.stdout.write("Okay to execute:\n\n%s\n(y/n)?\n"%('\n'.join([' '.join([quote(s) for s in c]) for c in cmds])))
    while 1:
        input = sys.stdin.readline().strip().lower()
        if input in ['y', 'n']:
            break
    accepted = input == 'y'
    if accepted:
        _call(cmds, cwd)
    return accepted

def _call(cmds, cwd=None):
    """
    Runs cmds using subprocess.check_call.

    :param cmds: a list of commands executed one after another, ``list``
    :param cwd: (optional) set cwd of command that is executed, ``str``
    """
    for c in cmds:
        if cwd:
            subprocess.check_call(c, cwd=cwd)
        else:
            subprocess.check_call(c)

def _usage():
    print("""Usage: rosclean <command>

Commands:
\trosclean check\tCheck usage of log files
\trosclean purge\tRemove log files
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))
    
def _get_check_dirs():
    home_dir = rospkg.get_ros_home()
    log_dir = rospkg.get_log_dir()
    dirs = [ (log_dir, 'ROS node logs'),
             (os.path.join(home_dir, 'rosmake'), 'rosmake logs')]
    return [x for x in dirs if os.path.isdir(x[0])]
    
def _rosclean_cmd_check(args):
    dirs = _get_check_dirs()
    for d, label in dirs:
        desc = get_human_readable_disk_usage(d)
        print("%s %s"%(desc, label))

def get_human_readable_disk_usage(d):
    """
    Get human-readable disk usage for directory

    :param d: directory path, ``str`
    :returns: human-readable disk usage (du -h), ``str``
    """
    # only implemented on Linux and FreeBSD for now. Should work on OS X but need to verify first (du is not identical)
    if platform.system() in ['Linux', 'FreeBSD']:
        try:
            return subprocess.Popen(['du', '-sh', d], stdout=subprocess.PIPE).communicate()[0].split()[0]
        except:
            raise CleanupException("rosclean is not supported on this platform")
    else:
        raise CleanupException("rosclean is not supported on this platform")
    
def get_disk_usage(d):
    """
    Get disk usage in bytes for directory
    :param d: directory path, ``str``
    :returns: disk usage in bytes (du -b) or (du -A) * 1024, ``int``
    :raises: :exc:`CleanupException` If get_disk_usage() cannot be used on this platform
    """
    # only implemented on Linux and FreeBSD for now. Should work on OS X but need to verify first (du is not identical)
    if platform.system() == 'Linux':
        try:
            return int(subprocess.Popen(['du', '-sb', d], stdout=subprocess.PIPE).communicate()[0].split()[0])
        except:
            raise CleanupException("rosclean is not supported on this platform")
    elif platform.system() == 'FreeBSD':
        try:
            return int(subprocess.Popen(['du', '-sA', d], stdout=subprocess.PIPE).communicate()[0].split()[0]) * 1024
        except:
            raise CleanupException("rosclean is not supported on this platform")
    else:
        raise CleanupException("rosclean is not supported on this platform")

def _rosclean_cmd_purge(args):
    dirs = _get_check_dirs()

    for d, label in dirs:
        print("Purging %s."%label)
        cmds = [['rm', '-rf', d]]
        try:
            if args.y:
                _call(cmds)
            else:
                print("PLEASE BE CAREFUL TO VERIFY THE COMMAND BELOW!")
                _ask_and_call(cmds)
        except:
            print("FAILED to execute command", file=sys.stderr)

def rosclean_main(argv=None):
    if argv is None:
        argv = sys.argv
    parser = argparse.ArgumentParser(prog='rosclean')
    subparsers = parser.add_subparsers()#help='sub-command help')
    parser_check = subparsers.add_parser('check', help='Check usage of log files')
    parser_check.set_defaults(func=_rosclean_cmd_check)
    parser_purge = subparsers.add_parser('purge', help='Remove log files')
    parser_purge.set_defaults(func=_rosclean_cmd_purge)
    parser_purge.add_argument('-y', action='store_true', default=False, help='CAUTION: automatically confirms all questions to delete files')
    args = parser.parse_args(argv[1:])
    args.func(args)

if __name__ == '__main__':
    rosclean_main()
