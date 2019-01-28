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

from distutils.spawn import find_executable
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
    accepted = _ask('\n'.join([' '.join([quote(s) for s in c]) for c in cmds]))
    if accepted:
        _call(cmds, cwd)
    return accepted

def _ask(comment):
    """
    ask user with provided comment. If user responds with y, return True

    :param comment: comment, ``str``
    :return: ``True`` if user responds with y
    """
    sys.stdout.write("Okay to perform:\n\n%s\n(y/n)?\n"%comment)
    while 1:
        input = sys.stdin.readline().strip().lower()
        if input in ['y', 'n']:
            break
    return input == 'y'

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

def _get_disk_usage_by_walking_tree(d):
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(d):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            total_size += os.path.getsize(fp)
    return total_size
    
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
    elif platform.system() == 'Windows':
        total_size = _get_disk_usage_by_walking_tree(d)
        return "Total Size: " + str(total_size) + " " + d
    else:
        raise CleanupException("rosclean is not supported on this platform")
    
def get_disk_usage(d):
    """
    Get disk usage in bytes for directory
    :param d: directory path, ``str``
    :returns: disk usage in bytes (du -b) or (du -A) * 1024, ``int``
    :raises: :exc:`CleanupException` If get_disk_usage() cannot be used on this platform
    """
    if platform.system() == 'Windows':
        return _get_disk_usage_by_walking_tree(d)

    # only implemented on Linux and FreeBSD for now. Should work on OS X but need to verify first (du is not identical)
    cmd = None
    unit = 1
    du = find_executable('du')
    if du is not None:
        if platform.system() == 'Linux':
            cmd = [du, '-sb', d]
        elif platform.system() == 'FreeBSD':
            cmd = [du, '-skA', d]
            unit = 1024
        try:
            # detect BusyBox du command by following symlink
            if os.path.basename(os.readlink(du)) == 'busybox':
                cmd = [du, '-sk', d]
                unit = 1024
        except OSError:
            # readlink raises OSError if the target is not symlink
            pass

    if cmd is None:
        raise CleanupException("rosclean is not supported on this platform")
    try:
        return int(subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0].split()[0]) * unit
    except:
        raise CleanupException("rosclean is not supported on this platform")

def _sort_file_by_oldest(d):
    """
    Get files and directories in specified path sorted by last modified time
    :param d: directory path, ```str```
    :return:  a list of files and directories sorted by last modified time (old first), ```list```
    """
    files = os.listdir(d)
    files.sort(key=lambda f: os.path.getmtime(os.path.join(d, f)))
    return files

def _rosclean_cmd_purge(args):
    dirs = _get_check_dirs()

    for d, label in dirs:
        if not args.size:
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
        else:
            files = _sort_file_by_oldest(d)
            log_size = get_disk_usage(d)
            if log_size <= args.size * 1024 * 1024:
                print("Directory size of %s is %d MB which is already below the requested threshold of %d MB."%(label, log_size / 1024 / 1024, args.size))
                continue
            print("Purging %s until directory size is at most %d MB (currently %d MB)."%(label, args.size, log_size / 1024 / 1024))
            if not args.y:
                print("PLEASE BE CAREFUL TO VERIFY THE COMMAND BELOW!")
                if not _ask("Purge some of old logs in %s"%d):
                    return
            for f in files:
                if log_size <= args.size * 1024 * 1024:
                    break
                path = os.path.join(d, f)
                log_size -= get_disk_usage(path)
                if platform.system() == 'Windows':
                    cmds = [['rd', '/s', '/q', path]]
                else:
                    cmds = [['rm', '-rf', path]]
                try:
                    _call(cmds)
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
    parser_purge.add_argument('--size', action='store', default=None, type=int, help='Maximum total size in MB to keep when deleting old files')
    args = parser.parse_args(argv[1:])
    args.func(args)

if __name__ == '__main__':
    rosclean_main()
