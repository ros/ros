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
# $Author$

import os
import sys
import logging

from roslib.rosenv import get_ros_root, ROS_LOG_DIR

## @param environ dict: environment dictionary (defaults to os.environ)
## @return str: path to use use for log file directory
def get_log_dir(environ=os.environ):
    if ROS_LOG_DIR in environ:
        log_dir = os.environ[ROS_LOG_DIR]
    else:
        # if ROS_LOG_DIR is not set and ROS_ROOT/log is not writeable, try $HOME/.ros/log instead
        log_dir = os.path.join(get_ros_root(required=True), 'log')
        #1222: log files should go to $HOME/.ros/logs if ROS_ROOT/log is not writable
        if not os.access(log_dir, os.W_OK):
            user_home_dir = os.path.expanduser('~') #slightly more robust than $HOME
            log_dir = os.path.join(user_home_dir, '.ros', 'logs')
    return log_dir
    
## configure Python logging package to send log files to ROS-specific log directory
## @param logname str: name of logger
## @param filename str: filename to log to. If not set, a log filename
## will be generated using \a logname
## @param additional [str]: additional log names to attach to same log handler
## @return str: log file name
def configure_logging(logname, level=logging.INFO, filename=None, additional=None, environ=os.environ):
    import logging.handlers
    
    logname = logname or 'unknown'
    log_dir = get_log_dir(environ)
    
    # if filename is not explicitly provided, generate one using logname
    if not filename:
        log_filename = os.path.join(log_dir, '%s-%s.log'%(logname, os.getpid()))
    else:
        log_filename = os.path.join(log_dir, filename)

    logfile_dir = os.path.dirname(log_filename)
    if not os.path.exists(logfile_dir):
        try:
            makedirs_with_parent_perms(logfile_dir)
        except OSError:
            # cannot print to screen because command-line tools with output use this
            print >> sys.stderr, "WARNING: cannot create log directory [%s]. Please set %s to a writable location."%(logfile_dir, ROS_LOG_DIR)
            return None
    elif os.path.isfile(logfile_dir):
        raise Exception("Cannot save log files: file [%s] is in the way"%logfile_dir)

    handler = logging.handlers.RotatingFileHandler(
        log_filename, maxBytes=100000000, backupCount=10)
    formatter = logging.Formatter("[%(levelname)s] %(asctime)s: %(message)s")
    handler.setFormatter(formatter)
    additional = additional or []
    for n in [logname] + additional:
        logger = logging.getLogger(n)
        logger.setLevel(level)
        logger.addHandler(handler)
    return log_filename

## Create the directory \a p using the permissions of the nearest
## (existing) parent directory. This is useful for logging, where a
## root process sometimes has to log in the user's space.
## @param p str: directory to create
def makedirs_with_parent_perms(p):
    p = os.path.abspath(p)
    parent = os.path.dirname(p)
    # recurse upwards, checking to make sure we haven't reached the
    # top
    if not os.path.exists(p) and p and parent != p:
        makedirs_with_parent_perms(parent)
        s = os.stat(parent)
        os.mkdir(p)
        os.chown(p, s.st_uid, s.st_gid)
        os.chmod(p, s.st_mode)    
