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
# Copyright (c) 2008, Willow Garage, Inc.
# Revision $Id$

import getopt
import logging
import os
import string
import sys
import time

import rospy

## Command-line handler for zenmaster

# Environment variables used to configure master/slave

from roslib.rosenv import ROS_ROOT, ROS_MASTER_URI, ROS_HOSTNAME, ROS_NAMESPACE, ROS_PACKAGE_PATH, ROS_LOG_DIR

ENV_DOC = {
    ROS_ROOT: "Directory of ROS installation to use",
    ROS_PACKAGE_PATH: "Paths to search for additional ROS packages",    
    ROS_LOG_DIR: "Directory to write log files to",
    ROS_NAMESPACE: "Namespace to place node in", #defined in names.py
    ROS_MASTER_URI: "Default URI of ROS central server",
    ROS_HOSTNAME: "address to bind to",    
    }
ENV_VARS = ENV_DOC.keys()

ZENMASTER_USAGE_ENV = "Environment variables:\n"+'\n'.join([" * %s: %s"%(k,v) for (k,v) in ENV_DOC.iteritems()])

ZENMASTER_USAGE = """
usage: %(progname)s [-p node-port] 

Flags:
 -p\tOverride port environment variable
"""+ZENMASTER_USAGE_ENV

def zenmaster_usage(progname):
    print ZENMASTER_USAGE%vars()

def zenmaster_main(argv=sys.argv, stdout=sys.stdout, environ=os.environ):
    import optparse
    parser = optparse.OptionParser(usage="usage: zenmaster [options]")
    parser.add_option("--core",
                      dest="core", action="store_true", default=False,
                      help="run as core")
    parser.add_option("-p", "--port", 
                      dest="port", default=0,
                      help="override port", metavar="PORT")
    options, args = parser.parse_args(argv[1:])

    # only arg that zenmaster supports is __log remapping of logfilename
    for arg in args:
        if not arg.startswith('__log:='):
            parser.error("unrecognized arg: %s"%arg)
    rospy.core.configure_logging('zenmaster')    
    
    port = rospy.init.DEFAULT_MASTER_PORT
    if options.port:
        port = string.atoi(options.port)

    if not options.core:
        print """


ACHTUNG WARNING ACHTUNG WARNING ACHTUNG
WARNING ACHTUNG WARNING ACHTUNG WARNING


Standalone zenmaster has been deprecated, please use 'roscore' instead


ACHTUNG WARNING ACHTUNG WARNING ACHTUNG
WARNING ACHTUNG WARNING ACHTUNG WARNING


"""

    logger = logging.getLogger("rospy")
    logger.info("initialization complete, waiting for shutdown")
    try:
        logger.info("Starting ROS Master Node")
        rospy.init.start_master(environ, port)
        while not rospy.is_shutdown():
            time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info("keyboard interrupt, will exit")
        rospy.signal_shutdown('keyboard interrupt')

    logger.info("exiting...: %s", rospy.is_shutdown())
    print "exiting..."

if __name__ == "__main__":
    main()
