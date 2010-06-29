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

import os
import logging
import sys

import roslaunch.core

# symbol exports
from roslaunch.core import Node, Test, Master, RLException
from roslaunch.config import ROSLaunchConfig
from roslaunch.launch import ROSLaunchRunner
from roslaunch.xmlloader import XmlLoader, XmlParseException

# script api
from roslaunch.scriptapi import ROSLaunch
from roslaunch.pmon import Process

NAME = 'roslaunch'

def configure_logging(uuid):
    """
    scripts using roslaunch MUST call configure_logging
    """
    try:
        import socket
        import roslib.roslogging
        logfile_basename = os.path.join(uuid, '%s-%s-%s.log'%(NAME, socket.gethostname(), os.getpid()))
        # additional: names of python packages we depend on that may also be logging
        logfile_name = roslib.roslogging.configure_logging(NAME, filename=logfile_basename, additional=['paramiko', 'roslib', 'rospy'])
        if logfile_name:
            print "... logging to %s"%logfile_name

        # add logger to internal roslaunch logging infrastructure
        logger = logging.getLogger('roslaunch')
        roslaunch.core.add_printlog_handler(logger.info)
        roslaunch.core.add_printerrlog_handler(logger.error)
    except:
        print >> sys.stderr, "WARNING: unable to configure logging. No log files will be generated"
        
def _get_optparse():
    from optparse import OptionParser

    parser = OptionParser(usage="usage: %prog [options] [package] <filename>", prog=NAME)
    parser.add_option("--args",
                      dest="node_args", default=None,
                      help="Print command-line arguments for node", metavar="NODE_NAME")
    parser.add_option("--nodes",
                      dest="node_list", default=False, action="store_true",
                      help="Print list of node names in launch file")
    parser.add_option("--find-node",
                      dest="find_node", default=None, 
                      help="Find launch file that node is defined in", metavar="NODE_NAME")
    parser.add_option("-c", "--child",
                      dest="child_name", default=None,
                      help="Run as child service 'NAME'. Required with -u", metavar="NAME")
    parser.add_option("--local",
                      dest="local_only", default=False, action="store_true",
                      help="Do not launch remote nodes")
    # #2370
    parser.add_option("--screen",
                      dest="force_screen", default=False, action="store_true",
                      help="Force output of all local nodes to screen")
    parser.add_option("-u", "--server_uri",
                      dest="server_uri", default=None,
                      help="URI of server. Required with -c", metavar="URI")
    parser.add_option("--run_id",
                      dest="run_id", default=None,
                      help="run_id of session. Required with -c", metavar="RUN_ID")
    # #1254: wait until master comes online before starting
    parser.add_option("--wait", action="store_true",
                      dest="wait_for_master", default=False,
                      help="wait for master to start before launching")
    parser.add_option("-p", "--port",
                      dest="port", default=None,
                      help="master port. Only valid if master is launched", metavar="PORT")
    parser.add_option("--core", action="store_true",
                      dest="core", default=False, 
                      help="Launch core services only")
    parser.add_option("--pid",
                      dest="pid_fn", default="",
                      help="write the roslaunch pid to filename")
    parser.add_option("-v", action="store_true",
                      dest="verbose", default=False,
                      help="verbose printing")
    return parser
    
def _validate_args(parser, options, args):
    # validate args first so we don't spin up any resources
    if options.child_name:
        if not options.server_uri:
            parser.error("--child option requires --server_uri to be set as well")
        if not options.run_id:
            parser.error("--child option requires --run_id to be set as well")                
        if options.port:
            parser.error("port option cannot be used with roslaunch child mode")
        if args:
            parser.error("Input files are not allowed when run in child mode")
    elif options.core:
        if args:
            parser.error("Input files are not allowed when launching core")
        if options.run_id:
            parser.error("--run_id should only be set for child roslaunches (-c)")
                
        # we don't actually do anything special for core as the roscore.xml file
        # is an implicit include for any roslaunch

    elif len(args) == 0:
        parser.error("you must specify at least one input file")
    elif [f for f in args if not os.path.exists(f)]:
        parser.error("The following input files do not exist: %s"%f)

    if len([x for x in [options.node_list, options.find_node, options.node_args] if x]) > 1:
        parser.error("only one of [--nodes, --find-node, --args] may be specified")
    
def main(argv=sys.argv):
    options = None
    try:
        import roslaunch.rlutil
        parser = _get_optparse()
        
        (options, args) = parser.parse_args(argv[1:])
        args = roslaunch.rlutil.resolve_launch_arguments(args)
        _validate_args(parser, options, args)

        # node args doesn't require any roslaunch infrastructure, so process it first
        if options.node_args or options.node_list or options.find_node:
            if options.node_args and not args:
                parser.error("please specify a launch file")
            import roslaunch.node_args
            if options.node_args:
                roslaunch.node_args.print_node_args(options.node_args, args)
            elif options.find_node:
                roslaunch.node_args.print_node_filename(options.find_node, args)                
            else:
                roslaunch.node_args.print_node_list(args)
            return

        # we have to wait for the master here because we don't have the run_id yet
        if options.wait_for_master:
            if options.core:
                parser.error("--wait cannot be used with roscore")
            roslaunch.rlutil._wait_for_master()            

        # write the pid to a file
        if options.pid_fn:
          open(options.pid_fn, "w").write(str(os.getpid()))

        # spin up the logging infrastructure. have to wait until we can read options.run_id
        uuid = roslaunch.rlutil.get_or_generate_uuid(options.run_id, options.wait_for_master)
        configure_logging(uuid)

        # #2761
        roslaunch.rlutil.check_log_disk_usage()

        logger = logging.getLogger('roslaunch')
        logger.info("roslaunch starting with args %s"%str(argv))
        logger.info("roslaunch env is %s"%os.environ)
            
        if options.child_name:
            logger.info('starting in child mode')

            # This is a roslaunch child, spin up client server.
            # client spins up an XML-RPC server that waits for
            # commands and configuration from the server.
            import roslaunch.child
            c = roslaunch.child.ROSLaunchChild(uuid, options.child_name, options.server_uri)
            c.run()
        else:
            logger.info('starting in server mode')

            # #1491 change terminal name
            roslaunch.rlutil.change_terminal_name(args, options.core)
            
            # This is a roslaunch parent, spin up parent server and launch processes.
            # args are the roslaunch files to load
            import roslaunch.parent
            try:
              p = roslaunch.parent.ROSLaunchParent(uuid, args, is_core=options.core, port=options.port, local_only=options.local_only, verbose=options.verbose, force_screen=options.force_screen)
              p.start()
              p.spin()
            finally:
              # remove the pid file
              if options.pid_fn:
                try: os.unlink(options.pid_fn)
                except os.error, reason: pass

    except RLException, e:
        roslaunch.core.printerrlog(str(e))
        sys.exit(1)
    except Exception, e:
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
