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
import sys
import traceback

import roslib.packages

import roslaunch.core
from roslaunch.config import ROSLaunchConfig, load_roscore
from roslaunch.launch import ROSLaunchRunner
from roslaunch.pmon import pmon_shutdown as _pmon_shutdown
from roslaunch.xmlloader import *

NAME = 'roslaunch'

## scripts using roslaunch MUST call configure_logging
def configure_logging():
    from roslib.scriptutil import configure_logging
    logfile_name = configure_logging(NAME)
    if logfile_name:
        print "... logging to %s"%logfile_name

def main(argv=sys.argv):
    configure_logging()
    try:
        logger = logging.getLogger('roslaunch')
        roslaunch.core.add_printlog_handler(logger.info)
        roslaunch.core.add_printerrlog_handler(logger.error)        
        
        logger.info("roslaunch starting with args %s"%str(argv))
        from optparse import OptionParser

        parser = OptionParser(usage="usage: %prog [-p|--port=port] [--core] [-u|--server_uri=uri]  [-c|--child_name=name] [files...]", prog=NAME)
        parser.add_option("-c", "--child",
                          dest="child_name", default=None,
                          help="Run as child service 'NAME'. Required with -u", metavar="NAME")
        parser.add_option("-u", "--server_uri",
                          dest="server_uri", default=None,
                          help="URI of server. Required with -c", metavar="URI")
        parser.add_option("-p", "--port",
                          dest="port", default=None,
                          help="master port. Only valid if master is launched", metavar="PORT")
        parser.add_option("--core", action="store_true",
                          dest="core", default=False, 
                          help="Launch core services only")

        (options, args) = parser.parse_args(argv[1:])

        # validate args first so we don't spin up any resources
        if options.child_name:
            if not options.server_uri:
                parser.error("--child option required --server_uri to be set as well")
            if options.port:
                parser.error("port option cannot be used with roslaunch child mode")
            if args:
                parser.error("Input files are not allowed when run in child mode")
        elif options.core:
            if args:
                parser.error("Input files are not allowed when launching core")
                
            # we don't actually do anything special for core as the roscore.xml file
            # is an implicit include for any roslaunch
            
        elif len(args) == 0:
            parser.error("you must specify at least one input file")
        elif [f for f in args if not os.path.exists(f)]:
            parser.error("The following input files do not exist: %s"%f)

        if options.child_name:
            logger.info('starting in child mode')
            roslaunch.core.set_child_mode(True)

            # This is a roslaunch child, spin up client server.
            # client spins up an XML-RPC server that waits for
            # commands and configuration from the server.
            import remote
            remote.run_child(options.child_name, options.server_uri)

        else:
            logger.info('starting in server mode')
            
            # This is the main roslaunch server process. Load up the
            # files specified on the command line and launch the
            # requested resourcs.
            
            config = ROSLaunchConfig()
            loader = XmlLoader()

            # load the roscore file first. we currently have
            # last-declaration wins rules.  roscore is just a
            # roslaunch file with special load semantics
            load_roscore(loader, config)

            for f in args:
                try:
                    logger.info('loading config file %s'%f)
                    loader.load(f, config)
                except XmlParseException, e:
                    print >> sys.stderr, "ERROR", e
                    sys.exit(1)
                except XmlLoadException, e:
                    print >> sys.stderr, "ERROR", e
                    sys.exit(1)

            if options.port:
                logger.info("overriding master port to %s"%options.port)
                config.master.set_port(options.port)
                
            runner = ROSLaunchRunner(config, is_core=options.core)
            runner.launch() 
            runner.spin() #blocks until shutdown

    except RLException, e:
        printerrlog(str(e))
        sys.exit(1)
    except Exception, e:
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
