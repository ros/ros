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

"""Command-line handler for ROS zenmaster (Python Master)"""

import logging
import os
import sys
import time
import optparse

import rosmaster.master

def configure_logging():
    """
    Setup filesystem logging for the master
    """
    filename = 'master.log'
    # #988 __log command-line remapping argument
    import rosgraph.names
    import rosgraph.roslogging
    mappings = rosgraph.names.load_mappings(sys.argv)
    if '__log' in mappings:
        logfilename_remap = mappings['__log']
        filename = os.path.abspath(logfilename_remap)
    _log_filename = rosgraph.roslogging.configure_logging('rosmaster', logging.DEBUG, filename=filename)

def rosmaster_main(argv=sys.argv, stdout=sys.stdout, env=os.environ):
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
    configure_logging()   
    
    port = rosmaster.master.DEFAULT_MASTER_PORT
    if options.port:
        port = int(options.port)

    if not options.core:
        print """


ACHTUNG WARNING ACHTUNG WARNING ACHTUNG
WARNING ACHTUNG WARNING ACHTUNG WARNING


Standalone zenmaster has been deprecated, please use 'roscore' instead


ACHTUNG WARNING ACHTUNG WARNING ACHTUNG
WARNING ACHTUNG WARNING ACHTUNG WARNING


"""

    logger = logging.getLogger("rosmaster.main")
    logger.info("initialization complete, waiting for shutdown")
    try:
        logger.info("Starting ROS Master Node")
        master = rosmaster.master.Master(port)
        master.start()

        import time
        while master.ok():
            time.sleep(.1)
    except KeyboardInterrupt:
        logger.info("keyboard interrupt, will exit")
    finally:
        logger.info("stopping master...")
        master.stop()

if __name__ == "__main__":
    main()
