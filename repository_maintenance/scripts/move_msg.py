#! /usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import with_statement

import os
import time
import sys, string
import subprocess
import roslib
import roslib.scriptutil
import roslib.rosenv

from optparse import OptionParser


def vdmain():
    parser = OptionParser(usage="usage: %prog [options]", prog='move_msg')
    parser.add_option("-s", "--source", dest="source_package", default=False,
                      action="store", type="string", help="the package in which the message is currently")
    parser.add_option("-t", "--target", dest="target_package", default=False,
                      action="store", type="string", help="the package in which the message should be")
    parser.add_option("-m", "--message", dest="message_name", default=False,
                      action="store", type="string", help="the name of the message")
    parser.add_option("--service", dest="service", default=False,
                      action="store_true", help="the item being manipulated is a service")
    parser.add_option("-n", dest="check_only", default=False,
                      action="store_true", help="print but don't evaluate command")

    options, args = parser.parse_args()
    if len(args) > 0:
        out = ["echo"]
        out.extend(args)
        print out
        subprocess.call(["echo", "Got", "args:"])
        subprocess.call(out)
        #    print args

    if not options.source_package:
        print "source_package required"
        sys.exit(-1)
    if not options.target_package:
        print "target_package required"
        sys.exit(-1)
    if not options.source_package:
        print "message_name required"
        sys.exit(-1)

    message_or_service_string = "msg"
    if options.service:
        message_or_service_string = "srv"
            

    #Replace all instances of / or :: seperated source_package message_name pairs in msg srv py c* h*
    # add to manifest


    find_filter = ["-type", "f", "!", "-name", "*.svn*", "-a", "(", "-name", "*.c*", "-o", "-name", "*.h*", "-o", "-name", "*.msg", "-o", "-name", "*.srv", "-o", "-name", "*.py", ")"]
    packages = [options.source_package] + roslib.scriptutil.rospack_depends_on(options.source_package)

    cmd = ["find"]
    paths = []
    for p in packages:
        paths.append(roslib.packages.get_pkg_dir(p))

        
    cpp_namespace_exec_rule = ["-exec", "sed", "-i",  "s/%s::%s/%s::%s/g"%(options.source_package, options.message_name, options.target_package, options.message_name), "{}", ";"]
    #below also catches msg and srv files
    cpp_include_exec_rule = ["-exec", "sed", "-i", "s/%s\/%s/%s\/%s/g"%(options.source_package, options.message_name, options.target_package, options.message_name), "{}", ";"]
    
    ##\todo python changes for fully qualified
    python_full_exec_rule = ["-exec", "sed", "-i", "s/%s\.%s\.%s/%s\.%s\.%s/g"%(options.source_package, message_or_service_string, options.message_name, options.target_package, message_or_service_string, options.message_name), "{}", ";"] #\todo update manifest automatically to assert it includes a dependency on the new package

    cmd.extend(paths)
    cmd.extend(find_filter)
    cmd.extend(cpp_namespace_exec_rule)
    cmd.extend(cpp_include_exec_rule)
    cmd.extend(python_full_exec_rule)
    #    print cmd
    if options.check_only:
        out = ["echo"]
        out.extend(cmd)
        subprocess.call(out)
    else:
        subprocess.call(cmd)



if __name__ == '__main__':
    start_time = time.time()
    vdmain()
    print "Elapsed Time: %.2f seconds."%(time.time() - start_time)
